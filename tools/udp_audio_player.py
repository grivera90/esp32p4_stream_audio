#!/usr/bin/env python3
"""
Receptor UDP del stream ESP32-P4 (audio_udp_streamer).

Protocolo (little-endian, packed, 12 bytes + PCM):
  uint16 magic          0x5541 ('A''U')
  uint16 seq
  uint32 timestamp_ms
  uint8  channels
  uint8  format         1 = PCM S16LE interleaved
  uint16 samples_per_ch
  int16  pcm[]          channels * samples_per_ch

Firmware actual: 48 kHz, 2 canales, destino 192.168.100.68:5000.

Uso:
  python3 -m pip install -r tools/requirements.txt
  python3 tools/udp_audio_player.py
  python3 tools/udp_audio_player.py --port 5000 --rate 48000 --headphones
  python3 tools/udp_audio_player.py --save capture.wav --no-play
"""

from __future__ import annotations

import argparse
import collections
import os
import socket
import struct
import subprocess
import sys
import threading
import time
import wave
from typing import Optional

import numpy as np

try:
    import sounddevice as sd
except ImportError:
    sd = None

HDR_FMT = "<HHIBBH"
HDR_SIZE = struct.calcsize(HDR_FMT)
MAGIC = 0x5541
FMT_PCM_S16LE = 1
MAX_DATAGRAM = 2048
PLAY_QUEUE_MAX = 12


class StreamStats:
    def __init__(self) -> None:
        self.pkts = 0
        self.bytes = 0
        self.lost = 0
        self.bad = 0
        self.peak = 0
        self.rms = 0.0
        self.last_seq: Optional[int] = None
        self.t0 = time.monotonic()
        self.lock = threading.Lock()

    def on_ok(self, seq: int, nbytes: int, peak: int = 0, rms: float = 0.0) -> None:
        with self.lock:
            if self.last_seq is not None:
                gap = (seq - self.last_seq - 1) & 0xFFFF
                if gap > 0 and gap < 4096:
                    self.lost += gap
            self.last_seq = seq
            self.pkts += 1
            self.bytes += nbytes
            self.peak = peak
            self.rms = rms

    def on_bad(self) -> None:
        with self.lock:
            self.bad += 1

    def snapshot(self) -> str:
        with self.lock:
            dt = max(time.monotonic() - self.t0, 1e-6)
            kbps = (self.bytes * 8.0) / dt / 1000.0
            return (
                f"pkts={self.pkts}  lost={self.lost}  bad={self.bad}  "
                f"rate={kbps:.1f} kbps  peak={self.peak}  rms={self.rms:.0f}  "
                f"elapsed={dt:.0f}s"
            )


def parse_packet(data: bytes) -> Optional[tuple[int, int, int, int, np.ndarray]]:
    if len(data) < HDR_SIZE:
        return None

    magic, seq, ts_ms, channels, fmt, samples_per_ch = struct.unpack_from(HDR_FMT, data, 0)
    if magic != MAGIC or fmt != FMT_PCM_S16LE or channels not in (1, 2, 4):
        return None
    if samples_per_ch == 0:
        return None

    pcm_bytes = samples_per_ch * channels * 2
    if len(data) < HDR_SIZE + pcm_bytes:
        return None

    pcm = np.frombuffer(data, dtype="<i2", offset=HDR_SIZE, count=samples_per_ch * channels)
    frames = pcm.reshape(samples_per_ch, channels)
    return seq, ts_ms, channels, samples_per_ch, frames.copy()


def map_channels(frames: np.ndarray, out_ch: int) -> np.ndarray:
    in_ch = frames.shape[1]
    if in_ch == out_ch:
        return frames
    if out_ch == 1:
        return frames.mean(axis=1, keepdims=True).astype(np.int16)
    if out_ch == 2:
        if in_ch == 1:
            return np.repeat(frames, 2, axis=1)
        return frames[:, :2]
    if in_ch == 2:
        return np.concatenate((frames, frames), axis=1)
    return frames[:, :out_ch]


def pick_output_device(explicit: Optional[int]) -> Optional[int]:
    if sd is None or explicit is not None:
        return explicit
    devices = sd.query_devices()
    preferred = []
    for i, dev in enumerate(devices):
        if dev["max_output_channels"] < 2:
            continue
        name = dev["name"].lower()
        # ALC analog hw:* often reports 0 out; PipeWire "default" llega a parlantes/jack
        if name == "default":
            preferred.append((0, i, dev["name"]))
        elif name == "pipewire":
            preferred.append((1, i, dev["name"]))
        elif "hdmi" in name or "nvidia" in name:
            continue
        else:
            preferred.append((2, i, dev["name"]))
    if not preferred:
        return None
    preferred.sort()
    idx = preferred[0][1]
    print(f"Dispositivo de salida: [{idx}] {preferred[0][2]}")
    return idx


def _amixer_card1(*args: str) -> None:
    try:
        subprocess.run(["amixer", "-c", "1", "-q", *args], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except OSError:
        pass


def enable_notebook_speakers(volume_pct: int = 100) -> None:
    """Desmutea Speaker/Master y desactiva Auto-Mute para salir por parlantes."""
    if os.name == "nt":
        print("Windows: se usa el volumen y dispositivo configurados en el sistema.")
        return
    vol = max(5, min(100, int(volume_pct)))
    _amixer_card1("sset", "Auto-Mute Mode", "Disabled")
    _amixer_card1("sset", "Speaker", f"{vol}%", "unmute")
    _amixer_card1("sset", "Headphone", "0%", "mute")
    _amixer_card1("sset", "Master", f"{vol}%", "unmute")
    _amixer_card1("sset", "PCM", "100%")
    try:
        subprocess.run(["wpctl", "set-mute", "@DEFAULT_AUDIO_SINK@", "0"], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run(["wpctl", "set-volume", "@DEFAULT_AUDIO_SINK@", f"{vol / 100.0:.2f}"], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except OSError:
        pass
    print(f"Salida de audio: parlantes unmute ~{vol}%")


def enable_headphones_only(volume_pct: int = 100) -> None:
    """Mutea parlantes y sale solo por el jack de auriculares (ALC233 card 1)."""
    if os.name == "nt":
        print("Windows: --headphones no puede mutear otros dispositivos; selecciona la salida con --device.")
        return
    vol = max(5, min(100, int(volume_pct)))
    _amixer_card1("sset", "Speaker", "0%", "mute")
    _amixer_card1("sset", "Headphone", f"{vol}%", "unmute")
    _amixer_card1("sset", "Master", f"{vol}%", "unmute")
    _amixer_card1("sset", "PCM", "100%")
    _amixer_card1("sset", "Auto-Mute Mode", "Enabled")
    try:
        subprocess.run(["wpctl", "set-mute", "@DEFAULT_AUDIO_SINK@", "0"], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run(["wpctl", "set-volume", "@DEFAULT_AUDIO_SINK@", f"{vol / 100.0:.2f}"], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except OSError:
        pass
    print(f"Salida de audio: solo auriculares ~{vol}% (parlantes mute)")


def apply_gain(frames: np.ndarray, gain: float) -> np.ndarray:
    if gain == 1.0:
        return frames
    scaled = np.clip(frames.astype(np.float32) * gain, -32768, 32767)
    return scaled.astype(np.int16)


def apply_dc_block(frames: np.ndarray, state: list[float], alpha: float = 0.995) -> np.ndarray:
    """One-pole HPF per channel: quita DC / rumble que suena a 'ruido' constante."""
    x = frames.astype(np.float32)
    y = np.empty_like(x)
    for ch in range(x.shape[1]):
        prev_x = state[ch * 2]
        prev_y = state[ch * 2 + 1]
        for i in range(x.shape[0]):
            cur = x[i, ch]
            out = cur - prev_x + alpha * prev_y
            prev_x = cur
            prev_y = out
            y[i, ch] = out
        state[ch * 2] = prev_x
        state[ch * 2 + 1] = prev_y
    return np.clip(y, -32768, 32767).astype(np.int16)


class Player:
    def __init__(self, rate: int, channels: int, device: Optional[int]) -> None:
        if sd is None:
            raise RuntimeError("Falta sounddevice. Instalar: pip install -r tools/requirements.txt")
        self.rate = rate
        self.channels = channels
        self.q: collections.deque[np.ndarray] = collections.deque()
        self.lock = threading.Lock()
        self.underruns = 0
        self.primed = False
        self.stream = sd.OutputStream(
            samplerate=rate,
            channels=channels,
            dtype="int16",
            device=device,
            callback=self._callback,
            blocksize=1024,
            latency=0.08,
        )

    def start(self) -> None:
        self.stream.start()

    def stop(self) -> None:
        self.stream.stop()
        self.stream.close()

    def push(self, frames: np.ndarray) -> None:
        mapped = map_channels(frames, self.channels)
        with self.lock:
            if len(self.q) >= PLAY_QUEUE_MAX:
                self.q.popleft()
            self.q.append(mapped)
            if not self.primed and len(self.q) >= 4:
                self.primed = True

    def _callback(self, outdata, frames, time_info, status) -> None:
        needed = frames
        pos = 0
        outdata.fill(0)
        with self.lock:
            if not self.primed:
                return
            while needed > 0 and self.q:
                chunk = self.q[0]
                take = min(needed, chunk.shape[0])
                outdata[pos : pos + take, :] = chunk[:take, :]
                if take == chunk.shape[0]:
                    self.q.popleft()
                else:
                    self.q[0] = chunk[take:, :]
                pos += take
                needed -= take
            if needed > 0:
                self.underruns += 1
                self.primed = False


def list_local_ipv4() -> list[tuple[str, str]]:
    if os.name == "nt":
        found = []
        seen = set()
        try:
            addresses = socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET)
            for address in addresses:
                ip = address[4][0]
                if not ip.startswith("127.") and ip not in seen:
                    seen.add(ip)
                    found.append(("Windows", ip))
        except OSError:
            pass
        return found

    found: list[tuple[str, str]] = []
    for _idx, name in socket.if_nameindex():
        if name == "lo":
            continue
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                ifr = struct.pack("256s", name.encode("utf-8")[:15])
                import fcntl

                res = fcntl.ioctl(sock.fileno(), 0x8915, ifr)
                ip = socket.inet_ntoa(res[20:24])
                found.append((name, ip))
            finally:
                sock.close()
        except OSError:
            pass
    return found


def primary_ipv4() -> Optional[str]:
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.connect(("8.8.8.8", 80))
            return sock.getsockname()[0]
        finally:
            sock.close()
    except OSError:
        return None


def print_local_ips(port: int) -> None:
    ifaces = list_local_ipv4()
    primary = primary_ipv4()
    print("------------------------------------------------")
    print("IP de esta notebook (dest_ip del firmware ESP32-P4)")
    if not ifaces:
        print("  (no se detectaron IPv4; revisa la conexion de red)")
    for name, ip in ifaces:
        mark = "  <-- usar esta" if primary and ip == primary else ""
        print(f"  {name:12} {ip}{mark}")
    if primary:
        print()
        print("  Copiar en main.c:")
        print(f'    .dest_ip   = "{primary}",')
        print(f"    .dest_port = {port},")
    print("------------------------------------------------")


def open_wav(path: str, rate: int, channels: int) -> wave.Wave_write:
    wf = wave.open(path, "wb")
    wf.setnchannels(channels)
    wf.setsampwidth(2)
    wf.setframerate(rate)
    return wf


def main() -> int:
    ap = argparse.ArgumentParser(description="Reproduce el stream UDP de audio del ESP32-P4")
    ap.add_argument("--bind", default="0.0.0.0", help="IP local de escucha (default 0.0.0.0)")
    ap.add_argument("--port", type=int, default=5000, help="Puerto UDP (debe coincidir con dest_port del firmware)")
    ap.add_argument("--rate", type=int, default=48000, help="Sample rate Hz (debe coincidir con sample_rate_hz)")
    ap.add_argument("--play-channels", type=int, default=0, choices=(0, 1, 2, 4),
                    help="Canales de salida (0 = usar los del paquete)")
    ap.add_argument("--device", type=int, default=None, help="Indice del dispositivo sounddevice (default: Analog/ALC)")
    ap.add_argument("--gain", type=float, default=1.0, help="Ganancia lineal (1.0 = sin amplificar)")
    ap.add_argument("--channel", type=int, default=-1, choices=(-1, 0, 1, 2, 3, 99),
                    help="Canal a escuchar: -1=stereo/directo (default), 0..3=solo mic 0..3, 99=mezcla (downmix de todos a mono)")
    ap.add_argument("--dc-block", action="store_true", help="HPF extra en el player (off por default)")
    ap.add_argument("--speaker-vol", type=int, default=100, help="Volumen %% de salida (default 100)")
    ap.add_argument("--headphones", action="store_true", help="Solo jack de auriculares; en Windows selecciona la salida con --device")
    ap.add_argument("--no-unmute", action="store_true", help="No tocar amixer/wpctl (si ya tenes el mixer como queres)")
    ap.add_argument("--list-devices", action="store_true", help="Listar dispositivos de audio y salir")
    ap.add_argument("--save", default=None, help="Guardar PCM en WAV")
    ap.add_argument("--no-play", action="store_true", help="Solo recibir/estadisticas (y --save si se pide)")
    ap.add_argument("--stats-s", type=float, default=2.0, help="Periodo de stats en segundos")
    args = ap.parse_args()

    if args.list_devices:
        if sd is None:
            print("Falta sounddevice", file=sys.stderr)
            return 1
        print(sd.query_devices())
        return 0

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 256 * 1024)
    sock.bind((args.bind, args.port))
    sock.settimeout(0.5)

    stats = StreamStats()
    player: Optional[Player] = None
    wav: Optional[wave.Wave_write] = None
    wav_ch: Optional[int] = None
    stop = threading.Event()
    dc_state = [0.0] * 8

    print_local_ips(args.port)
    print(f"Escuchando UDP {args.bind}:{args.port}  rate={args.rate} Hz  gain={args.gain}  channel={args.channel}")
    if not args.no_unmute and not args.no_play:
        if args.headphones:
            enable_headphones_only(args.speaker_vol)
        else:
            enable_notebook_speakers(args.speaker_vol)
    print("Ctrl+C para salir")

    last_stats = time.monotonic()
    try:
        while not stop.is_set():
            try:
                data, addr = sock.recvfrom(MAX_DATAGRAM)
            except socket.timeout:
                now = time.monotonic()
                if now - last_stats >= args.stats_s:
                    print(f"[stats] {stats.snapshot()}  src=—")
                    last_stats = now
                continue
            except KeyboardInterrupt:
                break

            parsed = parse_packet(data)
            if parsed is None:
                stats.on_bad()
                continue

            seq, ts_ms, channels, samples_per_ch, frames = parsed

            # Grabacion en WAV del audio original multi-canal antes de cualquier downmix o filtro
            if args.save:
                if wav is None:
                    wav = open_wav(args.save, args.rate, frames.shape[1])
                    print(f"Grabando {args.save} ({frames.shape[1]} ch crudos)")
                if frames.shape[1] == wav.getnchannels():
                    wav.writeframes(frames.tobytes())

            # Seleccion o mezcla de canales para reproduccion / monitoreo
            if args.channel in (0, 1, 2, 3):
                if args.channel < frames.shape[1]:
                    mono = frames[:, args.channel]
                    frames = np.stack((mono, mono), axis=1)
                else:
                    print(f"Advertencia: mic {args.channel} no disponible en stream de {frames.shape[1]} ch", file=sys.stderr)
            elif args.channel == 99:
                # Downmix / promedio de todos los canales recibidos
                mono = frames.mean(axis=1).astype(np.int16)
                frames = np.stack((mono, mono), axis=1)

            if args.dc_block:
                frames = apply_dc_block(frames, dc_state)

            peak = int(np.max(np.abs(frames))) if frames.size else 0
            rms = float(np.sqrt(np.mean(frames.astype(np.float32) ** 2))) if frames.size else 0.0
            stats.on_ok(seq, len(data), peak, rms)

            if player is None and not args.no_play:
                out_ch = args.play_channels if args.play_channels else min(frames.shape[1], 2)
                try:
                    dev = pick_output_device(args.device)
                    player = Player(args.rate, out_ch, dev)
                    player.start()
                    print(f"Playback: {args.rate} Hz, {out_ch} ch, gain={args.gain}, channel={args.channel}  (stream={channels} ch, src={addr[0]})")
                except Exception as exc:
                    print(f"No se pudo abrir audio ({exc}). Sigo en --no-play.", file=sys.stderr)
                    args.no_play = True

            if player is not None:
                player.push(apply_gain(frames, args.gain))

            now = time.monotonic()
            if now - last_stats >= args.stats_s:
                extra = ""
                if player is not None:
                    extra = f"  underruns={player.underruns}  q={len(player.q)}"
                print(f"[stats] {stats.snapshot()}  src={addr[0]}  seq={seq}  ts={ts_ms}{extra}")
                last_stats = now
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        if player is not None:
            player.stop()
        if wav is not None:
            wav.close()
        print(f"Fin  {stats.snapshot()}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
