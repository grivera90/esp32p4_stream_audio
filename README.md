# esp32p4_stream_audio

Firmware ESP-IDF para **ESP32-P4** que captura audio de micrófonos I2S (**INMP441** o **SPH0645**) y lo envía por **UDP** (PCM s16le) a una PC en la misma red Wi‑Fi.

Soporta **2 o 4 micrófonos** (1 bus I2S o 2 buses sincronizados). El P4 no tiene radio Wi‑Fi propia: usa el coprocesador (C6/C2) vía `esp_wifi_remote` / `esp_hosted`.

```
Mic(s) I2S → DMA → int16 → UDP → PC (udp_audio_player.py)
```

## Requisitos

- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32p4/get-started/) ≥ 5.5
- Target: `esp32p4` (rev &lt; 3.0; ver `sdkconfig.defaults`)
- PC en la misma Wi‑Fi que el ESP, con Python 3

## Configurar firmware

### Wi‑Fi

En `wifi_manager/wifi_connect.h`:

```c
#define WIFI_SSID     "tu_ssid"
#define WIFI_PASSWORD "tu_password"
```

### Micrófonos y canales

| Parámetro | Archivo | Valores |
|-----------|---------|---------|
| Modelo de mic | `audio_capture/bsp_audio.c` → `BSP_MIC_TYPE` | `BSP_MIC_INMP441` (default) o `BSP_MIC_SPH0645` |
| Cantidad de mics | `main/main.c` → `ACTIVE_MICS` | `2` (bus0: L/R) o `4` (bus0 + bus1: Mic0–Mic3) |
| Sample rate | `bsp_audio.c` → `BSP_*_SAMPLE_RATE_HZ` | Hoy `48000` en ambos modelos |
| Destino UDP | `main/main.c` → `dest_ip` / `dest_port` | IP que imprime el player al arrancar |

Con 4 mics el orden en el stream es: **Mic0, Mic1** (bus0 L/R), **Mic2, Mic3** (bus1 L/R).

El `--rate` del player debe coincidir con el sample rate del BSP.

## Uso rápido

### 1. Receptor en la PC (antes de flashear / resetear el ESP)

```bash
python3 -m venv tools/.venv
tools/.venv/bin/pip install -r tools/requirements.txt
# Linux:
sudo apt install libportaudio2

tools/.venv/bin/python tools/udp_audio_player.py --port 5000 --rate 48000 --headphones
```

Copiá la IP que muestra el script en `.dest_ip` de `main/main.c`. Dejá el player corriendo.

### 2. Compilar y flashear

```bash
idf.py set-target esp32p4
idf.py build flash monitor
```

En el monitor deberías ver `UDP stream -> <ip>:5000` y `Pipeline ready`.

### 3. Verificar

El player imprime stats cada ~2 s (`pkts`, `lost`, `rate`). Si `pkts` sube, el stream llega.

## Player Python (`udp_audio_player.py`)

Decodifica paquetes UDP (header `AU` + PCM s16le interleaved) y reproduce, graba o monitorea por canal.

### Opciones principales

| Flag | Descripción |
|------|-------------|
| `--port`, `--rate` | Deben coincidir con `dest_port` y sample rate del firmware |
| `--channel N` | Escuchar un mic: `0`–`3` (Mic0–Mic3). `99` = mezcla todos a mono. `-1` = directo (default) |
| `--gain` | Ganancia lineal (ej. `2.0`) |
| `--dc-block` | Filtro pasa-altos extra (ruido DC / rumble) |
| `--headphones` | Salida solo por jack (mutea parlantes en ALC233) |
| `--speaker-vol N` | Volumen % (default 100) |
| `--device N` | Índice de salida sounddevice |
| `--play-channels` | Canales de salida: `0` = auto, `1`, `2` o `4` |
| `--save archivo.wav` | Graba WAV **multi-canal crudo** (2 o 4 ch, antes de downmix) |
| `--no-play` | Solo recibir stats / grabar, sin reproducir |
| `--list-devices` | Lista dispositivos de audio y sale |

### Ejemplos

```bash
# Reproducción normal (2 o 4 ch según el firmware)
tools/.venv/bin/python tools/udp_audio_player.py --port 5000 --rate 48000 --headphones

# Escuchar solo Mic2 (útil con ACTIVE_MICS=4)
tools/.venv/bin/python tools/udp_audio_player.py --channel 2 --headphones

# Mezclar los 4 mics a mono
tools/.venv/bin/python tools/udp_audio_player.py --channel 99 --headphones

# Grabar los 4 canales sin reproducir
tools/.venv/bin/python tools/udp_audio_player.py --no-play --save capture_4mics.wav

# Grabar mientras reproduce
tools/.venv/bin/python tools/udp_audio_player.py --save capture.wav

# Listar salidas de audio
tools/.venv/bin/python tools/udp_audio_player.py --list-devices
```

## Estructura

| Path | Rol |
|------|-----|
| `main/` | App: Wi‑Fi → BSP audio → streamer UDP |
| `audio_capture/` | BSP I2S (INMP441 / SPH0645, 2 o 4 ch) |
| `audio_udp_streamer/` | Empaquetado AU + PCM y envío UDP |
| `wifi_manager/` | STA (Wi‑Fi remoto) |
| `tools/udp_audio_player.py` | Receptor, reproducción por canal, WAV |

## Si no llega audio

- `dest_ip` desactualizada (DHCP)
- `--rate` distinto al sample rate del BSP
- Notebook y ESP en SSIDs distintos
- Firewall bloqueando UDP 5000
- Player no estaba escuchando al arrancar el ESP → reiniciar el ESP con el player ya abierto
- Con 4 mics: `--channel 2` o `3` solo funcionan si `ACTIVE_MICS=4`

Detalle ampliado: `tools/INSTRUCTIVO.txt`.
