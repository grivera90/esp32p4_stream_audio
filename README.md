# esp32p4_stream_audio

Firmware ESP-IDF para **ESP32-P4** que captura audio de micrófonos I2S (SPH0645, 48 kHz, 2 canales) y lo envía por **UDP** (PCM s16le) a una PC en la misma red Wi‑Fi.

El P4 no tiene radio Wi‑Fi propia: usa el coprocesador (C6/C2) vía `esp_wifi_remote` / `esp_hosted`.

```
Mic SPH0645 → I2S DMA → int16 → UDP → PC (udp_audio_player.py)
```

## Requisitos

- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32p4/get-started/) ≥ 5.5
- Target: `esp32p4` (rev &lt; 3.0; ver `sdkconfig.defaults`)
- PC en la misma Wi‑Fi que el ESP, con Python 3

## Configurar

1. **Wi‑Fi** — en `wifi_manager/wifi_connect.h`:

   ```c
   #define WIFI_SSID     "tu_ssid"
   #define WIFI_PASSWORD "tu_password"
   ```

2. **Destino UDP** — en `main/main.c` (`dest_ip` / `dest_port`). La IP la imprime el player al arrancar.

3. **Sample rate** — debe coincidir en firmware (`BSP_SPH0645_SAMPLE_RATE_HZ` en `audio_capture/bsp_audio.c`, hoy `48000`) y en el player (`--rate`).

## Uso rápido

### 1. Receptor en la PC (antes de flashear / resetear el ESP)

```bash
python3 -m venv tools/.venv
tools/.venv/bin/pip install -r tools/requirements.txt
# Linux:
sudo apt install libportaudio2

tools/.venv/bin/python tools/udp_audio_player.py --port 5000 --rate 48000 --headphones
```

Copia la IP que muestra el script en `.dest_ip` de `main/main.c`. Dejá el player corriendo.

### 2. Compilar y flashear

```bash
idf.py set-target esp32p4
idf.py build flash monitor
```

En el monitor deberías ver algo como `UDP stream -> <ip>:5000` y `Pipeline ready`.

### 3. Verificar

El player imprime stats cada ~2 s (`pkts`, `lost`, `rate`). Si `pkts` sube, el stream llega.

Opciones útiles:

```bash
tools/.venv/bin/python tools/udp_audio_player.py --list-devices
tools/.venv/bin/python tools/udp_audio_player.py --save capture.wav
tools/.venv/bin/python tools/udp_audio_player.py --no-play --save capture.wav
```

## Estructura

| Path | Rol |
|------|-----|
| `main/` | App: Wi‑Fi → BSP audio → streamer UDP |
| `audio_capture/` | BSP I2S / mics |
| `audio_udp_streamer/` | Empaquetado AU + PCM y envío UDP |
| `wifi_manager/` | STA (Wi‑Fi remoto) |
| `tools/udp_audio_player.py` | Receptor / reproducción / WAV |

## Si no llega audio

- `dest_ip` desactualizada (DHCP)
- Notebook y ESP en SSIDs distintos
- Firewall bloqueando UDP 5000
- Player no estaba escuchando al arrancar el ESP → reiniciar el ESP con el player ya abierto

Detalle ampliado: `tools/INSTRUCTIVO.txt`.
