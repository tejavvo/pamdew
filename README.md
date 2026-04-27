# PAMDEW - ESP32 Multimodal IoT Health Sensor Hub

PAMDEW is an ESP32-based IoT sensor hub for collecting multimodal physiological and motion signals. It combines ECG, PPG, SpO2-style estimation, accelerometer motion, GSR, and temperature sensing into a FreeRTOS firmware pipeline, displays live readings on an SSD1306 OLED, sends JSON telemetry over WiFi, and runs a lightweight TinyML-style feature/inference layer on-device.

> Important: this is an educational/research prototype, not a medical device. The values and ML scores are not clinically validated and must not be used for diagnosis, treatment, or safety-critical decisions.

## Features

- ESP32 firmware using ESP-IDF through PlatformIO.
- FreeRTOS task-based architecture with separate tasks for ECG, PPG, motion, GSR, temperature, OLED, reporter, and ML sampling.
- MAX30102 PPG capture with custom BPM/SpO2-style calculation and signal validation.
- AD8232 ECG capture at 250 Hz with Pan-Tompkins QRS detection and HRV statistics.
- MPU6050 accelerometer capture for motion/artifact analysis.
- GSR and LM35 analog sensing through ADC1.
- SSD1306 OLED live display.
- WiFi HTTP JSON telemetry for dashboard integration.
- Local Python receiver that logs incoming telemetry to JSONL and CSV.
- On-device TinyML-style feature extraction and inference for stress/artifact/anomaly/reliability scoring.
- Runtime sensor calibration before tasks start.
- Menuconfig options for WiFi credentials, JSON logging, and ML logging.

## Repository Layout

```text
.
├── README.md
├── esp1/
│   ├── platformio.ini
│   ├── server.py
│   ├── src/
│   │   ├── main.c
│   │   ├── ml_model.c
│   │   ├── ml_model.h
│   │   ├── ssd1306.c
│   │   ├── ssd1306.h
│   │   ├── Kconfig.projbuild
│   │   └── CMakeLists.txt
│   └── lib/
│       ├── calibration/
│       ├── pan_tompkins/
│       └── wifi_tx/
└── esp2/
```

`esp1` is the main firmware project. `server.py` is a simple development receiver/logger for the JSON telemetry stream.

## Hardware

Target board:

- ESP32 development board (`esp32dev` in PlatformIO).

Sensors/modules:

- MAX30102 pulse oximeter / PPG sensor.
- MPU6050 accelerometer/gyro module.
- AD8232 ECG module.
- GSR analog sensor.
- LM35 temperature sensor.
- SSD1306 OLED display.

## Pin Map

### Sensor I2C Bus

| Signal | ESP32 pin |
| --- | --- |
| I2C0 SDA | GPIO21 |
| I2C0 SCL | GPIO22 |
| MAX30102 address | `0x57` |
| MPU6050 address | `0x68` |

### OLED I2C Bus

| Signal | ESP32 pin |
| --- | --- |
| OLED SDA | GPIO25 |
| OLED SCL | GPIO26 |
| OLED address | `0x3C` |

### Interrupt Pins

| Module | ESP32 pin |
| --- | --- |
| MAX30102 INT | GPIO4 |
| MPU6050 INT | GPIO5 |

### Analog / ECG Pins

| Signal | ESP32 pin / ADC channel |
| --- | --- |
| AD8232 OUT | GPIO34 / ADC1 CH6 |
| GSR OUT | GPIO35 / ADC1 CH7 |
| LM35 OUT | GPIO36 / ADC1 CH0 |
| AD8232 LO+ | GPIO32 |
| AD8232 LO- | GPIO33 |

## Firmware Architecture

The firmware uses FreeRTOS tasks and a shared `sensor_data_t` snapshot protected by `sensor_mutex`. I2C access is serialized with `i2c_mutex`.

Main tasks:

| Task | Rate | Purpose |
| --- | ---: | --- |
| `ecg_task` | 250 Hz | Reads AD8232 ADC, checks leads, feeds Pan-Tompkins |
| `max_task` | 100 Hz | Reads MAX30102 red/IR FIFO and computes PPG vitals |
| `mpu_task` | 50 Hz | Reads MPU6050 accelerometer values |
| `gsr_task` | 10 Hz | Reads GSR ADC |
| `temp_task` | 1/min | Reads LM35 and converts to temperature |
| `oled_task` | 5 Hz | Displays live local status |
| `ml_task` | 1 Hz | Samples fused state for ML feature windows |
| `reporter_task` | 30 s | Builds dashboard JSON and sends it over WiFi |

Startup flow:

1. Configure I2C and ADC.
2. Create mutexes.
3. Initialize MAX30102 and MPU6050.
4. Initialize Pan-Tompkins.
5. Initialize TinyML feature model if enabled.
6. Connect WiFi if SSID is configured.
7. Run calibration for all sensors.
8. Start FreeRTOS tasks.
9. Register GPIO interrupt handlers.

## Signal Processing

### ECG

The ECG path uses the AD8232 module sampled at 250 Hz. The signal is centered using the ECG calibration baseline and processed through a Pan-Tompkins-style QRS detector.

The reporter exposes:

- `bpm`
- `mean_rr_ms`
- `sdnn_ms`
- `rmssd_ms`
- `r_peaks`

If ECG leads disconnect, the Pan-Tompkins state is reset so stale filter state is not reused after reconnect.

### PPG / SpO2-style Calculation

The MAX30102 path stores a 4-second red/IR window at 100 Hz. It computes:

- PPG BPM from IR peak crossings.
- SpO2-style value from red/IR AC/DC ratio.
- `ir_ac` as a signal-quality indicator.
- `ppg_valid` to distinguish valid PPG from stale or bad data.

Invalid PPG conditions clear the displayed/reported PPG values instead of keeping old values.

### Motion

The MPU6050 accelerometer is sampled at 50 Hz. The live JSON reports raw acceleration, while the ML feature layer applies calibration offsets and computes motion magnitude statistics.

### GSR

GSR is sampled at 10 Hz and reported as raw ADC counts. The ML feature layer subtracts the calibrated resting baseline and extracts mean, standard deviation, and slope.

### Temperature

The LM35 is sampled once per minute. Runtime conversion uses the LM35 calibration result for ADC offset and voltage reference.

## TinyML Layer

The project includes a lightweight on-device TinyML-style module in:

- `esp1/src/ml_model.c`
- `esp1/src/ml_model.h`

It does not change the dashboard JSON payload. Instead, it logs local ML output on UART.

The ML path:

1. `ml_task` samples the latest fused sensor state once per second.
2. The model accumulates a 30-second feature window.
3. `reporter_task` finalizes the feature window at the same cadence as JSON reporting.
4. A small fixed-weight inference model estimates:
   - `stress_score`
   - `artifact_score`
   - `anomaly_score`
   - `reliability_score`
   - label: `nominal`, `high_arousal`, `motion_artifact`, or `anomaly`

Example UART output:

```text
[ML] label=nominal stress=42 artifact=12 anomaly=8 reliability=88 ecg_bpm=74.3 ppg_bpm=73.8 ppg_valid=1.00 accel_std=220.4 gsr_slope=12.0
```

Optional CSV feature logging can be enabled from menuconfig. It prints rows prefixed with `ml_csv,...`, which can be captured from the serial monitor and used for later model training.

Current ML features include:

- ECG HRV: BPM, mean RR, SDNN, RMSSD, R-peak count.
- PPG: mean PPG BPM, mean SpO2, valid ratio, IR AC.
- Motion: accelerometer magnitude mean and standard deviation.
- ECG quality: leads-ok ratio.
- GSR: mean, standard deviation, slope.
- Temperature: mean temperature over the window.

## Telemetry JSON

The reporter sends one JSON payload every 30 seconds.

Current schema:

```json
{
  "bpm": 72.4,
  "mean_rr_ms": 828.5,
  "sdnn_ms": 32.1,
  "rmssd_ms": 28.8,
  "r_peaks": 36,
  "ppg_valid": 1,
  "ppg_bpm": 73.0,
  "ppg_spo2": 97.2,
  "ir_ac": 12000.0,
  "spo2_red": 85321,
  "spo2_ir": 93402,
  "accel": [120, -80, 16320],
  "ecg_leads": 1,
  "gsr": 1870,
  "temp": 28.1
}
```

Notes:

- `bpm` is ECG/Pan-Tompkins-derived.
- `ppg_bpm` is MAX30102/PPG-derived.
- `ppg_spo2` is an approximate SpO2-style value, not clinically validated.
- When PPG is invalid, `ppg_valid` is `0` and `ppg_bpm` / `ppg_spo2` are sent as `null`.
- The ML results are not included in this JSON so existing dashboards do not break.

## Development Server and Logging

`esp1/server.py` is a simple HTTP receiver for local development.

It accepts:

- `/data`
- `/api/sensor_data`

It logs incoming telemetry to:

- `esp1/logs/sensor_data.jsonl`
- `esp1/logs/sensor_data.csv`

Run it with:

```bash
cd esp1
python server.py
```

Optional environment variables:

```bash
SENSOR_SERVER_PORT=8080 SENSOR_LOG_DIR=logs python server.py
```

If your firmware endpoint still points to `192.168.4.1:80/data`, make sure your dashboard/server IP, port, and path match `esp1/lib/wifi_tx/wifi_tx.h`.

## Configuration

Project configuration is exposed through ESP-IDF menuconfig:

```bash
cd esp1
idf.py menuconfig
```

Open:

```text
Sensor Hub Configuration
```

Available options:

| Option | Purpose |
| --- | --- |
| `SENSOR_HUB_WIFI_SSID` | WiFi network name |
| `SENSOR_HUB_WIFI_PASSWORD` | WiFi password |
| `SENSOR_HUB_LOG_REPORT_JSON` | Print full dashboard JSON to UART |
| `SENSOR_HUB_ML_ENABLE` | Enable on-device ML feature extraction/inference |
| `SENSOR_HUB_ML_LOG_FEATURES` | Print ML feature CSV rows to UART |

Local ESP-IDF config files are ignored by `.gitignore` because they may contain WiFi credentials.

## Build and Flash

This project is configured for PlatformIO with ESP-IDF:

```bash
cd esp1
pio run
pio run -t upload
pio device monitor
```

If using ESP-IDF directly:

```bash
cd esp1
idf.py set-target esp32
idf.py menuconfig
idf.py build
idf.py flash monitor
```

Serial monitor speed:

```text
115200
```

## Calibration

Calibration runs once during boot before sensor tasks start.

Calibration steps:

1. MAX30102: keep finger off the sensor during ambient calibration.
2. MPU6050: keep the board flat and still.
3. ECG: attach electrodes and remain still.
4. GSR: attach electrodes and relax.
5. LM35: update the reference temperature in code if a real thermometer is available.

The firmware currently uses:

- MAX30102 calibration for PPG gate/baseline handling.
- MPU6050 calibration for ML motion features.
- ECG calibration for Pan-Tompkins input centering.
- GSR calibration for ML features.
- LM35 calibration for runtime temperature conversion.

## Demo Workflow

1. Wire the sensors and OLED.
2. Start your dashboard/backend receiver.
3. Configure WiFi credentials with `idf.py menuconfig`.
4. Confirm firmware endpoint matches your dashboard/server.
5. Flash the ESP32.
6. Watch serial logs for:
   - calibration messages
   - sensor task startup
   - reporter POST status
   - `[ML]` inference lines
7. Verify OLED updates locally.
8. Verify dashboard receives JSON every 30 seconds.
9. Enable `SENSOR_HUB_ML_LOG_FEATURES` if collecting model-training data.

## Data Collection for Better ML

The included model is a lightweight fixed-weight feature model. To make it stronger:

1. Enable CSV logging from `server.py` and/or UART `ml_csv` logging.
2. Collect sessions with labels, for example:
   - rest
   - walking
   - hand motion artifact
   - stressed/aroused
   - recovery
   - bad PPG contact
   - ECG leads off
3. Train a small model in Python using the same feature set.
4. Replace the fixed weights in `ml_model.c`.
5. Re-test on the ESP32.

Good first models:

- Logistic regression for explainability.
- Small dense neural network for TinyML-style inference.
- Decision tree exported to C.
- Isolation forest server-side for anomaly detection.

## Known Limitations

- The SpO2-style calculation is approximate and not clinically validated.
- WiFi transport is plain HTTP.
- The HTTP endpoint is currently configured in `wifi_tx.h`, not menuconfig.
- The Python server is for development, not production.
- The TinyML model is a starter embedded feature model, not trained on a validated dataset.
- Sensor quality depends heavily on wiring, electrode placement, finger contact, and motion.

## Troubleshooting

### No WiFi POSTs

- Check `SENSOR_HUB_WIFI_SSID` and `SENSOR_HUB_WIFI_PASSWORD`.
- Confirm the server IP/port/path in `wifi_tx.h`.
- Confirm dashboard/server is on the same network.
- Check serial logs for `[REPORTER] POST failed`.

### PPG shows invalid

- Ensure finger contact on the MAX30102.
- Check MAX30102 wiring and I2C address.
- Avoid motion during PPG capture.
- Reboot and let ambient calibration run with no finger on the sensor.

### ECG BPM is zero

- Check AD8232 electrode placement.
- Confirm LO+ and LO- polarity.
- Watch OLED/JSON `ecg_leads`.
- Remain still during ECG calibration.

### Temperature looks wrong

- LM35 output is low-voltage, and ESP32 ADCs are not very linear in that range.
- Update the reference temperature passed to `lm35_calibrate()`.
- Verify sensor power and ground.

### ML output looks noisy

- Enable CSV logging and inspect features.
- Check `artifact_score`; high motion intentionally lowers confidence.
- Collect labeled data and replace starter weights with trained weights.

## Future Improvements

- Move HTTP host/port/path into menuconfig.
- Add TLS/authentication or signed telemetry.
- Add offline queueing when WiFi is unavailable.
- Add a trained TensorFlow Lite Micro model.
- Add OTA updates.
- Store calibration values in NVS.
- Add per-sensor health/error counters to telemetry.
- Add a proper web dashboard with trend plots, ML confidence, and event markers.
