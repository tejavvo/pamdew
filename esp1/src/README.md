Here's a focused walkthrough of `src/main.c` so you can quickly understand how the sensor hub is organized, what each part does, and a few practical notes about robustness and timing.

High-level summary
- This is an ESP32 FreeRTOS application that reads five sensors and shows values on an SSD1306 OLED.
- Sensors and rates:
  - MAX30102 (PPG) — 100 Hz, on I2C
  - MPU6050 (accelerometer) — 50 Hz, on I2C
  - AD8232 (ECG analog) — 250 Hz via periodic esp_timer + ADC
  - GSR (analog) — 10 Hz via ADC
  - LM35 (temperature analog) — once per minute via ADC
- A shared `sensor_data_t` struct holds the latest snapshot of all sensors. Access is protected by `sensor_mutex`.
- I2C transfers are serialized with `i2c_mutex`.
- Hardware INT pins for MAX30102 and MPU6050 notify FreeRTOS tasks using `xTaskNotifyGiveFromISR`.
- OLED is updated at 5 Hz by `oled_task`.

Important symbols (from the file)
- I2C: `I2C_PORT`, `SDA_PIN` (21), `SCL_PIN` (22), `I2C_FREQ` (400kHz)
- Device addresses: `MAX30102_ADDR = 0x57`, `MPU6050_ADDR = 0x68`
- GPIO interrupts: `MAX_INT_PIN = GPIO_NUM_4`, `MPU_INT_PIN = GPIO_NUM_5`
- ADC channels: `ECG_CHAN = ADC_CHANNEL_6` (GPIO34), `GSR_CHAN = ADC_CHANNEL_7` (GPIO35), `LM35_CHAN = ADC_CHANNEL_0` (GPIO36)
- AD8232 lead-off pins: `AD8232_LO_POS = GPIO_NUM_32`, `AD8232_LO_NEG = GPIO_NUM_33`

Data structure
- `sensor_data_t` stores:
  - `red, ir` (MAX30102 PPG FIFO samples)
  - `ax, ay, az` (MPU6050 accel)
  - `ecg_raw`, `ecg_leads_ok` (AD8232 + lead detection)
  - `gsr_raw`
  - `temp_c` (derived from LM35 ADC)
- Global instance: `g_sensor` (shared snapshot).

Initialization
- `sensor_i2c_init()`:
  - Configures I2C master, SDA/SCL pins, enables pull-ups, sets clock.
  - Installs the I2C driver.
- `adc_sensors_init()`:
  - Creates an ADC one-shot unit (`adc1_handle`) and configures channels with 12 dB attenuation.
  - Configures AD8232 lead pins as inputs.
- Per-device init:
  - `max30102_init()` writes a few registers to set SpO2 mode, sample rate (100 sps), interrupt to PPG_RDY (bit6).
  - `mpu6050_init()` wakes device, sets DLPF and sample rate divider to get a 50 Hz internal update, enables DATA_RDY interrupt.

I2C helpers
- `i2c_wr()` writes a 2-byte buffer (register, value).
- `i2c_rd()` does a write-then-read to read device registers.
- Both call the ESP-IDF I2C master helpers. Note: the code does not check returned `esp_err_t` values (see improvements).

GPIO interrupts and FreeRTOS notifications
- GPIOs for MAX & MPU are configured as inputs with pull-ups and negative-edge interrupts.
- ISRs:
  - `max_isr()` and `mpu_isr()` call `vTaskNotifyGiveFromISR()` to wake their respective tasks (`max_task` and `mpu_task`).
  - They use `portYIELD_FROM_ISR()` when higher priority task waking requires a context switch.

Tasks and timing
- The program creates these tasks in `app_main()` (with given stacks and priorities):
  - `ecg_task` — priority 5, stack 4096
  - `max_task` — priority 4, stack 4096
  - `mpu_task` — priority 4, stack 4096
  - `gsr_task` — priority 2, stack 2048
  - `temp_task` — priority 1, stack 2048
  - `oled_task` — priority 1, stack 4096

- `max_task` (PPG, 100 Hz)
  - Waits on task notification (from `max_isr`) with a 20 ms timeout (2× sample interval).
  - Grabs `i2c_mutex` and reads 6 bytes from MAX30102 FIFO (register 0x07).
  - Extracts 18-bit red and IR values, stores them in `g_sensor` under `sensor_mutex`.

- `mpu_task` (50 Hz)
  - Waits on notification (from `mpu_isr`) with a 40 ms timeout.
  - Uses `i2c_mutex` and reads 6 bytes from register 0x3B (accel X/Y/Z high/low).
  - Stores accel readings in `g_sensor`.

- `ecg_task` (250 Hz)
  - Creates an `esp_timer` (periodic) with callback `ecg_timer_cb` that calls `xTaskNotifyGive()` to wake the task (not an ISR, it runs in esp_timer task context).
  - The periodic timer uses 4000 µs (4 ms) → 250 Hz.
  - The task then reads ADC channel `ECG_CHAN`, samples lead-off GPIOs (`AD8232_LO_POS/NEG`) and writes `ecg_raw` and `ecg_leads_ok` to `g_sensor`.

- `gsr_task` (10 Hz)
  - Uses `vTaskDelayUntil` every 100 ms to poll ADC for GSR and writes to `g_sensor`.

- `temp_task` (1/min)
  - Reads LM35 ADC, converts ADC raw → mV → °C using a rough scaling (top ≈ 3100 mV at 12 dB atten, 10 mV/°C), stores `temp_c`, prints it, then delays 60 s.

- `oled_task` (5 Hz)
  - Initializes SSD1306, clears screen, displays a title.
  - Every 200 ms it takes `sensor_mutex`, copies `g_sensor` to a local `d`, releases mutex (non-blocking snapshot), and writes formatted lines for the sensors to the OLED.

Concurrency & synchronization
- `i2c_mutex` serializes I2C accesses because multiple tasks read devices on the same bus.
- `sensor_mutex` serializes access to `g_sensor`. Producers lock it briefly when updating, OLED copies a full snapshot under the mutex for consistent display.
- ISRs use task notifications to wake tasks efficiently (lower overhead than semaphores).

Hardware-specific notes / details
- MAX30102: the code writes register 0x0A = 0x27 for LED pulse width and sample rate that maps to 100 sps and 4096 ADC range; it enables the PPG_RDY interrupt.
- MPU6050: accelerometer registers start at 0x3B; the sample rate is configured with `SMPLRT_DIV = 19` giving 50 Hz when the internal rate is 1 kHz (DLPF_CFG=1).
- ADC configuration: channels use 12 dB attenuation and `ADC_BITWIDTH_DEFAULT` (ESP-IDF default). The code assumes ADC raw max = 4095 and top voltage ≈ 3100 mV at the configured attenuation (used in `temp_task`).

Practical observations and possible improvements
- Error handling: I2C and ADC calls do not check return values. If a device disconnects or I2C fails, it will silently overwrite with stale/zero data. Consider checking `esp_err_t` returns and implementing retries/fault counters.
- I2C blocking timeouts: `i2c_master_write_to_device` uses a 10 ms timeout. If bus contention occurs and transfer blocks, higher-rate tasks (ECG at 250 Hz reading ADC only — probably OK) might be affected. Keeping I2C use limited to lower-rate sensors mitigates this; the two I2C sensors here are 100 Hz and 50 Hz.
- Priority design: `ecg_task` is the highest priority (5) and uses a timer that notifies at 250 Hz. That makes sense if ECG timing is most critical. MAX/MPU (4) are next. OLED is lowest — it's non-blocking and only snapshots.
- ISR safety: `max_isr` and `mpu_isr` call `vTaskNotifyGiveFromISR` correctly. The code sets up GPIO interrupt on negative edge with pull-ups enabled — verify that the devices indeed assert INT low.
- ADC calibration: temperature conversion uses an approximation for the ADC reference/top voltage. For accurate temperature, calibrate or read ADC reference properly.
- Lead detection logic: `ecg_leads_ok` is computed as both lead-off pins being low (`!gpio_get_level`), which matches some AD8232 implementations where LO signals are active-high/low; confirm wiring and expected polarity.
- Stack sizes look conservative but probably adequate. If you see stack overflows, increase the task stack allocation in `xTaskCreate`.

Main flow (at runtime)
1. `app_main()` prints "boot".
2. Initializes I2C, ADC, creates semaphores, configures sensors (MAX & MPU).
3. Creates tasks for ECG, MAX, MPU, GSR, temp, and OLED (with provided priorities).
4. Sets up the GPIO ISR handlers for MAX and MPU interrupts.
5. Prints "ready".
6. Sensors begin producing data; tasks update `g_sensor`, OLED refreshes periodically.
