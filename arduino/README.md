# Arduino — XIAO nRF52840 Sense IMU Reader

Sketch that reads the built-in LSM6DS3TR-C IMU on the Seeed Studio XIAO nRF52840 Sense and streams sensor data over Serial in CSV format at 10 Hz.

---

## Hardware Requirements

| Component  | Details                                  |
|------------|------------------------------------------|
| Board      | Seeed Studio XIAO nRF52840 Sense         |
| IMU        | LSM6DS3TR-C (on-board, I2C address 0x6A) |
| Connection | USB-C to PC                              |

> The **Sense** variant of the XIAO nRF52840 is required. The standard (non-Sense) version does not include the IMU.

## Software Requirements

| Requirement        | Details                                              |
|--------------------|------------------------------------------------------|
| Arduino IDE        | 1.8.x or 2.x                                         |
| Board package      | Seeed nRF52 mbed-enabled Boards                      |
| IMU library        | Seeed Arduino LSM6DS3                                |
| Serial baud rate   | 115200                                               |

---

## 1. Install the Board Package

1. Open Arduino IDE → **File → Preferences**
2. Under *Additional boards manager URLs*, add:
   ```
   https://files.seeedstudio.com/arduino/package_seeedstudio_boards_index.json
   ```
3. Go to **Tools → Board → Boards Manager**
4. Search for **Seeed nRF52 mbed-enabled Boards** and click **Install**
5. Select the board:
   **Tools → Board → Seeed nRF52 mbed-enabled Boards → Seeed XIAO BLE Sense - nRF52840**

---

## 2. Install the IMU Library

1. Go to **Tools → Manage Libraries**
2. Search for **LSM6DS3** (author: Seeed Studio)
3. Install **Seeed Arduino LSM6DS3**

The library provides the `LSM6DS3` class that communicates with the sensor over I2C and exposes `readFloatAccelX/Y/Z()`, `readFloatGyroX/Y/Z()`, and `readTempC()`.

---

## 3. Upload the Sketch

1. Connect the board via USB-C
2. Select the correct port under **Tools → Port**
3. Open `sensor.ino` and click **Upload**
4. Open **Tools → Serial Monitor** at **115200 baud**

---

## How the Sketch Works

### Dependencies (`#include`)

```cpp
#include "LSM6DS3.h"   // Seeed Arduino LSM6DS3 — IMU driver
#include "Wire.h"      // Arduino I2C library — required by LSM6DS3
```

`Wire.h` handles low-level I2C communication. `LSM6DS3.h` wraps it with sensor-specific register reads.

---

### Constants

```cpp
const uint16_t SAMPLE_INTERVAL_MS = 100;       // 10 Hz sampling rate
const float    G_TO_MS2           = 9.80665f;  // standard gravity (m/s²)
const float    DEG_TO_RAD         = 0.017453293f; // π / 180
```

- **`SAMPLE_INTERVAL_MS`** — controls how often a new reading is taken. 100 ms = 10 Hz. Change this to adjust sampling frequency (e.g. 10 ms = 100 Hz).
- **`G_TO_MS2`** — the IMU returns accelerometer values in *g* (gravitational units). Multiplying by this constant converts to m/s², which is required for the sensor fusion pipeline downstream.
- **`DEG_TO_RAD`** — the IMU returns gyroscope values in deg/s. Multiplying converts to rad/s, the standard unit for angular velocity in quaternion math.

---

### IMU Initialization (`setup`)

```cpp
LSM6DS3 imu(I2C_MODE, 0x6A);
```

Creates the IMU object using I2C mode at address `0x6A` (the fixed hardware address of the LSM6DS3TR-C on this board).

```cpp
Serial.begin(115200);
while (!Serial);  // wait for USB Serial (remove for standalone use)
```

Opens Serial at 115200 baud. The `while (!Serial)` holds execution until a Serial Monitor is connected — useful during development, but should be removed if the board needs to run without a PC.

```cpp
if (imu.begin() != 0) {
    Serial.println("ERROR: IMU not detected on I2C 0x6A. Check wiring and board selection.");
    while (true);  // halt execution
}
```

`imu.begin()` returns `0` on success. If it fails (returns non-zero), the sketch prints an error and halts. The most common cause is selecting the wrong board variant (standard XIAO instead of Sense).

The header comments printed at startup (`#`) mark metadata lines so they can be filtered out when capturing to CSV.

---

### Non-blocking Timing (`loop`)

```cpp
unsigned long now = millis();
if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = now;
    // ... read and print
}
```

Uses a non-blocking interval check instead of `delay()`. This keeps the loop free for other tasks and avoids cumulative timing drift that `delay()` introduces. `millis()` returns milliseconds since boot and is used directly as the CSV timestamp.

---

### Accelerometer Reading

```cpp
float ax = imu.readFloatAccelX() * G_TO_MS2;
float ay = imu.readFloatAccelY() * G_TO_MS2;
float az = imu.readFloatAccelZ() * G_TO_MS2;
```

Reads the 3-axis linear acceleration in *g* and converts to m/s². At rest on a flat surface, `az` ≈ 9.81 m/s² (gravity). Axes follow the chip's body frame: X = forward, Y = left, Z = up (board-relative).

---

### Gyroscope Reading

```cpp
float gx = imu.readFloatGyroX() * DEG_TO_RAD;
float gy = imu.readFloatGyroY() * DEG_TO_RAD;
float gz = imu.readFloatGyroZ() * DEG_TO_RAD;
```

Reads the 3-axis angular velocity in deg/s and converts to rad/s. At rest, all values should be near zero. Non-zero values when still indicate gyroscope bias — a calibration offset that must be removed before integration.

---

### Temperature Reading

```cpp
float temp = imu.readTempC();
```

Reads the on-chip temperature sensor in °C. This reflects the die temperature of the LSM6DS3, not ambient air temperature. It is useful for bias compensation, as gyroscope and accelerometer outputs vary with temperature.

---

### CSV Output

```cpp
Serial.print(now);       Serial.print(",");
Serial.print(ax, 4);     Serial.print(",");
Serial.print(ay, 4);     Serial.print(",");
Serial.print(az, 4);     Serial.print(",");
Serial.print(gx, 6);     Serial.print(",");
Serial.print(gy, 6);     Serial.print(",");
Serial.print(gz, 6);     Serial.print(",");
Serial.println(temp, 2);
Serial.println("----------------------------------------");
```

Prints one row per sample. The second argument to `Serial.print()` is the number of decimal places:
- Accelerometer: 4 decimal places (precision ≈ 0.1 mg)
- Gyroscope: 6 decimal places (precision ≈ 0.001 mdeg/s — needed to capture small drift)
- Temperature: 2 decimal places

The `---` separator line is printed after each row for readability in the Serial Monitor. It should be filtered out when saving to CSV (see section 5).

---

## 4. Output Format

Lines starting with `#` are header/metadata comments. Data rows:

```
timestamp_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,temp_c
```

| Field          | Unit                    | Decimal places |
|----------------|-------------------------|----------------|
| `timestamp_ms` | ms since boot           | integer        |
| `accel_x/y/z`  | m/s²                    | 4              |
| `gyro_x/y/z`   | rad/s                   | 6              |
| `temp_c`       | °C                      | 2              |

Example Serial Monitor output:

```
# XIAO nRF52840 Sense — IMU LSM6DS3TR-C
# timestamp_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,temp_c
# IMU OK — sampling at 10 Hz
103,0.0412,-0.0231,9.7954,0.000321,-0.000154,0.000089,27.50
----------------------------------------
203,0.0398,-0.0244,9.8011,0.000298,-0.000167,0.000102,27.51
----------------------------------------
```

---

## 5. Capture Data to CSV

The `#` comment lines and `---` separator lines must be filtered out.

**Linux / macOS:**
```bash
cat /dev/ttyACM0 | grep -v "^#" | grep -v "^-" > data.csv
```

**Windows (Python):**
```python
import serial

s = serial.Serial('COM3', 115200)
with open('data.csv', 'w') as f:
    while True:
        line = s.readline().decode().strip()
        if not line.startswith('#') and not line.startswith('-'):
            f.write(line + '\n')
            print(line)
```

---

## 6. Notes

- **Sampling rate:** 10 Hz (every 100 ms). Change `SAMPLE_INTERVAL_MS` in the sketch to adjust.
- **IMU not detected:** If the Serial Monitor shows `ERROR: IMU not detected`, check that you selected the **Sense** variant of the board.
- **Standalone use:** Remove `while (!Serial)` from `setup()` if running without a PC (the board will wait forever otherwise).
- **Gyroscope bias:** Readings are raw — no factory calibration is applied. A static bias offset is expected and must be subtracted in post-processing.
- **VS Code IntelliSense errors** on `#include` lines are expected — VS Code does not know the Arduino library paths. The sketch compiles correctly from Arduino IDE.
