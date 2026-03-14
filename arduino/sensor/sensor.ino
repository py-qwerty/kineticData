/*
 * sensor.ino
 * Seeed Studio XIAO nRF52840 Sense — LSM6DS3TR-C IMU reader
 *
 * Reads accelerometer and gyroscope at 50 Hz, streams over BLE.
 * Gyroscope bias is calibrated on first boot and saved to internal flash
 * via NanoBLEFlashPrefs so subsequent boots skip the calibration step.
 *
 * Output format (CSV over Serial, and BLE payload):
 *   timestamp_ms, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
 *   Units: ms | m/s² | m/s² | m/s² | rad/s | rad/s | rad/s
 */

#include "LSM6DS3.h"
#include "Wire.h"
#include <ArduinoBLE.h>
#include <NanoBLEFlashPrefs.h>

// ─── IMU ─────────────────────────────────────────────────────────────────────

// IMU instance: I2C mode, address 0x6A
LSM6DS3 imu(I2C_MODE, 0x6A);

const float DEG_TO_RAD_FACTOR  = 0.017453293f; // PI / 180
const float G_TO_MS2           = 9.80665f;      // standard gravity
const int   SAMPLE_INTERVAL_MS = 20;            // 50 Hz
const int   CALIBRATION_SAMPLES = 500;          // samples averaged for gyro bias

// Gyroscope bias offsets (deg/s), subtracted before unit conversion
float offsetGx = 0, offsetGy = 0, offsetGz = 0;

unsigned long lastSampleTime = 0;

// ─── FLASH PREFS ─────────────────────────────────────────────────────────────

// NanoBLEFlashPrefs stores arbitrary structs in the last pages of nRF52840
// flash using the Nordic FDS (Flash Data Storage) system.
// The struct must stay the same between boots for reads to be valid.
NanoBLEFlashPrefs prefs;

struct GyroPrefs {
    float gx;
    float gy;
    float gz;
    uint32_t magic; // set to PREFS_MAGIC when data is valid
};

#define PREFS_MAGIC 0xDEADBEEF

// ─── BLE ─────────────────────────────────────────────────────────────────────

// Custom 128-bit UUIDs — servicio y característica deben ser DIFERENTES
// para evitar confusión en algunos stacks BLE
BLEService imuService("19B10000-E8F2-537E-4F6C-D104768A1214");

// Characteristic carries 6 floats (ax, ay, az, gx, gy, gz) = 24 bytes
BLECharacteristic imuCharacteristic(
    "19B10001-E8F2-537E-4F6C-D104768A1214",
    BLERead | BLENotify,
    24
);

// ─── FLASH: save offsets ─────────────────────────────────────────────────────

// Writes the three gyro bias floats to flash via NanoBLEFlashPrefs.
void saveOffsets() {
    GyroPrefs data = {offsetGx, offsetGy, offsetGz, PREFS_MAGIC};
    prefs.writePrefs(&data, sizeof(data));
    Serial.println("# Offsets saved to flash");
}

// ─── FLASH: load offsets ─────────────────────────────────────────────────────

// Returns true if a valid GyroPrefs record was found in flash.
// Returns false on first boot (no record yet) or if magic is wrong.
bool loadOffsets() {
    GyroPrefs data;
    int rc = prefs.readPrefs(&data, sizeof(data));

    // rc == 0 means FDS_SUCCESS; any other value means no record or error
    if (rc != 0) return false;
    if (data.magic != PREFS_MAGIC) return false;
    if (isnan(data.gx) || isnan(data.gy) || isnan(data.gz)) return false;

    offsetGx = data.gx;
    offsetGy = data.gy;
    offsetGz = data.gz;
    return true;
}

// ─── CALIBRATION ─────────────────────────────────────────────────────────────

// Averages CALIBRATION_SAMPLES gyroscope readings with the board held still
// to estimate the static bias on each axis. Saves result to flash.
void calibrateGyro() {
    Serial.println("# Calibrating — keep the board still...");
    double sumX = 0, sumY = 0, sumZ = 0;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        sumX += imu.readFloatGyroX();
        sumY += imu.readFloatGyroY();
        sumZ += imu.readFloatGyroZ();
        if (i % 100 == 0) {
            Serial.print("# Sample "); Serial.print(i);
            Serial.print("/"); Serial.println(CALIBRATION_SAMPLES);
        }
        delay(2);
    }

    offsetGx = sumX / CALIBRATION_SAMPLES;
    offsetGy = sumY / CALIBRATION_SAMPLES;
    offsetGz = sumZ / CALIBRATION_SAMPLES;

    Serial.println("# Calibration complete:");
    Serial.print("# offset_gx="); Serial.println(offsetGx, 6);
    Serial.print("# offset_gy="); Serial.println(offsetGy, 6);
    Serial.print("# offset_gz="); Serial.println(offsetGz, 6);

    saveOffsets();
}

// ─── SETUP ───────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    // Esperar Serial solo si hay USB conectado (máximo 2 segundos)
    // Sin esto, la placa se cuelga si arranca sin USB
    unsigned long serialWait = millis();
    while (!Serial && millis() - serialWait < 2000);
    Serial.println("# SensorKit — XIAO nRF52840 Sense");

    // Initialize IMU
    if (imu.begin() != 0) {
        Serial.println("ERROR: IMU not detected on I2C 0x6A. Check board selection.");
        while (true);
    }
    Serial.println("# IMU OK");

    // Load saved gyro offsets from flash, or run calibration if none exist
    if (loadOffsets()) {
        Serial.println("# Offsets loaded from flash — skipping calibration");
        Serial.print("# offset_gx="); Serial.println(offsetGx, 6);
        Serial.print("# offset_gy="); Serial.println(offsetGy, 6);
        Serial.print("# offset_gz="); Serial.println(offsetGz, 6);
    } else {
        Serial.println("# No saved offsets found — running calibration");
        calibrateGyro();
    }

    // Initialize BLE
    if (!BLE.begin()) {
        Serial.println("ERROR: BLE failed to start");
        while (true);
    }

    BLE.setLocalName("SensorKit");
    BLE.setAdvertisedService(imuService);
    imuService.addCharacteristic(imuCharacteristic);
    BLE.addService(imuService);
    BLE.advertise();

    Serial.println("# BLE OK — advertising as 'SensorKit' at 50 Hz");
    Serial.println("# timestamp_ms,ax,ay,az,gx,gy,gz");
}

// ─── LOOP ────────────────────────────────────────────────────────────────────

void loop() {
    // Wait for a BLE central device to connect
    BLEDevice central = BLE.central();

    if (central) {
        Serial.print("# Connected: ");
        Serial.println(central.address());

        while (central.connected()) {
            unsigned long now = millis();

            if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
                lastSampleTime = now;

                // Accelerometer: convert g → m/s²
                float ax = imu.readFloatAccelX() * G_TO_MS2;
                float ay = imu.readFloatAccelY() * G_TO_MS2;
                float az = imu.readFloatAccelZ() * G_TO_MS2;

                // Gyroscope: subtract bias offset, then convert deg/s → rad/s
                float gx = (imu.readFloatGyroX() - offsetGx) * DEG_TO_RAD_FACTOR;
                float gy = (imu.readFloatGyroY() - offsetGy) * DEG_TO_RAD_FACTOR;
                float gz = (imu.readFloatGyroZ() - offsetGz) * DEG_TO_RAD_FACTOR;

                // Pack 6 floats (24 bytes) into BLE characteristic payload
                float payload[6] = {ax, ay, az, gx, gy, gz};
                imuCharacteristic.writeValue((byte*)payload, sizeof(payload));

                // Serial debug — solo si hay Serial conectado, y sin separador
                // para no bloquear el loop cuando el buffer se llena
                if (Serial) {
                    Serial.print(now);    Serial.print(",");
                    Serial.print(ax, 4);  Serial.print(",");
                    Serial.print(ay, 4);  Serial.print(",");
                    Serial.print(az, 4);  Serial.print(",");
                    Serial.print(gx, 6);  Serial.print(",");
                    Serial.print(gy, 6);  Serial.print(",");
                    Serial.println(gz, 6);
                }
            }
        }

        Serial.println("# Disconnected");
        BLE.advertise(); // resume advertising after disconnect
    }
}
