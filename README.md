# MPU6050_DMP
## MPU6050 library with DMP support

Based on jrowberg/i2cdevlib DMP Firmware (supporting versions 6.12 and 2.0)  (greetings to Jeff Rowberg)
It is including base class MPU6050_Base providing only raw data and temperature reading
and MPU6050_DMP class covers DMP data processing for accurate measurements.


```cpp
#include "mpu6050.h"
#include "i2c.h"
#include <exception>
#include <cstdio>

/* I2C interface and device provided by my custom platform-independent implementation */
static I2C_Interface i2c_interface;
static I2C_DeviceBase i2c_device(i2c_interface, 0x68);

/*
 * Accelerometer instance using the DMP 6.12 firmware. The MPU6050_DMP612 class implements the necessary
 * initialization and data retrieval logic to work with the DMP 6.12 firmware, which provides advanced sensor fusion capabilities and outputs real-world IMU data directly from the FIFO buffer.
 * The main function initializes the I2C interface, sets up the MPU6050 device, and demonstrates reading both raw sensor data and processed IMU data from the DMP, printing the results to the console.
 * Note: The actual DMP firmware binary data and the implementation of the Init() method to upload the firmware are not shown here, but should be included in the corresponding source files for a complete implementation.
 * Other firmware versions (like DMP 2.0) can be implemented similarly by creating additional classes that inherit from MPU6050_DMP_Base and implementing the specific logic for each firmware version.
 */
static MPU6050_DMP612 mpu6050(i2c_device);

int main(int argc, char *argv[])
{
    i2c_interface.Init(argc > 1 ? argv[1] : "/dev/i2c-0");

    int res = 0;

    try {
        mpu6050.Reset(); // Reset the MPU6050 to ensure it's in a known state before initialization
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Wait for 100 ms after reset
        res = mpu6050.Init();
        if (!res) {
            res = mpu6050.Calibrate();
            printf("MPU6050 calibration %s\n", res ? "failed" : "succeeded");
        }
    } catch (const std::exception &e) {
        fprintf(stderr, "Failed to initialize MPU6050: %s\n", e.what());
        return 1;
    }

    /* Read and print raw sensor data */
    for (int i = 0; i < 10; ++i) {
        auto real_data = mpu6050.GetData();
        printf("Accelerometer: ax=%.2f g, ay=%.2f g, az=%.2f g\n", real_data.GetAccX(), real_data.GetAccY(), real_data.GetAccZ());
        printf("Gyroscope: gx=%.2f °/s, gy=%.2f °/s, gz=%.2f °/s\n", real_data.GetGyroX(), real_data.GetGyroY(), real_data.GetGyroZ());
        printf("Temperature: %.2f °C\n\n", real_data.GetTemperature());
    }

    /* Read and print real-world IMU data from DMP */
    for (int i = 0; i < 10; ++i) {
        auto real_data = mpu6050.GetRealIMUData();
        printf("Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°\n", real_data.roll, real_data.pitch, real_data.yaw);
        printf("Gyro: gx=%.2f °/s, gy=%.2f °/s, gz=%.2f °/s\n", real_data.gx, real_data.gy, real_data.gz);
        printf("Linear Acceleration: ax=%.2f m/s², ay=%.2f m/s², az=%.2f m/s²\n\n", real_data.ax_linear, real_data.ay_linear, real_data.az_linear);
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Sleep to simulate a 10 Hz update rate
    }

    return res;
}

```

## Output:
```
Accelerometer calibration succeeded
## Accelerometer raw data
Accelerometer: ax=-0.00 g, ay=-0.00 g, az=0.00 g
Gyroscope: gx=-0.05 °/s, gy=-0.03 °/s, gz=0.04 °/s
Temperature: 22.13 °C

Accelerometer: ax=0.00 g, ay=-0.01 g, az=0.00 g
Gyroscope: gx=-0.08 °/s, gy=0.08 °/s, gz=-0.03 °/s
Temperature: 22.13 °C

Accelerometer: ax=0.00 g, ay=0.00 g, az=0.01 g
Gyroscope: gx=0.18 °/s, gy=-0.05 °/s, gz=-0.04 °/s
Temperature: 22.32 °C

## Accelerometer DMP processed data
Roll: 7.68°, Pitch: -1.17°, Yaw: 0.22°
Gyro: gx=3.90 °/s, gy=-3.41 °/s, gz=-4.15 °/s
Linear Acceleration: ax=-0.20 m/s², ay=-1.31 m/s², az=-9.73 m/s²

Roll: 7.68°, Pitch: -1.18°, Yaw: 0.22°
Gyro: gx=-1.46 °/s, gy=-5.37 °/s, gz=1.95 °/s
Linear Acceleration: ax=-0.20 m/s², ay=-1.32 m/s², az=-9.71 m/s²

Roll: 7.56°, Pitch: -1.20°, Yaw: 0.24°
Gyro: gx=7.56 °/s, gy=-0.49 °/s, gz=-5.37 °/s
Linear Acceleration: ax=-0.21 m/s², ay=-1.29 m/s², az=-9.72 m/s²

Roll: 7.56°, Pitch: -1.20°, Yaw: 0.24°
Gyro: gx=-6.83 °/s, gy=-6.34 °/s, gz=1.95 °/s
Linear Acceleration: ax=-0.21 m/s², ay=-1.29 m/s², az=-9.72 m/s²
```

Note: This project depends on my personal library collection providing interface classes for I2c bus and I2c device.
This library doesn't not use any platform specific code and compatible with little and big-endian hardware.

### Known issues:
DMP 2.0 firmware:
- Yaw is wrong, gyro Z is wrong, az is wrong. No idea why. Will not support this firwmare. Please, use latest 6.12 instead.

### How to use
```
1. Clone this repo
$ https://github.com/Virviglaz/MPU6050_DMP.git

2. Clone 'mylibraries'
$ https://github.com/Virviglaz/mylibraries.git

3. Download submodules
$ git submodule update --init

4. Open Makefile and check your build target architecture
CROSS_COMPILE	:= aarch64-linux-gnu-
This example uses Orange Pi3 ARM64 Linux

5. Install related toolchain
for ARM64 (aarch64-linux-gnu-g++):
$ sudo apt install g++-aarch64-linux-gnu -y

6. Build and run
$ make run
```