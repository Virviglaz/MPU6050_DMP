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

static I2C_Interface i2c_interface;
static I2C_DeviceBase i2c_device(i2c_interface, 0x68);
static MPU6050_DMP612 mpu6050(i2c_device);

int main(int argc, char *argv[])
{
    i2c_interface.Init(argc > 1 ? argv[1] : "/dev/i2c-0");

    int res = 0;

    try {
        res = mpu6050.Init();
    } catch (const std::exception &e) {
        fprintf(stderr, "Failed to initialize MPU6050: %s\n", e.what());
        return 1;
    }

    for (int i = 0; i < 10; ++i) {
        auto real_data = mpu6050.GetRealIMUData();
        printf("Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°\n", real_data.roll, real_data.pitch, real_data.yaw);
        printf("Gyro: gx=%.2f °/s, gy=%.2f °/s, gz=%.2f °/s\n", real_data.gx, real_data.gy, real_data.gz);
        printf("Linear Acceleration: ax=%.2f m/s², ay=%.2f m/s², az=%.2f m/s²\n\n", real_data.ax_linear, real_data.ay_linear, real_data.az_linear);
    }

    return res;
}
```

Note: This project depends on my personal library collection providing interface classes for I2c bus and I2c device.
This library doesn't not use any platform specific code and compatible with little and big-endian hardware.