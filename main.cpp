#include "mpu6050.h"
#include "i2c.h"
#include <exception>
#include <cstdio>
#include <thread>
#include <chrono>

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
            printf("MPU6050 DMP %s calibration %s\n", mpu6050.GetDMPVersion(), res ? "failed" : "succeeded");
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

