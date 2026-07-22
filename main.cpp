#include "mpu6050.h"
#include "imu.h"
#include "i2c.h"
#include <exception>
#include <cstdio>
#include <thread>
#include <chrono>

static I2C_Interface i2c_interface;                     // I2C interface for communication with the MPU6050
static I2C_DeviceBase i2c_device(i2c_interface, 0x68);  // I2C device representing the MPU6050 at address 0x68
static MPU6050_DMP612 mpu6050(i2c_device);              // MPU6050 device with DMP 6.12 firmware
static IMU::Position<double> imu_position;              // IMU position tracker to compute position and velocity from acceleration data

int main(int argc, char *argv[])
{
    i2c_interface.Init(argc > 1 ? argv[1] : "/dev/i2c-0");

    int res = 0;

    try {
        mpu6050.Reset(); // Reset the MPU6050 to ensure it's in a known state before initialization
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Wait for 100 ms after reset
        res = mpu6050.Init();
        if (!res) {
            // Calibrate the MPU6050 to ensure accurate readings. This step is crucial for obtaining reliable data from the sensor.
            res = mpu6050.Calibrate();
            printf("MPU6050 DMP %s calibration %s\n", mpu6050.GetDMPVersion(), res ? "failed" : "succeeded");
        }
    } catch (const std::exception &e) {
        fprintf(stderr, "Failed to initialize MPU6050: %s\n", e.what());
        return 1;
    }

    /* 
     * This is an optional step to set the accelerometer and gyroscope gain.
     * The default values are typically sufficient for most applications, but you can adjust them if needed.
     */
    mpu6050.SetAccGain(IMU::DeviceBase::AccelGain::ACC_GAIN_2G);
    mpu6050.SetGyroGain(IMU::DeviceBase::GyroGain::GYRO_GAIN_250DS);

	const double FIXED_DT = 0.01;   // Fixed time step for position updates (in seconds).
	int i = 0;
	while (1) {
		// 1. Get quaternion and raw data from MPU6050
		IMU::Quaternion<double> quaternion = mpu6050.WaitForQuaternion();

        // 2. Get cached raw data from MPU6050 (obtained from the DMP FIFO buffer together with the quaternion).
		MPU6050_Base::RawData& rawdata = mpu6050.GetData();

		// 3. Get acceleration and angular velocity from raw data and quaternion
		IMU::Acceleration<double> acc{rawdata, quaternion};

		// 4. Get Euler angles from quaternion
		IMU::EulerAngles<double> euler{quaternion};

        // 5. Update position using acceleration and fixed time step
		imu_position.Update(acc, FIXED_DT);

        // 6. Print position, speed, and Euler angles every 100 iterations
		if (++i % 100 == 0) {
            IMU::Vector3D<double> position = imu_position.getPosition();
			IMU::Vector3D<double> velocity = imu_position.getVelocity();
			printf("Pos: X=%.3f, Y=%.3f, Z=%.3f | Speed: X=%.3f, Y=%.3f, Z=%.3f | Roll: %.0f Pitch: %.0f Yaw: %.0f\n",
					position.x, position.y, position.z,     // Print current position in meters
					velocity.x, velocity.y, velocity.z,     // Print current velocity in meters per second
					euler.GetRollDeg(), euler.GetPitchDeg(), euler.GetYawDeg()); // Print Euler angles in degrees
		}
	}

    return res;
}

