/*
 * This file is provided under a MIT license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * MIT License
 *
 * Copyright (c) 2026 Pavel Nadein
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * MPU-6050 C++ driver implementation.
 *
 * Contact Information:
 * Pavel Nadein <pavelnadein@gmail.com>
 */

#ifndef MPU6050_H
#define MPU6050_H

#ifndef __cplusplus
#error "This header requires C++11 or higher"
#endif

#include "devices.h"
#include "imu.h"

/**
 * MPU6050_Base class provides basic functionality to interface with the MPU-6050 sensor,
 * including reading raw accelerometer, gyroscope, and temperature data, as well
 * as configuring the sensor's gain settings, sample rate, and filter order. It also includes
 * methods to convert raw sensor data to real-world units and to handle data ready interrupts.
 */
class MPU6050_Base : public IMU::DeviceBase
{
public:
    /**
     * Accelerometer gain settings (full scale range):
     * SCALE_2G  = ±2g   (16384 LSB/g)
     * SCALE_4G  = ±4g   (8192 LSB/g)
     * SCALE_8G  = ±8g   (4096 LSB/g)
     * SCALE_16G = ±16g  (2048 LSB/g)
     */
    enum acc_gain
    {
        SCALE_2G = 0x00,
        SCALE_4G = 0x08,
        SCALE_8G = 0x10,
        SCALE_16G = 0x18,
    };

    /**
     * Gyroscope gain settings (full scale range):
     * GYRO_0250DS = ±250°/s   (131 LSB/°/s)
     * GYRO_0500DS = ±500°/s   (65.5 LSB/°/s)
     * GYRO_1000DS = ±1000°/s  (32.8 LSB/°/s)
     * GYRO_2000DS = ±2000°/s  (16.4 LSB/°/s)
     */
    enum gyro_gain
    {
        GYRO_0250DS = 0x00,
        GYRO_0500DS = 0x08,
        GYRO_1000DS = 0x10,
        GYRO_2000DS = 0x18,
    };

    MPU6050_Base() = delete;
    ~MPU6050_Base() = default;

    /**
     * Constructor that takes an I2C device interface. The caller is responsible for ensuring
     * that the I2C device is properly initialized and configured before calling Init().
     */
    MPU6050_Base(I2C_DeviceBase& ifs)
        : ifs_(ifs) {}

    /**
     * Initializes the MPU6050 device. This must be called before any other operations.
     * Returns 0 on success, or a negative error code on failure.
     * Returns -ENODEV if the device is not found or does not respond correctly.
     */
    int Init();

    /**
     * Resets the MPU6050 device. This will reset all registers to their default values and clear the FIFO buffer.
     * After calling this method, it is recommended to wait at least 100 ms before calling Init() again to reconfigure the device.
     */
    int Reset() override;

    /**
     * Sets the accelerometer gain (full scale range). This affects the sensitivity of the accelerometer readings.
     * The default gain is SCALE_2G (±2g).
     *
     * @param gain The desired accelerometer gain setting.
     */
    void SetAccGain(acc_gain gain);


    /**
     * Sets the gyroscope gain (full scale range). This affects the sensitivity of the gyroscope readings.
     * The default gain is GYRO_0250DS (±250°/s).
     *
     * @param gain The desired gyroscope gain setting.
     */
    void SetGyroGain(gyro_gain gain);

    /**
     * Sets the sample rate of the MPU6050. The sample rate determines how often the sensor data is updated.
     * The default sample rate is 125 Hz (when using the DMP, the sample rate is determined by the DMP firmware).
     *
     * @param sample_rate_hz The desired sample rate in Hertz (Hz). Valid values are typically between 4 Hz and 1000 Hz.
     */
    void SetSampleRate(uint8_t sample_rate_hz);

    /**
     * Sets the digital low-pass filter (DLPF) order. This affects the bandwidth of the sensor data and can help reduce noise.
     * The default filter order is 0 (no filtering).
     *
     * @param filter_order The desired filter order (0-7). Higher values correspond to stronger filtering and lower bandwidth.
     */
    void SetFilterOrder(uint8_t filter_order);

    /**
     * Enables the data ready interrupt. When enabled, the MPU6050 will generate an interrupt signal when new sensor data is available.
     * The caller must ensure that the appropriate interrupt handler is set up to handle the data ready interrupt.
     * Note: Enabling the data ready interrupt may require additional configuration of the MPU6050's interrupt settings and the host system's interrupt handling.
     * Override IsDataReady() if you want to use a different method of checking for new data instead of polling.
     */
    void EnableDataReadyInterrupt();

    /**
	 * @brief Holds raw accelerometer and gyroscope data from the sensor.
	 */
	class RawData: public IMU::RawData_6DOF_Base {
	public:
		float GetTemperature() const override {
			return (temperature / 340.0f) + 36.53f;
		}
		float GetTempRaw() const {
			return temperature;
		}
		using RawData_6DOF_Base::RawData_6DOF_Base;
	};

    /**
     * Waits for new sensor data to be available and returns the raw data. This method will block until new data is ready.
     * It is recommended to use EnableDataReadyInterrupt() and override IsDataReady() for a more efficient implementation
     * that does not rely on busy-waiting.
     */
	virtual RawData& WaitForData() override;

    /**
     * Returns the latest raw sensor data without waiting. The caller should ensure that new data is available before calling
     * this method, either by using EnableDataReadyInterrupt() and checking IsDataReady(), or by implementing their own timing mechanism.
     * This method reads the raw accelerometer, gyroscope, and temperature data from the MPU6050 and returns it as a RawData
     * struct. The raw values are in big-endian format and are converted to native endianness before being returned.
     */
	virtual RawData& GetData() override;

    /**
     * Performs a calibration of the MPU6050 sensor. The calibration process adjusts the hardware offset registers
     * iteratively to minimize the error between the sensor readings and the expected values (0 g for accelerometer axes,
     * 0 °/s for gyro axes). The calibration will run for a maximum of max_iterations or until the error is within
     * target_error LSB of the target values.
     * Returns 0 on success, or a negative error code on failure.
     *
     * @param max_iterations The maximum number of calibration iterations to perform.
     * A higher number may yield better results but will take longer.
     * @param target_error The target error threshold in LSB.
     * Calibration will stop when all sensor readings are within this error
     * threshold of the target values (0 for accel X/Y, 16384 for accel Z, 0 for gyro X/Y/Z).
     *
     * @return 0 if calibration succeeded, or a negative error code if calibration failed after max_iterations.
     */
    int Calibrate(int max_iterations = 100, int16_t target_error = 50);

    /**
     * Struct to hold the calibration offsets for the accelerometer and gyroscope.
     * This struct is used to read and write the hardware offset registers of the
     * MPU6050 during the calibration process.
     * The offsets are stored as 16-bit signed integers, and the struct is packed
     * to ensure it matches the layout of the MPU6050's offset registers.
     */
    #pragma pack(push, 1)
    struct cal_offsets {
        cal_offsets() :
            acc_x_offset(0), acc_y_offset(0), acc_z_offset(0),
            gyro_x_offset(0), gyro_y_offset(0), gyro_z_offset(0) {}
        int16_t acc_x_offset;
        int16_t acc_y_offset;
        int16_t acc_z_offset;
        int16_t gyro_x_offset;
        int16_t gyro_y_offset;
        int16_t gyro_z_offset;
    };
    #pragma pack(pop)
    static_assert(sizeof(cal_offsets) == 12,
        "cal_offsets struct must be exactly 12 bytes to match the MPU6050 offset register layout");

    /**
     * Reads the current calibration offsets from the MPU6050's hardware
     * offset registers and returns them as a cal_offsets struct.
     * The caller can use this method to retrieve the current offsets
     * before starting a calibration process, or to read the offsets after
     * calibration to save them for future use.
     */
    cal_offsets ReadCalibrationOffsets();

    /**
     * Writes the given calibration offsets to the MPU6050's hardware offset registers.
     * The caller can use this method to apply new offsets to the sensor, either as
     * part of a calibration process or to restore previously saved offsets. The offsets
     * should be provided as a cal_offsets struct, which contains the accelerometer and gyroscope
     * offsets as 16-bit signed integers. The method will write the offsets to the appropriate
     * registers on the MPU6050.
     */
    void WriteCalibrationOffsets(const cal_offsets& offsets);
protected:
    /* Replace this with a GPIO read implementation if needed */
    virtual bool IsDataReady();

    I2C_DeviceBase& ifs_;
    void SetBit(uint8_t reg, uint8_t bit_mask);
    void ClearBit(uint8_t reg, uint8_t bit_mask);
    int16_t ReadReg_s16(uint8_t reg);
    void WriteReg_s16(uint8_t reg, int16_t value);
    void ReadFIFO(uint8_t *buf, size_t size);
    uint16_t GetFIFOCount();
    void ResetFIFO();
    enum acc_gain _acc_gain = SCALE_2G;
    enum gyro_gain _gyro_gain = GYRO_0250DS;
    RawData cached_data;
private:
    int CalibratePID(float kP, float kI, uint8_t loops);
};

using IMU_DMP_DeviceBase = IMU::DMP_DeviceBase<double>;
using IMU_DMP_RealData = IMU::RealData<double>;

/**
 * MPU6050_DMP_Base is an abstract base class for MPU6050 drivers that utilize
 * the Digital Motion Processor (DMP) firmware. It provides common functionality
 * for initializing the DMP, reading DMP packets from the FIFO buffer, and
 * converting raw DMP data into real-world IMU data (roll, pitch, yaw, angular
 * velocity, and linear acceleration). Derived classes must implement the specific
 * DMP firmware upload and packet parsing logic.
 */
class MPU6050_DMP_Base : public IMU_DMP_DeviceBase, public MPU6050_Base {
public:
    using MPU6050_Base::MPU6050_Base;

    /**
     * Abstract method to initialize the DMP firmware. This must be called after initializing the base MPU6050 device.
     * Returns 0 on success, or a negative error code on failure.
     */
    virtual int Init() = 0;

    /**
     * Optional event handler that is called when the FIFO buffer is full. Derived classes can override this
     * method to implement custom behavior when the FIFO buffer overflows.
     */
    virtual void FifoFullEventHandler() {}

    /**
     * @brief Every DMP implementation must provide a method to wait for raw data to be available.
     * This method should block until new data is ready, and then return the raw DMP data.
     */
    virtual RawData& WaitForData() override = 0;

    /**
	 * @brief Returns cashed raw IMU data from the DMP FIFO buffer.
	 *
	 * @return cashed raw IMU data from the DMP FIFO buffer.
	 */
    RawData& GetData() override { return cached_data; }

    /**
     * Returns the DMP firmware version as a string.
     *
     * @return A string representing the DMP firmware version, e.g., "2.0 or 6.12".
     */
    virtual const char *GetDMPVersion() const = 0;
protected:
    /**
     * Intermediate method to read a DMP packet from the FIFO buffer.
     * This struct needed to align the data between 2 DMP versions (2.0 and 6.12).
     * The derived class should implement the specific packet parsing logic.
     */
    struct DMPPacketRaw
    {
        // Quaternions
        int16_t w, x, y, z;

        // Raw gyro data
        int16_t gyro_x, gyro_y, gyro_z;

        // Raw acc data
        int16_t acc_x, acc_y, acc_z;
    };

    int UploadDMPFirmware(const uint8_t *firmware, size_t size);
    bool DMPPacketAvailable();
    IMU_DMP_RealData ConvertDMPData(DMPPacketRaw &raw_packet);
    virtual size_t GetDMPPacketSize() const = 0;
    IMU_DMP_RealData cached_dmp_data;
};

/**
 * MPU6050_DMP612 is a concrete implementation of the MPU6050_DMP_Base class
 * that uses the DMP 6.12 firmware. It implements the Init() method to upload
 * the DMP 6.12 firmware and the GetRealIMUData() method to read DMP packets
 * from the FIFO buffer and convert them into real-world IMU data.
 * The DMPPacket612 struct defines the format of the DMP 6.12 data packets,
 * which include quaternions, raw gyro data, and raw accelerometer data.
 * The GetRealIMUData() method processes the raw DMP data to calculate roll,
 * pitch, yaw, angular velocity, and linear acceleration, taking into account
 * the specific scaling factors and data formats used by the DMP 6.12 firmware.
 */
class MPU6050_DMP612 : public MPU6050_DMP_Base {
public:
    using MPU6050_DMP_Base::MPU6050_DMP_Base;

    /**
     * Initializes the MPU6050 with the DMP 6.12 firmware.
     * This method first initializes the base MPU6050 device
     * and then uploads the DMP 6.12 firmware to the device.
     * It returns 0 on success, or a negative error code on failure.
     */
    int Init() override;

    /**
     * @brief Wait for RAW data to be available in the FIFO buffer and return it.
     */
    RawData& WaitForData() override;

    /**
     * @brief Get cashed DMP data.
     *
     * @return Cashed DMP processed IMU data (roll, pitch, yaw, angular velocity, linear acceleration).
     */
    IMU_DMP_RealData& GetRealIMUData() override;

    /**
     * @brief Wait for DMP packet to be available in FIFO buffer and read it.
     *
     * @return DMP processed IMU data (roll, pitch, yaw, angular velocity, linear acceleration).
     */
    IMU_DMP_RealData& WaitForRealIMUData() override;

    /**
     * Returns the DMP firmware version as a string.
     *
     * @return A string representing the DMP firmware version, e.g., "6.12".
     */
    const char *GetDMPVersion() const override { return "6.12"; }
private:
    size_t GetDMPPacketSize() const override { return sizeof(DMPPacket612); }

    #pragma pack(push, 1)
    struct DMPPacket612
    {
        // Quaternions
        int16_t w;
        int16_t dummy_1;
        int16_t x;
        int16_t dummy_2;
        int16_t y;
        int16_t dummy_3;
        int16_t z;
        int16_t dummy_4;

        // Raw acc data
        int16_t acc_x;
        int16_t acc_y;
        int16_t acc_z;

        // Raw gyro data
        int16_t gyro_x;
        int16_t gyro_y;
        int16_t gyro_z;
    };
    #pragma pack(pop)

    bool ReadDMPPacket(DMPPacket612 &packet);

    static_assert(sizeof(DMPPacket612) == 28, "DMPPacket612 must be exactly 28 bytes");
};

/**
 * MPU6050_DMP20 is a concrete implementation of the MPU6050_DMP_Base class
 * that uses the DMP 2.0 firmware. It implements the Init() method to upload
 * the DMP 2.0 firmware and the GetRealIMUData() method to read DMP packets
 * from the FIFO buffer and convert them into real-world IMU data.
 * The DMPPacket20 struct defines the format of the DMP 2.0 data packets,
 * which include quaternions, raw gyro data, and raw accelerometer data, with
 * additional dummy bytes for alignment. The GetRealIMUData() method processes
 * the raw DMP data to calculate roll, pitch, yaw, angular velocity,
 * and linear acceleration, taking into account
 * the specific scaling factors and data formats used by the DMP 2.0 firmware.
 */
class MPU6050_DMP20 : public MPU6050_DMP_Base {
public:
    using MPU6050_DMP_Base::MPU6050_DMP_Base;

    /**
     * Initializes the MPU6050 with the DMP 2.0 firmware.
     * This method first initializes the base MPU6050 device
     * and then uploads the DMP 2.0 firmware to the device.
     * It returns 0 on success, or a negative error code on failure.
     */
    int Init() override;

    /**
     * @brief Wait for RAW data to be available in the FIFO buffer and return it.
     */
    RawData& WaitForData() override;

    /**
     * @brief Get cashed DMP data.
     *
     * @return Cashed DMP processed IMU data (roll, pitch, yaw, angular velocity, linear acceleration).
     */
    IMU_DMP_RealData& GetRealIMUData() override;

    /**
     * @brief Wait for DMP packet to be available in FIFO buffer and read it.
     *
     * @return DMP processed IMU data (roll, pitch, yaw, angular velocity, linear acceleration).
     */
    IMU_DMP_RealData& WaitForRealIMUData() override;

    /**
     * Returns the DMP firmware version as a string.
     *
     * @return A string representing the DMP firmware version, e.g., "2.0".
     */
    const char *GetDMPVersion() const override { return "2.0"; }
private:
    size_t GetDMPPacketSize() const override { return sizeof(DMPPacket20); }

    #pragma pack(push, 1)
    struct DMPPacket20
    {
        // Quaternions
        int16_t w;
        int16_t dummy_1;
        int16_t x;
        int16_t dummy_2;
        int16_t y;
        int16_t dummy_3;
        int16_t z;
        int16_t dummy_4;

        // Raw acc data
        int16_t acc_x;
        int16_t dummy_8;
        int16_t acc_y;
        int16_t dummy_9;
        int16_t acc_z;
        int16_t dummy_10;
        int16_t dummy_11;

        // Raw gyro data
        int16_t gyro_x;
        int16_t dummy_5;
        int16_t gyro_y;
        int16_t dummy_6;
        int16_t gyro_z;
        int16_t dummy_7;
    };
    #pragma pack(pop)

    bool ReadDMPPacket(DMPPacket20 &packet);

    static_assert(sizeof(DMPPacket20) == 42, "DMPPacket20 must be exactly 42 bytes");
};

#endif /* MPU6050_H */
