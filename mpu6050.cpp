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

#include "mpu6050.h"
#include "bitops.h"
#include <errno.h>
#include <cmath>
#include <array>

#define MPU6050_RA_XG_OFFS_TC 0x00
#define MPU6050_RA_YG_OFFS_TC 0x01
#define MPU6050_RA_ZG_OFFS_TC 0x02
#define MPU6050_RA_X_FINE_GAIN 0x03
#define MPU6050_RA_Y_FINE_GAIN 0x04
#define MPU6050_RA_Z_FINE_GAIN 0x05

/* Accelerometer offset registers */
#define MPU6050_RA_XA_OFFS_H 0x06
#define MPU6050_RA_XA_OFFS_L_TC 0x07
#define MPU6050_RA_YA_OFFS_H 0x08
#define MPU6050_RA_YA_OFFS_L_TC 0x09
#define MPU6050_RA_ZA_OFFS_H 0x0A
#define MPU6050_RA_ZA_OFFS_L_TC 0x0B

/* Gyroscope offset registers */
#define MPU6050_RA_XG_OFFS_USRH 0x13
#define MPU6050_RA_XG_OFFS_USRL 0x14
#define MPU6050_RA_YG_OFFS_USRH 0x15
#define MPU6050_RA_YG_OFFS_USRL 0x16
#define MPU6050_RA_ZG_OFFS_USRH 0x17
#define MPU6050_RA_ZG_OFFS_USRL 0x18

#define MPU6050_RA_SMPLRT_DIV 0x19
#define MPU6050_RA_CONFIG 0x1A
#define MPU6050_RA_GYRO_CONFIG 0x1B
#define MPU6050_RA_ACCEL_CONFIG 0x1C
#define MPU6050_RA_FF_THR 0x1D
#define MPU6050_RA_FF_DUR 0x1E
#define MPU6050_RA_MOT_THR 0x1F
#define MPU6050_RA_MOT_DUR 0x20
#define MPU6050_RA_ZRMOT_THR 0x21
#define MPU6050_RA_ZRMOT_DUR 0x22
#define MPU6050_RA_FIFO_EN 0x23
#define MPU6050_RA_I2C_MST_CTRL 0x24
#define MPU6050_RA_I2C_SLV0_ADDR 0x25
#define MPU6050_RA_I2C_SLV0_REG 0x26
#define MPU6050_RA_I2C_SLV0_CTRL 0x27
#define MPU6050_RA_I2C_SLV1_ADDR 0x28
#define MPU6050_RA_I2C_SLV1_REG 0x29
#define MPU6050_RA_I2C_SLV1_CTRL 0x2A
#define MPU6050_RA_I2C_SLV2_ADDR 0x2B
#define MPU6050_RA_I2C_SLV2_REG 0x2C
#define MPU6050_RA_I2C_SLV2_CTRL 0x2D
#define MPU6050_RA_I2C_SLV3_ADDR 0x2E
#define MPU6050_RA_I2C_SLV3_REG 0x2F
#define MPU6050_RA_I2C_SLV3_CTRL 0x30
#define MPU6050_RA_I2C_SLV4_ADDR 0x31
#define MPU6050_RA_I2C_SLV4_REG 0x32
#define MPU6050_RA_I2C_SLV4_DO 0x33
#define MPU6050_RA_I2C_SLV4_CTRL 0x34
#define MPU6050_RA_I2C_SLV4_DI 0x35
#define MPU6050_RA_I2C_MST_STATUS 0x36
#define MPU6050_RA_INT_PIN_CFG 0x37
#define MPU6050_RA_INT_ENABLE 0x38
#define MPU6050_RA_DMP_INT_STATUS 0x39
#define MPU6050_RA_INT_STATUS 0x3A
#define MPU6050_RA_ACCEL_XOUT_H 0x3B
#define MPU6050_RA_ACCEL_XOUT_L 0x3C
#define MPU6050_RA_ACCEL_YOUT_H 0x3D
#define MPU6050_RA_ACCEL_YOUT_L 0x3E
#define MPU6050_RA_ACCEL_ZOUT_H 0x3F
#define MPU6050_RA_ACCEL_ZOUT_L 0x40
#define MPU6050_RA_TEMP_OUT_H 0x41
#define MPU6050_RA_TEMP_OUT_L 0x42
#define MPU6050_RA_GYRO_XOUT_H 0x43
#define MPU6050_RA_GYRO_XOUT_L 0x44
#define MPU6050_RA_GYRO_YOUT_H 0x45
#define MPU6050_RA_GYRO_YOUT_L 0x46
#define MPU6050_RA_GYRO_ZOUT_H 0x47
#define MPU6050_RA_GYRO_ZOUT_L 0x48
#define MPU6050_RA_EXT_SENS_DATA_00 0x49
#define MPU6050_RA_EXT_SENS_DATA_01 0x4A
#define MPU6050_RA_EXT_SENS_DATA_02 0x4B
#define MPU6050_RA_EXT_SENS_DATA_03 0x4C
#define MPU6050_RA_EXT_SENS_DATA_04 0x4D
#define MPU6050_RA_EXT_SENS_DATA_05 0x4E
#define MPU6050_RA_EXT_SENS_DATA_06 0x4F
#define MPU6050_RA_EXT_SENS_DATA_07 0x50
#define MPU6050_RA_EXT_SENS_DATA_08 0x51
#define MPU6050_RA_EXT_SENS_DATA_09 0x52
#define MPU6050_RA_EXT_SENS_DATA_10 0x53
#define MPU6050_RA_EXT_SENS_DATA_11 0x54
#define MPU6050_RA_EXT_SENS_DATA_12 0x55
#define MPU6050_RA_EXT_SENS_DATA_13 0x56
#define MPU6050_RA_EXT_SENS_DATA_14 0x57
#define MPU6050_RA_EXT_SENS_DATA_15 0x58
#define MPU6050_RA_EXT_SENS_DATA_16 0x59
#define MPU6050_RA_EXT_SENS_DATA_17 0x5A
#define MPU6050_RA_EXT_SENS_DATA_18 0x5B
#define MPU6050_RA_EXT_SENS_DATA_19 0x5C
#define MPU6050_RA_EXT_SENS_DATA_20 0x5D
#define MPU6050_RA_EXT_SENS_DATA_21 0x5E
#define MPU6050_RA_EXT_SENS_DATA_22 0x5F
#define MPU6050_RA_EXT_SENS_DATA_23 0x60
#define MPU6050_RA_MOT_DETECT_STATUS 0x61
#define MPU6050_RA_I2C_SLV0_DO 0x63
#define MPU6050_RA_I2C_SLV1_DO 0x64
#define MPU6050_RA_I2C_SLV2_DO 0x65
#define MPU6050_RA_I2C_SLV3_DO 0x66
#define MPU6050_RA_I2C_MST_DELAY_CTRL 0x67
#define MPU6050_RA_SIGNAL_PATH_RESET 0x68
#define MPU6050_RA_MOT_DETECT_CTRL 0x69
#define MPU6050_RA_USER_CTRL 0x6A
#define MPU6050_RA_PWR_MGMT_1 0x6B
#define MPU6050_RA_PWR_MGMT_2 0x6C
#define MPU6050_RA_BANK_SEL 0x6D
#define MPU6050_RA_MEM_START_ADDR 0x6E
#define MPU6050_RA_MEM_R_W 0x6F
#define MPU6050_RA_DMP_CFG_1 0x70
#define MPU6050_RA_DMP_CFG_2 0x71
#define MPU6050_RA_FIFO_COUNTH 0x72
#define MPU6050_RA_FIFO_COUNTL 0x73
#define MPU6050_RA_FIFO_R_W 0x74
#define MPU6050_RA_WHO_AM_I 0x75

/* MPU6050_RA_INT_PIN_CFG */
#define INT_LEVEL 0x80
#define INT_OPEN 0x40
#define LATCH_INT_EN 0x20
#define INT_RD_CLEAR 0x10
#define FSYNC_INT_LEVEL 0x08
#define FSYNC_INT_EN 0x04
#define I2C_BYPASS_EN 0x02

/* MPU6050_RA_INT_ENABLE */
#define MOT_EN 0x40
#define FIFO_OFLOW_EN 0x10
#define I2C_MST_INT_EN 0x08
#define DATA_RDY_EN 0x01

/* MPU6050_RA_PWR_MGMT_1 */
#define DEVICE_RESET 0x80
#define SLEEP_BIT 0x40
#define CYCLE 0x20
#define TEMP_DIS 0x08
#define DATA_READY_BIT 0x01

#define IFS_BUS_TIMEOUT_CYCLES 50000

int MPU6050_Base::Init()
{
    uint8_t who_am_i = 0;
    ifs_.Read(MPU6050_RA_WHO_AM_I, who_am_i);
    who_am_i &= 0x7E;
    who_am_i >>= 1;
    if (who_am_i != 0x34)
        return -ENODEV;

    /* Minimal configuration */
    ifs_.Write(MPU6050_RA_PWR_MGMT_1, 1);
    ifs_.Write(MPU6050_RA_PWR_MGMT_2, 0);
    ifs_.Write(MPU6050_RA_SMPLRT_DIV, 0x07);
    ifs_.Write(MPU6050_RA_INT_PIN_CFG, 0x10);
    ifs_.Write(MPU6050_RA_INT_ENABLE, DATA_READY_BIT);
    return 0;
}

int MPU6050_Base::Reset()
{
    SetBit(MPU6050_RA_PWR_MGMT_1, DEVICE_RESET);

    uint32_t reset_timeout = IFS_BUS_TIMEOUT_CYCLES;
    while (true) {
        if (--reset_timeout == 0) {
            return -ETIMEDOUT; // Error: Chip reset timed out (hardware failure)
        }

        uint8_t status;
        ifs_.Read(MPU6050_RA_PWR_MGMT_1, status);
        if ((status & DEVICE_RESET) == 0) {
			break; // Reset complete
		}
    }

    return 0;
}

bool MPU6050_Base::IsDataReady()
{
    uint8_t status;
    ifs_.Read(MPU6050_RA_INT_STATUS, status);
    return status & DATA_READY_BIT;
}

MPU6050_Base::RawData& MPU6050_Base::WaitForData()
{
    while (!IsDataReady()) {};
    return GetData();
}

void MPU6050_Base::SetAccGain(AccelGain gain)
{
    ifs_.Write(MPU6050_RA_ACCEL_CONFIG, static_cast<uint8_t>(gain));
    _acc_gain = gain;
}

void MPU6050_Base::SetGyroGain(GyroGain gain)
{
    ifs_.Write(MPU6050_RA_GYRO_CONFIG, static_cast<uint8_t>(gain));
    _gyro_gain = gain;
}

void MPU6050_Base::SetSampleRate(uint8_t sample_rate_hz)
{
    ifs_.Write(MPU6050_RA_SMPLRT_DIV, 1000 / sample_rate_hz - 1);
}

void MPU6050_Base::SetFilterOrder(uint8_t filter_order)
{
    ifs_.Write(MPU6050_RA_CONFIG, filter_order & 0x07);
}

void MPU6050_Base::EnableDataReadyInterrupt()
{
    /* LATCH_INT_EN + INT_RD_CLEAR */
    ifs_.Write(MPU6050_RA_INT_PIN_CFG, 0x30);
}

MPU6050_Base::RawData& MPU6050_Base::GetData()
{
#pragma pack(push, 1)
	union {
		uint8_t be_buffer[14]; // Buffer to hold big-endian raw data (14 bytes: AccX, AccY, AccZ, Temp, GyroX, GyroY, GyroZ)
		struct {
			int16_t x; /* MPU6050_RA_ACCEL_XOUT H/L */
			int16_t y; /* MPU6050_RA_ACCEL_YOUT H/L */
			int16_t z; /* MPU6050_RA_ACCEL_ZOUT H/L */
			int16_t temp; /* MPU6050_RA_TEMP_OUT H/L */
			int16_t ax; /* MPU6050_RA_GYRO_XOUT H/L */
			int16_t ay; /* MPU6050_RA_GYRO_YOUT H/L */
			int16_t az; /* MPU6050_RA_GYRO_ZOUT H/L */
		} data_struct; // Struct to access the raw data as individual fields
    } data_union;
#pragma pack(pop)

    ifs_.Read(MPU6050_RA_ACCEL_XOUT_H, data_union.be_buffer, sizeof(data_union.be_buffer));

    cached_data = RawData(
        		IMU::RawData16_XYZ( // acceleration data
        				BigEndianToNative(data_union.data_struct.x),
        				BigEndianToNative(data_union.data_struct.y),
    					BigEndianToNative(data_union.data_struct.z),
    					static_cast<float>(16384.0f / (1 << static_cast<uint8_t>(_acc_gain)))),
    			IMU::RawData16_XYZ( // gyroscope data
    					BigEndianToNative(data_union.data_struct.ax),
    					BigEndianToNative(data_union.data_struct.ay),
    					BigEndianToNative(data_union.data_struct.az),
    					static_cast<float>(131.0f / (1 << static_cast<uint8_t>(_gyro_gain)))),
    			static_cast<float>(BigEndianToNative(data_union.data_struct.temp))
    	);

    return cached_data;
}

void MPU6050_Base::SetBit(uint8_t reg, uint8_t bit_mask)
{
    uint8_t value;
    ifs_.Read(reg, value);
    value |= bit_mask;
    ifs_.Write(reg, value);
}

void MPU6050_Base::ClearBit(uint8_t reg, uint8_t bit_mask)
{
    uint8_t value;
    ifs_.Read(reg, value);
    value &= ~bit_mask;
    ifs_.Write(reg, value);
}

int16_t MPU6050_Base::ReadReg_s16(uint8_t reg)
{
    int16_t value;
    ifs_.Read(reg, reinterpret_cast<uint8_t*>(&value), 2);
    return BigEndianToNative(value);
}

void MPU6050_Base::WriteReg_s16(uint8_t reg, int16_t value)
{
    int16_t be_value = NativeToBigEndian(value);
    ifs_.Write(reg, reinterpret_cast<uint8_t*>(&be_value), 2);
}

void MPU6050_Base::ReadFIFO(uint8_t *buf, size_t size)
{
    ifs_.Read(MPU6050_RA_FIFO_R_W, buf, size);
}

uint16_t MPU6050_Base::GetFIFOCount()
{
    uint16_t fifo_count;
    ifs_.Read(MPU6050_RA_FIFO_COUNTH, (uint8_t*)&fifo_count, sizeof(fifo_count));
    return BigEndianToNative(fifo_count);
}

MPU6050_Base::CalibrationData MPU6050_Base::ReadCalibrationOffsets()
{
    CalibrationData offsets;
    uint8_t buf[6];

    // 1. Safe packet read from I2C into a local byte array (No Strict Aliasing violation)
    ifs_.Read(MPU6050_RA_XA_OFFS_H, buf, 6);
    
    // 2. Reassemble sign-correct values from Big-Endian array
    // We do sign extension by casting the high byte to int8_t before shifting
    offsets.ax = ((static_cast<int16_t>(static_cast<int8_t>(buf[0])) << 8) | buf[1]) >> 1;
    offsets.ay = ((static_cast<int16_t>(static_cast<int8_t>(buf[2])) << 8) | buf[3]) >> 1;
    offsets.az = ((static_cast<int16_t>(static_cast<int8_t>(buf[4])) << 8) | buf[5]) >> 1;

    // 3. Safe packet read for Gyroscope
    ifs_.Read(MPU6050_RA_XG_OFFS_USRH, buf, 6);
    offsets.gx = (static_cast<int16_t>(static_cast<int8_t>(buf[0])) << 8) | buf[1];
    offsets.gy = (static_cast<int16_t>(static_cast<int8_t>(buf[2])) << 8) | buf[3];
    offsets.gz = (static_cast<int16_t>(static_cast<int8_t>(buf[4])) << 8) | buf[5];

    return offsets;
}

void MPU6050_Base::WriteCalibrationOffsets(CalibrationData offsets)
{
    uint8_t buf[6];

    // 1. Pack Accelerometer offsets safely (Using const input, no user data corruption)
    // Shift left by 1 and force the temperature compensation bit to 1
    int16_t x = (offsets.ax << 1) | 1;
    int16_t y = (offsets.ay << 1) | 1;
    int16_t z = (offsets.az << 1) | 1;

    buf[0] = static_cast<uint8_t>((x >> 8) & 0xFF);  buf[1] = static_cast<uint8_t>(x & 0xFF);
    buf[2] = static_cast<uint8_t>((y >> 8) & 0xFF);  buf[3] = static_cast<uint8_t>(y & 0xFF);
    buf[4] = static_cast<uint8_t>((z >> 8) & 0xFF);  buf[5] = static_cast<uint8_t>(z & 0xFF);
    ifs_.Write(MPU6050_RA_XA_OFFS_H, buf, 6);

    // 2. Pack Gyroscope offsets safely
    buf[0] = static_cast<uint8_t>((offsets.gx >> 8) & 0xFF); buf[1] = static_cast<uint8_t>(offsets.gx & 0xFF);
    buf[2] = static_cast<uint8_t>((offsets.gy >> 8) & 0xFF); buf[3] = static_cast<uint8_t>(offsets.gy & 0xFF);
    buf[4] = static_cast<uint8_t>((offsets.gz >> 8) & 0xFF); buf[5] = static_cast<uint8_t>(offsets.gz & 0xFF);
    ifs_.Write(MPU6050_RA_XG_OFFS_USRH, buf, 6);

    // 3. Reset hardware FIFO and DMP pipeline to instantly apply new parameters
    SetBit(MPU6050_RA_USER_CTRL, 0x0C);
}

#ifdef DEBUG
#include <stdio.h>
#endif
int MPU6050_Base::Calibrate(int max_iterations, int16_t target_error)
{
    WriteCalibrationOffsets(CalibrationData()); // Clear existing offsets to start calibration from a known state
    int res = -EFAULT;

    CalibrationData offsets;

	/* Dynamically calculate 1g baseline LSB based on the active accelerometer scale */
	int16_t gravity_1g = static_cast<int16_t>(16384 >> static_cast<uint8_t>(_acc_gain));

    for (int i = 0; i != max_iterations; i++) {
        auto data = WaitForData();

        /* Lambda function to divide by 64 with rounding */
        auto divide_by_64_fast = [](int16_t x) -> int16_t {
            return (x + ((x >> 15) & 63)) >> 6;
        };

        offsets.ax -= divide_by_64_fast(data.Accel.GetRawX());
        offsets.ay -= divide_by_64_fast(data.Accel.GetRawY());
        offsets.az -= divide_by_64_fast(data.Accel.GetRawZ() - gravity_1g);
        offsets.gx -= divide_by_64_fast(data.Gyro.GetRawX());
        offsets.gy -= divide_by_64_fast(data.Gyro.GetRawY());
        offsets.gz -= divide_by_64_fast(data.Gyro.GetRawZ());

        WriteCalibrationOffsets(offsets); // Apply new offsets to the sensor

        /* Calculate the maximum error between the current readings and the target values */
        auto get_error = [&data, gravity_1g]() {
        	std::array<int16_t, 6> results = { data.Accel.GetRawX(), data.Accel.GetRawY(), data.Accel.GetRawZ(), data.Gyro.GetRawX(), data.Gyro.GetRawY(), data.Gyro.GetRawZ() };
            int16_t max_error = 0;
            for (size_t k = 0; k < 6; k++) {
                int16_t error = std::abs(k == 2 ? results[k] - gravity_1g : results[k]);
                if (error > max_error)
                    max_error = error;
            }
            return max_error;
        };


        /* Measure the maximum error and break if within target */
        int16_t error = get_error();
#ifdef DEBUG
        printf("Iter %3d: x=%6d y=%6d z=%6d gx=%6d gy=%6d gz=%6d Err=%d\n",
               i + 1,
			   data.Accel.GetRawX(),
			   data.Accel.GetRawY(),
			   data.Accel.GetRawZ() - gravity_1g,
			   data.Gyro.GetRawX(),
			   data.Gyro.GetRawY(),
			   data.Gyro.GetRawZ(),
			   error);
#endif
        if (error < target_error) {
            res = 0;
            break; // Calibration successful if all readings are within target_error LSB of target
        }
    }

    SetBit(MPU6050_RA_USER_CTRL, 0x0C); // Bits 2 and 3 reset FIFO and DMP

    return res;
}

void MPU6050_Base::ResetFIFO()
{
    uint8_t user_ctrl = 0;

    ifs_.Read(MPU6050_RA_USER_CTRL, user_ctrl);
    user_ctrl |= 0x04;
    ifs_.Write(MPU6050_RA_USER_CTRL, user_ctrl);

    while (1) {
        ifs_.Read(MPU6050_RA_USER_CTRL, user_ctrl);

        if ((user_ctrl & 0x04) == 0)
        {
            break;
        }

    };
}

int MPU6050_DMP_Base::UploadDMPFirmware(const uint8_t *firmware, size_t size)
{
    /* Load DMP firmware */
    const size_t DMP_BANK_SIZE = 256;
    const size_t DMP_CHUNK_SIZE = 16;

    uint8_t bank = 0;
    uint8_t address = 0;
    size_t bytes_written = 0;

    while (bytes_written < size) {
        /* 1. Select the appropriate bank and address based on how many bytes we've already written to the DMP memory. */
        bank = static_cast<uint8_t>(bytes_written / DMP_BANK_SIZE);
        address = static_cast<uint8_t>(bytes_written % DMP_BANK_SIZE);

        /* 2. Choose the bank by writing to the BANK_SEL register (0x6D) */
        ifs_.Write(MPU6050_RA_BANK_SEL, bank);

        /* 3. Set the start address within the bank (Register 0x6E) */
        ifs_.Write(MPU6050_RA_MEM_START_ADDR, address);

        /*
         * 4. Calculate how many bytes we will send in the current transaction
         * (No more than the chunk limit and not exceeding the 256-byte bank boundary)
         */
        size_t current_chunk = DMP_CHUNK_SIZE;
        if (address + current_chunk > DMP_BANK_SIZE) {
            current_chunk = DMP_BANK_SIZE - address;
        }
        if (bytes_written + current_chunk > size) {
            current_chunk = size - bytes_written;
        }

        /*
         * 5. Write the block of data to the memory port 0x6F.
         * Use your corrected Write method that combines
         * register address and data into a single transaction.
         * The register address for the memory port is 0x6F.
         */
        ifs_.Write(MPU6050_RA_MEM_R_W, &firmware[bytes_written], current_chunk);

        bytes_written += current_chunk;
    }

    /* === CONFIGURATION FOR EXACTLY 28-BYTE PACKET (Quat + Accel + Gyro) === */
	/*
	 * Write 4 zero bytes to the DMP internal FIFO configuration register
	 * located at Bank 0, Address 0x4A. This forces DMP 6.12 to strip off
	 * all metadata tails (packet counters, Android orientation, pedometer, timestamps).
	 */
	const uint8_t disable_meta[4] = { 0x00, 0x00, 0x00, 0x00 };
	ifs_.Write(MPU6050_RA_BANK_SEL, static_cast<uint8_t>(0x00));
	ifs_.Write(MPU6050_RA_MEM_START_ADDR, static_cast<uint8_t>(0x4A));
	ifs_.Write(MPU6050_RA_MEM_R_W, disable_meta, sizeof(disable_meta));

    const uint8_t dmp_addr[2] = { 0x04, 0x00 }; // DMP program start address in the MPU6050 memory
    ifs_.Write(MPU6050_RA_DMP_CFG_1, dmp_addr, sizeof(dmp_addr));

    /* RAW_DMP_INT_EN on */
    ifs_.Write(MPU6050_RA_INT_ENABLE, 0x02);

    /* DMP reset */
    ifs_.Write(MPU6050_RA_USER_CTRL, 0x0C);

    uint8_t user_ctrl = 0;
    do {
        ifs_.Read(MPU6050_RA_USER_CTRL, user_ctrl);
    } while (user_ctrl & 0x04);

    /* Enable Fifo and Reset Fifo */
    ifs_.Write(MPU6050_RA_USER_CTRL, 0xC0);

    ResetFIFO();

    return 0;
}

bool MPU6050_DMP_Base::DMPPacketAvailable()
{
    return GetFIFOCount() >= GetDMPPacketSize(); // DMP packet size is 28 bytes
}

IMU_DMP_Quaternion MPU6050_DMP_Base::ConvertDMPData(DMPPacketRaw &raw_packet)
{
    cached_data = RawData(
        		IMU::RawData16_XYZ( // acceleration data
        				BigEndianToNative(raw_packet.acc_x),
        				BigEndianToNative(raw_packet.acc_y),
    					BigEndianToNative(raw_packet.acc_z),
    					static_cast<float>(16384.0f / (1 << static_cast<uint8_t>(_acc_gain)))),
    			IMU::RawData16_XYZ( // gyroscope data
    					BigEndianToNative(raw_packet.gyro_x),
    					BigEndianToNative(raw_packet.gyro_y),
    					BigEndianToNative(raw_packet.gyro_z),
    					static_cast<float>(131.0f / (1 << static_cast<uint8_t>(_gyro_gain)))),
    			static_cast<float>(cached_data.GetTempRaw())
    	);

    const double qscale = 1.0 / 16384.0;
    double w_raw = static_cast<double>(BigEndianToNative(raw_packet.w)) * qscale;
    double x_raw = static_cast<double>(BigEndianToNative(raw_packet.x)) * qscale;
    double y_raw = static_cast<double>(BigEndianToNative(raw_packet.y)) * qscale;
    double z_raw = static_cast<double>(BigEndianToNative(raw_packet.z)) * qscale;

	return IMU_DMP_Quaternion { y_raw, x_raw, -z_raw, w_raw };
}

bool MPU6050_DMP612::ReadDMPPacket(DMPPacket612 &packet)
{
    uint16_t fifo_count = GetFIFOCount();

    // 1. Check for FIFO buffer overflow (Maximum — 1024 bytes)
    if (fifo_count >= 1024)
    {
        MPU6050_DMP_Base::FifoFullEventHandler();
        ResetFIFO();
        return false;
    }

    // 2. Wait until at least one packet is available
    if (fifo_count < GetDMPPacketSize())
        return false;

    // 3. Read exactly one packet from FIFO (Register 0x74)
    ifs_.Read(MPU6050_RA_FIFO_R_W, reinterpret_cast<uint8_t *>(&packet), GetDMPPacketSize());

    return true;
}

int MPU6050_DMP612::Init()
{
    int res = MPU6050_Base::Init();
    if (res)
        return res;

    const uint8_t dmp_img[] = {
        #include "dmp_image_v612.h"
    };

    return UploadDMPFirmware(dmp_img, sizeof(dmp_img));
}

MPU6050_Base::RawData& MPU6050_DMP612::WaitForData()
{
	WaitForQuaternion();
	return cached_data;
}

IMU_DMP_Quaternion& MPU6050_DMP612::GetQuaternion()
{
	return cached_quaternion;
}

IMU_DMP_Quaternion& MPU6050_DMP612::WaitForQuaternion()
{
    DMPPacket612 raw_packet612;
    while (!ReadDMPPacket(raw_packet612)) {
        // It is recommended to add a timeout here to avoid an infinite loop
    }

    DMPPacketRaw raw_packet;
    raw_packet.w = raw_packet612.w;
    raw_packet.x = raw_packet612.x;
    raw_packet.y = raw_packet612.y;
    raw_packet.z = raw_packet612.z;
    raw_packet.acc_x = raw_packet612.acc_x;
    raw_packet.acc_y = raw_packet612.acc_y;
    raw_packet.acc_z = raw_packet612.acc_z;
    raw_packet.gyro_x = raw_packet612.gyro_x;
    raw_packet.gyro_y = raw_packet612.gyro_y;
    raw_packet.gyro_z = raw_packet612.gyro_z;

    cached_quaternion = ConvertDMPData(raw_packet);
    return cached_quaternion;
}

bool MPU6050_DMP20::ReadDMPPacket(DMPPacket20 &packet)
{
    uint16_t fifo_count = GetFIFOCount();

    // 1. Check for FIFO buffer overflow (Maximum — 1024 bytes)
    if (fifo_count >= 1024)
    {
        MPU6050_DMP_Base::FifoFullEventHandler();
        ResetFIFO();
        return false;
    }

    // 2. Wait until at least one packet is available
    if (fifo_count < GetDMPPacketSize())
        return false;

    // 3. Read exactly one packet from FIFO (Register 0x74)
    ifs_.Read(MPU6050_RA_FIFO_R_W, reinterpret_cast<uint8_t *>(&packet), GetDMPPacketSize());

    return true;
}

int MPU6050_DMP20::Init()
{
    int res = MPU6050_Base::Init();
    if (res)
        return res;

    const uint8_t dmp_img[] = {
        #include "dmp_image_v20.h"
    };

    res = UploadDMPFirmware(dmp_img, sizeof(dmp_img));
    if (res)
        return res;

    /* Set DMP configuration registers */
    ifs_.Write(MPU6050_RA_DMP_CFG_1, 0x03);
    ifs_.Write(MPU6050_RA_DMP_CFG_2, 0x00);
    SetBit(MPU6050_RA_XG_OFFS_TC, 1); // Enable temperature compensation for gyroscope offsets
    return res;
}

MPU6050_Base::RawData& MPU6050_DMP20::WaitForData()
{
	WaitForQuaternion();
	return cached_data;
}

IMU_DMP_Quaternion& MPU6050_DMP20::GetQuaternion()
{
	return cached_quaternion;
}

IMU_DMP_Quaternion& MPU6050_DMP20::WaitForQuaternion()
{
    DMPPacket20 raw_packet20;
    while (!ReadDMPPacket(raw_packet20)) {
        // It is recommended to add a timeout here to avoid an infinite loop
    }

    DMPPacketRaw raw_packet;
    raw_packet.w = raw_packet20.w;
    raw_packet.x = raw_packet20.x;
    raw_packet.y = raw_packet20.y;
    raw_packet.z = raw_packet20.z;
    raw_packet.acc_x = raw_packet20.acc_x;
    raw_packet.acc_y = raw_packet20.acc_y;
    raw_packet.acc_z = raw_packet20.acc_z;
    raw_packet.gyro_x = raw_packet20.gyro_x;
    raw_packet.gyro_y = raw_packet20.gyro_y;
    raw_packet.gyro_z = raw_packet20.gyro_z;

    cached_quaternion = ConvertDMPData(raw_packet);
    return cached_quaternion;
}
