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
#include <cstdint>

class MPU6050_Base {
public:
    enum gyro_gain
    {
        GYRO_0250DS = 0x00,
        GYRO_0500DS = 0x08,
        GYRO_1000DS = 0x10,
        GYRO_2000DS = 0x18,
    };

    enum acc_gain
    {
        SCALE_2G = 0x00,
        SCALE_4G = 0x08,
        SCALE_8G = 0x10,
        SCALE_16G = 0x18,
    };

    MPU6050_Base() = delete;
    ~MPU6050_Base() = default;

    MPU6050_Base(I2C_DeviceBase& ifs)
        : ifs_(ifs) {}

    int Init();

    void SetGyroGain(gyro_gain gain);
    void SetAccGain(acc_gain gain);
    void SetSampleRate(uint8_t sample_rate_hz);
    void SetFilterOrder(uint8_t filter_order);
    void EnableDataReadyInterrupt();
    void ReadFIFO(uint8_t *buf, size_t size);

    class Data {
        friend class MPU6050_Base;
    public:
        float GetTemperature() const;
    private:
        int16_t x; /* MPU6050_RA_ACCEL_XOUT H/L */
        int16_t y; /* MPU6050_RA_ACCEL_YOUT H/L */
        int16_t z; /* MPU6050_RA_ACCEL_ZOUT H/L */

        int16_t temp; /* MPU6050_RA_TEMP_OUT H/L */

        int16_t ax; /* MPU6050_RA_GYRO_XOUT H/L */
        int16_t ay; /* MPU6050_RA_GYRO_YOUT H/L */
        int16_t az; /* MPU6050_RA_GYRO_ZOUT H/L */
    };

    Data WaitForData();
    Data GetData();

protected:
    /* Replace this with a GPIO read implementation if needed */
    virtual bool IsDataReady();
    virtual void ResetIC();
    I2C_DeviceBase& ifs_;
    void SetBit(uint8_t reg, uint8_t bit_mask);
    void ClearBit(uint8_t reg, uint8_t bit_mask);
};

class MPU6050_DMP : public MPU6050_Base {
public:
    using MPU6050_Base::MPU6050_Base;

    int Init();

    struct RealIMUData
    {
        float roll, pitch, yaw;                // angles in degrees
        float gx, gy, gz;                      // angular velocity in degrees per second
        float ax_linear, ay_linear, az_linear; // linear acceleration (without gravity) in m/s²
    };

    RealIMUData GetRealIMUData();

private:
    uint16_t GetFIFOCount();
    void ResetFIFO();
    size_t GetDMPPacketSize() const { return 28; }
    bool DMPPacketAvailable();

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

        // Raw gyro data
        int16_t gyro_x;
        int16_t gyro_y;
        int16_t gyro_z;

        // Raw acc data
        int16_t acc_x;
        int16_t acc_y;
        int16_t acc_z;
    };
    #pragma pack(pop)

    bool ReadDMPPacket(DMPPacket612 &packet);
};

#endif /* MPU6050_H */
