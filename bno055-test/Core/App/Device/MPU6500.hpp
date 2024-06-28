/*
 * MPU6500.hpp
 *
 *  Created on: Jun 28, 2024
 *      Author: jumpei
 */

#ifndef APP_DEVICE_MPU6500_HPP_
#define APP_DEVICE_MPU6500_HPP_



class MPU6500 {
   public:
    MPU6500(MAL* mcu);
    void init();
    void update();

    int16_t raw_Ax, raw_Ay, raw_Az;
    int16_t raw_Gx, raw_Gy, raw_Gz;

    float Ax, Ay, Az;
    float Gx, Gy, Gz;

    float Yaw, Pitch, Roll;
    float Vy, Vx, Vz;

    float Px, Py, Pz;

    bool isInitialized;
    bool isCalibrationed;

   private:
    MAL* _mcu;
    Madgwick _madgwick;
    MovingAverageFilter<float, 50> _filter_Ax;
    MovingAverageFilter<float, 50> _filter_Ay;

    float _dt;
    int16_t _mode;

    int16_t _Gx_drift_constant;
    int16_t _Gy_drift_constant;
    int16_t _Gz_drift_constant;

    int32_t _calibration_sum_Gx;
    int32_t _calibration_sum_Gy;
    int32_t _calibration_sum_Gz;

    int16_t _Ax_drift_constant;
    int16_t _Ay_drift_constant;

    int32_t _calibration_sum_Ax;
    int32_t _calibration_sum_Ay;

    int32_t _calibration_sum_cnt;

    void _read_gyro_data();
    void _read_accel_data();

    void _write_byte(uint8_t reg, uint8_t val);
    uint8_t _read_byte(uint8_t reg);

    void _offset_Zero();
};


#endif /* APP_DEVICE_MPU6500_HPP_ */
