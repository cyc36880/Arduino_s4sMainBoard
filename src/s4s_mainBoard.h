#pragma once

#include "Arduino.h"
#include "Wire.h"

#if defined(__AVR__) || defined(ARDUINO_ARCH_AVR)
// The AVR platform (such as Arduino Nano) does not support std::string
#else
// Other platforms (such as ESP32, STM32) may be supported
#include <string>
#endif

/**
* - Charging management
* - Ambient light
* - RTC
* - Servo
* - Gyroscope
* - Voice module
* - Encoder motor 0 (built-in motor)
* - Encoder motor 1 (built-in motor)
* - Encoder motor 2
* - Encoder motor 3
*/


class s4s_mainBoard
{
public:
    s4s_mainBoard(uint8_t addr = 0x0F);
    ~s4s_mainBoard();

public:
    void begin(TwoWire *wire) { _wire=wire;}
    void begin(int sda = 42, int scl = 41)
    {
#if defined(__AVR__) || defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_RENESAS)
        Wire.begin();
#else
        Wire.begin(sda, scl);
#endif
        _wire = &Wire;
    }

    /**
    * @description: Get charging status
    * @param is_charging Whether it is charging 0: not charging, 1: charging
    * @param charging_voltage Battery level 0~100
    * @return int 0: success, other: fail
    */
    int charging_get_state(uint8_t * is_charging, uint8_t * charging_voltage); 

    /**
    * @description: Set the ambient light status
    * @param light Brightness 0~255; -1 means do not change
    * @param color Color array [r, g, b] 0~255; NULL means do not change
    * @return int 0: success, other: fail
    */
    int ambient_light_set_state(int light, uint8_t color[3]);
    int ambient_light_set_state(int light, uint8_t r, uint8_t g, uint8_t b);

    /**
    * @description: Set RTC time
    * @param year Year 0~99
    * @param month Month 1~12
    * @param day Day 1~31
    * @return int 0: success, other: fail
    */
    int rtc_set_data(uint8_t year, uint8_t month, uint8_t day);
    int rtc_set_time(uint8_t hour, uint8_t minute, uint8_t second);
    int rtc_get_data(uint8_t * year, uint8_t * month, uint8_t * day, uint8_t *week);
    int rtc_get_time(uint8_t * hour, uint8_t * minute, uint8_t * second);

    /**
    * @description: Set the servo angle
    * @param servo_id Servo ID 0~1
    * @param angle Angle 0~180
    * @return int 0: success, other: fail
    */
    int servo_set_angle(uint8_t id, uint8_t angle);

    /**
    * @description: Enable the gyroscope
    * @param enable 0: disable, 1: enable
    * @return int 0: success, other: fail
    */
    int gyro_enable(uint8_t enable);
    /**
    * @description: Set the gyroscope state
    * @param state State 0: idle, 1: start calibration, 2: reset yaw angle
    * @return int 0: success, other: fail
    */
    int gyro_set_state(uint8_t state);
    int gyro_get_state(uint8_t * state);
    int gyro_get_acc(int16_t * accX, int16_t * accY, int16_t * accZ);
    int gyro_get_gyro(int16_t * gyroX, int16_t * gyroY, int16_t * gyroZ);
    int gyro_get_angle(int16_t * angleX, int16_t * angleY, int16_t * angleZ);

    /**
    * @description: Set encoder motor mode
    * @param {uint8_t} id Encoder motor ID 0 ~ 3
    * @param {uint8_t} mode Mode 0: DC motor, 1: position control, 2: speed control
    * @return int 0: success, other: fail
    */
    int encoder_motor_set_mode(uint8_t id, uint8_t mode);
    /**
    * @description: Set encoder motor power/speed
    * @param {uint8_t} id Encoder motor ID 0 ~ 3
    * @param {int16_t} power Power -1000 ~ 1000
    * @return int 0: success, other: fail
    */
    int encoder_motor_set_power(uint8_t id, int16_t power);
    /**
    * @description: Set the target position for the encoder motor
    * @param {uint8_t} id Encoder motor ID 0 ~ 3
    * @param {int32_t} position Position -2147483648 ~ 2147483647
    * @return int 0: success, other: fail
    */
    int encoder_motor_set_position(uint8_t id, int32_t position);
    /**
    * @description: Get encoder value
    * @param {uint8_t} id Encoder motor ID 0 ~ 3
    * @return int32_t Position -2147483648 ~ 2147483647
    */
    int32_t encoder_motor_get_position(uint8_t id);
    /**
    * @description: Set the position PID parameters for the encoder motor
    * @param {uint8_t} id Encoder motor ID 0 ~ 3
    * @param {int32_t} kp Position PID parameter P -128 ~ 127
    * @param {int32_t} ki Position PID parameter I -128 ~ 127
    * @param {int32_t} kd Position PID parameter D -128 ~ 127
    * @return int 0: success, other: fail
    */
    int encoder_motor_set_position_pid(uint8_t id, int8_t kp, int8_t ki, int8_t kd);
    int encoder_motor_set_velocity_pid(uint8_t id, int8_t kp, int8_t ki, int8_t kd);

    uint8_t voice_get_state(void);

protected:
    virtual uint8_t writeReg(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len);
    virtual uint8_t readReg(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len);
    virtual uint8_t isOnline(uint8_t dev_addr);

private:
    const uint8_t DEV_ADDR;
    TwoWire *_wire = NULL;
};
