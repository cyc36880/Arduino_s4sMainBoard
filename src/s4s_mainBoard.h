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
    s4s_mainBoard();
     ~ s4s_mainBoard();
public:
    enum COLOR_TYPE
    {
        COLOR_NONE=0,
        COLOR_RED,
        COLOR_GREEN,
        COLOR_BLUE,
        COLOR_YELLOW,
        COLOR_PURPLE = COLOR_YELLOW + 2,
    };
public:
    void begin(TwoWire *wire) { _wire=wire;}
    void begin(void)
    {
#if defined(__AVR__) || defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_RENESAS)
        Wire.begin();
#else
        Wire.begin(42, 41);
        Wire1.begin(1, 2);
        _wire2 = &Wire1;
#endif
        _wire = &Wire;
    }

    /**
    * @description: Get battery level
    * @param charging_voltage Battery level 0 ~ 100
    * @return int 0: success, other: fail
    */
    int charging_get_state(uint8_t * charging_voltage); 

    /**
    * @description: Set the ambient light status
    * @param light Brightness 0 ~ 255; -1 means do not change
    * @param color Color array [r, g, b] 0 ~ 255; NULL means do not change
    * @return int 0: success, other: fail
    */
    int ambient_light_set_state(int light, uint8_t color[3]);
    int ambient_light_set_state(int light, uint8_t r, uint8_t g, uint8_t b);

    /**
    * @description: Set RTC time
    * @param year Year 0 ~ 99
    * @param month Month 1 ~ 12
    * @param day Day 1 ~ 31
    * @return int 0: success, other: fail
    */
    int rtc_set_data(uint8_t year, uint8_t month, uint8_t day);
    int rtc_set_time(uint8_t hour, uint8_t minute, uint8_t second);
    int rtc_get_data(uint8_t * year, uint8_t * month, uint8_t * day, uint8_t * week);
    int rtc_get_time(uint8_t * hour, uint8_t * minute, uint8_t * second);

    /**
    * @description: Set the servo angle
    * @param servo_id Servo ID 0 ~ 1
    * @param angle Angle 0 ~ 180
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
     * @description: Get voice commands
     * @return {*}
     */    
    uint8_t voice_get_state(void);


    /**
     * @description: Get the rotational angle of the encoded motor
     * @param id  Encoder motor ID 0 ~ 3
     * @param angle Rotational angle
     * @return int 0: success, other: fail
     */    
    int encoder_motor_get_angle(uint8_t id, int32_t * angle);
    /**
     * @description: Get the speed of the encoded motor
     * @param id  Encoder motor ID 0 ~ 3
     * @param speed Speed
     * @return int 0: success, other: fail
     */    
    int encoder_motor_get_speed(uint8_t id, int16_t * speed);
    /**
     * @description: Get the current power of the encoded motor
     * @param id  Encoder motor ID 0 ~ 3
     * @param power Current power
     * @return int 0: success, other: fail
     */    
    int encoder_motor_get_power(uint8_t id, uint16_t * power);
    /**
     * @description: Reset the rotation angle of the encoder motor
     * @param id  Encoder motor ID 0 ~ 3
     * @return int 0: success, other: fail
     */    
    int encoder_motor_reset_angle(uint8_t id);
    /**
     * @description: Set the motion commands for the encoded motor
     * @param id  Encoder motor ID 0 ~ 3
     * @param action Command
     *  @note 
     *      0:stop
     *      speed : 1:forward 2:backward
     *      power : 3:forward 4:backward
     *      ring  : 5:forward 6:backward
     *      angle : 7:forward 8:backward
     *      second: 9:forward 10:backward
     * @return int 0: success, other: fail
     */    
    int encoder_motor_set_action(uint8_t id, uint8_t action);
    /**
     * @description: Set the motion speed of the encoder motor
     * @param id  Encoder motor ID 0 ~ 3
     * @param speed Speed
     * @return int 0: success, other: fail
     */    
    int encoder_motor_set_speed(uint8_t id, uint16_t speed);
    /**
     * @description: Set the power of the encoder motor
     * @param id  Encoder motor ID 0 ~ 3
     * @param power Power 0 ~ 100
     * @return int 0: success, other: fail
     */ 
    int encoder_motor_set_power(uint8_t id, uint8_t power);
    /**
     * @description: Set the ring of the encoder motor
     * @param id  Encoder motor ID 0 ~ 3
     * @param ring Ring 0 ~ 65535
     * @return int 0: success, other: fail
     */  
    int encoder_motor_set_ring(uint8_t id, uint16_t ring);
    /**
     * @description: Set the relative angle for the encoder motor operation
     * @param id  Encoder motor ID 0 ~ 3
     * @param relativeAngle Relative angle
     * @return int 0: success, other: fail
     */    
    int encoder_motor_set_relative_angle(uint8_t id, uint16_t relativeAngle);
    /**
     * @description: Set the running duration of the encoder motor
     * @param id  Encoder motor ID 0 ~ 3
     * @param runTime Running duration, The unit is seconds
     * @return int 0: success, other: fail
     */
    int encoder_motor_set_run_time(uint8_t id, uint16_t runTime);


    /**
     * @description: Set the motion commands for the encoded motor
     * @param action Command
     * @note 
     *      0:stop
     *      speed : 1:forward 2:backward 3:turn left 4:turn right
     *      second: 5:forward 6:backward 7:turn left 8:turn right
     * @return int 0: success, other: fail
     */    
    int encoder_motor_pair_set_action(uint8_t action);
    /**
     * @description: Set the IDs for the left and right motor pairs
     * @param l_id Left motor ID 0 ~ 3
     * @param r_id Right motor ID 0 ~ 3
     * @return int 0: success, other: fail
     */    
    int encoder_motor_pair_set_group(uint8_t l_id, uint8_t r_id);
    /**
     * @description: Set the motor speed for the left and right motors
     * @param l_speed Left motor speed
     * @param r_speed Right motor speed
     * @return int 0: success, other: fail
     */    
    int encoder_motor_pair_set_run_speed(uint16_t l_speed, uint16_t r_speed);
    /**
     * @description: Set motor for motion time
     * @param runTime Running time 0 ~ 65535, unit: ms
     * @return int 0: success, other: fail
     */    
    int enmcoder_motor_pair_set_run_time(uint16_t runTime);


    /**
     * @description: Get the distance of the ultrasonic wave
     * @return The unit is centimeters
     */    
    uint16_t ultr_get_distance(void);
    void ultr_set_color(uint8_t light, uint8_t color[3]);
    void ultr_set_color(uint8_t light, uint8_t r, uint8_t g, uint8_t b);

    void gray_grayStudy(void);
    void gray_binaryStudy(void);
    void gray_colorStudy(enum COLOR_TYPE colorType);
    void gray_clearColor(void);
    void gray_getGrayData(uint8_t data[4]);
    void gray_getColorData(uint8_t data[4]);
    void gray_getBlack(uint8_t data[4]);
    void gray_getPhotosensitive(uint8_t data[4]);

protected:
    virtual uint8_t writeData(uint8_t dev_addr, uint8_t *data, uint16_t len);
    virtual uint8_t readData(uint8_t dev_addr, uint8_t *data, uint16_t len);
    virtual uint8_t writeReg(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len);
    virtual uint8_t readReg(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len);
    virtual uint8_t isOnline(uint8_t dev_addr);

private:
    const uint8_t MAINBOARD_ADDR = 0x0F;
    const uint8_t ULTR_ADDR = 0x57;
    const uint8_t GYRO_ADDR = 0x6F;
    TwoWire *_wire  = NULL;
    TwoWire *_wire2 = NULL;
};
