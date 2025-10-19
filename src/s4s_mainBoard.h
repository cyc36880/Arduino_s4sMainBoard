#pragma once

#include "Arduino.h"
#include "Wire.h"

#if defined(__AVR__) || defined(ARDUINO_ARCH_AVR)
// AVR 平台（如 Arduino Nano）不支持 std::string
#else
// 其他平台（如 ESP32、STM32）可能支持
#include <string>
#endif

/**
 * - 充电管理
 * - 氛围灯
 * - RTC 
 * - 舵机
 * - 陀螺仪
 * - 语音模块
 * - 编码电机0 （内置电机）
 * - 编码电机1 （内置电机）
 * - 编码电机2
 * - 编码电机3
 */


class s4s_mainBoard
{
public:
    s4s_mainBoard(uint8_t addr = 0x05);
    ~s4s_mainBoard();

public:
    void begin(TwoWire *wire) { _wire=wire;}
    void begin(int sda = -1, int scl = -1)
    {
#if defined(__AVR__) || defined(ARDUINO_ARCH_AVR)
        Wire.begin();
#else
        Wire.begin(sda, scl);
#endif
        _wire = &Wire;
    }

    /**
     * @description: 获取充电状态
     * @param is_charging 是否正在充电 0: not charging, 1: charging
     * @param charging_voltage 电池电量 0~100
     * @return int 0: success, other: fail
     */
    int charging_get_state(uint8_t * is_charging, uint8_t * charging_voltage); 

    /**
     * @description: 设置氛围灯状态
     * @param light 亮度 0~255；-1 表示不改变
     * @param color 颜色数组 [r, g, b] 0~255；NULL 表示不改变
     * @return int 0: success, other: fail
     */
    int ambient_light_set_state(int light, uint8_t color[3]);
    int ambient_light_set_state(int light, uint8_t r, uint8_t g, uint8_t b);

    /**
     * @description: 设置RTC时间
     * @param year 年 0~99
     * @param month 月 1~12
     * @param day 日 1~31
     * @return int 0: success, other: fail
     */
    int rtc_set_data(uint8_t year, uint8_t month, uint8_t day);
    int rtc_set_time(uint8_t hour, uint8_t minute, uint8_t second);
    int rtc_get_data(uint8_t * year, uint8_t * month, uint8_t * day, uint8_t *week);
    int rtc_get_time(uint8_t * hour, uint8_t * minute, uint8_t * second);

    /**
     * @description: 设置舵机角度
     * @param servo_id 舵机ID 0~1
     * @param angle 角度 0~180
     * @return int 0: success, other: fail
     */
    int servo_set_angle(uint8_t id, uint8_t angle);

    /**
     * @description: 启用陀螺仪
     * @param enable 0: disable, 1: enable
     * @return int 0: success, other: fail
     */
    int gyro_enable(uint8_t enable);
    /**
     * @description: 设置陀螺仪状态
     * @param state 状态 0:空闲，1：开始校准，2：重置偏航角
     * @return int 0: success, other: fail
     */
    int gyro_set_state(uint8_t state);
    int gyro_get_state(uint8_t * state);
    int gyro_get_acc(int16_t * accX, int16_t * accY, int16_t * accZ);
    int gyro_get_gyro(int16_t * gyroX, int16_t * gyroY, int16_t * gyroZ);
    int gyro_get_angle(int16_t * angleX, int16_t * angleY, int16_t * angleZ);

    /**
     * @description: 设置编码电机模式
     * @param {uint8_t} id 编码电机ID 0~3
     * @param {uint8_t} mode 模式 0: 直流电机，1：位置控制，2：速度控制
     * @return int 0: success, other: fail
     */    
    int encoder_motor_set_mode(uint8_t id, uint8_t mode);
    /**
     * @description: 设置编码电机 功率/速度
     * @param {uint8_t} id 编码电机ID 0~3
     * @param {int16_t} power 功率 -1000~1000
     * @return int 0: success, other: fail
     */
    int encoder_motor_set_power(uint8_t id, int16_t power);
    /**
     * @description: 设置编码电机需要转动到的 位置
     * @param {uint8_t} id 编码电机ID 0~3
     * @param {int32_t} position 位置 -2147483648~2147483647
     * @return int 0: success, other: fail
     */
    int encoder_motor_set_position(uint8_t id, int32_t position);
    /**
     * @description: 获取编码器值
     * @param {uint8_t} id 编码电机ID 0~3
     * @param {int32_t} *position 位置 -2147483648~2147483647
     * @return int 0: success, other: fail
     */
    int encoder_motor_get_position(uint8_t id, int32_t *position);
    /**
     * @description: 设置编码电机位置PID参数
     * @param {uint8_t} id 编码电机ID 0~3
     * @param {int32_t} kp 位置PID参数P -128~127
     * @param {int32_t} ki 位置PID参数I -128~127
     * @param {int32_t} kd 位置PID参数D -128~127
     * @return int 0: success, other: fail
     */
    int encoder_motor_set_position_pid(uint8_t id, int8_t kp, int8_t ki, int8_t kd);
    int encoder_motor_set_velocity_pid(uint8_t id, int8_t kp, int8_t ki, int8_t kd);

protected:
    virtual uint8_t writeReg(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len);
    virtual uint8_t readReg(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len);
    virtual uint8_t isOnline(uint8_t dev_addr);

private:
    const uint8_t DEV_ADDR;
    TwoWire *_wire = NULL;
};
