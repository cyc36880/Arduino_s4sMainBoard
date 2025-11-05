#include "s4s_mainBoard.h"
#include <string.h>

/*******************
 * static variable
 ******************/
#define POINTER_SPACE_COPY(type, dst, src, len) \
    do                                          \
    {                                           \
        if (NULL == dst || NULL == src)         \
            break;                              \
        memcpy(dst, src, len * sizeof(type));   \
    } while (0)

static const uint8_t CHARGING_REG        = 0x00; // 充电管理
static const uint8_t AMBIENT_LIGHT_REG   = 0x05; // 氛围灯
static const uint8_t RTC_REG             = 0x0A; // RTC 时间
static const uint8_t SERVO_REG           = 0x0F; // 舵机
static const uint8_t GYROSCOPE_REG       = 0x14; // 陀螺仪
static const uint8_t VOICE_REG           = 0x1E; // 语音模块
static const uint8_t ENCODER_MOTOR_REG[] = {0x50, 0x5A, 0x64, 0x6E}; // 编码电机


/*******************
 * static function
 ******************/
static uint8_t writeReg(TwoWire *wire, uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (NULL == wire)
        return 1;
    wire->beginTransmission(dev_addr);
    wire->write(reg);
    while (len--)
    {
        wire->write(*(data++));
    }
    wire->endTransmission(true);

    return 0;
}

static uint8_t readReg(TwoWire *wire, uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (NULL == wire)
        return 1;
    wire->beginTransmission(dev_addr);
    wire->write(reg);
    wire->endTransmission(false);

    wire->requestFrom((uint8_t)dev_addr, (uint8_t)len);
    while (wire->available())
    {
        *data = wire->read();
        data++;
    }

    return 0;
}

// 0:online, otherwise offline
static uint8_t isOnline(TwoWire *wire, uint8_t dev_addr)
{
    if (NULL == wire)
        return 0xFF;
    wire->beginTransmission(dev_addr);
    return wire->endTransmission(true);
}


/************************
 * class protected
 ************************/
uint8_t s4s_mainBoard::writeReg(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len)
{
    return ::writeReg(this->_wire, dev_addr, reg, data, len);
}

uint8_t s4s_mainBoard::readReg(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len)
{
    return ::readReg(this->_wire, dev_addr, reg, data, len);
}

uint8_t s4s_mainBoard::isOnline(uint8_t dev_addr)
{
    return ::isOnline(this->_wire, dev_addr);
}


/************************
 * class public
 ************************/
s4s_mainBoard::s4s_mainBoard(uint8_t addr) : DEV_ADDR(addr)
{

}

s4s_mainBoard::~s4s_mainBoard()
{

}

int s4s_mainBoard::charging_get_state(uint8_t * is_charging, uint8_t * charging_voltage)
{
    int ret = 0;
    uint8_t data = 0;
    if (is_charging)
    {
        ret += this->readReg(DEV_ADDR, CHARGING_REG + 0, &data, 1);
        POINTER_SPACE_COPY(uint8_t, is_charging, &data, 1);
    }
    if (charging_voltage)
    {
        ret += this->readReg(DEV_ADDR, CHARGING_REG + 1, &data, 1);
        POINTER_SPACE_COPY(uint8_t, charging_voltage, &data, 1);
    }
    return ret;
}

int s4s_mainBoard::ambient_light_set_state(int light, uint8_t color[3])
{
    int ret = 0;
    if (light > 0)
    {
        uint8_t data = light > 255 ? 255 : light;
        ret += this->writeReg(DEV_ADDR, AMBIENT_LIGHT_REG + 0, &data, 1);
    }
    if (color)
    {
        ret += this->writeReg(DEV_ADDR, AMBIENT_LIGHT_REG + 1, color, 3);
    }
    return ret;
}

int s4s_mainBoard::ambient_light_set_state(int light, uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t data[] = {r, g, b};
    return this->ambient_light_set_state(light, data);
}

int s4s_mainBoard::rtc_set_data(uint8_t year, uint8_t month, uint8_t day)
{
    int ret = 0;
    uint8_t data[] = {0/*week*/, month, day, year};
    ret += this->writeReg(DEV_ADDR, RTC_REG+1, data, 4);
    return ret;
}

int s4s_mainBoard::rtc_set_time(uint8_t hour, uint8_t minute, uint8_t second)
{
    int ret = 0;
    uint8_t data[] = {hour, minute, second};
    ret += this->writeReg(DEV_ADDR, RTC_REG+0, data, 3);
    return ret;
}

int s4s_mainBoard::rtc_get_data(uint8_t * year, uint8_t * month, uint8_t * day, uint8_t *week)
{
    int ret = 0;
    uint8_t data[4] = {0};
    ret += this->readReg(DEV_ADDR, RTC_REG+1, data, 4);
    POINTER_SPACE_COPY(uint8_t, week, &data[0], 1);
    POINTER_SPACE_COPY(uint8_t, month, &data[1], 1);
    POINTER_SPACE_COPY(uint8_t, day, &data[2], 1);
    POINTER_SPACE_COPY(uint8_t, year, &data[3], 1);
    return ret;
}

int s4s_mainBoard::rtc_get_time(uint8_t * hour, uint8_t * minute, uint8_t * second)
{
    int ret=0;
    uint8_t data[3] = {0};
    ret += this->readReg(DEV_ADDR, RTC_REG+0, data, 3);
    POINTER_SPACE_COPY(uint8_t, hour, &data[0], 1);
    POINTER_SPACE_COPY(uint8_t, minute, &data[1], 1);
    POINTER_SPACE_COPY(uint8_t, second, &data[2], 1);
    return ret;
}

int s4s_mainBoard::servo_set_angle(uint8_t id, uint8_t angle)
{
    if (id > 1) return -1;

    int ret = 0;
    uint8_t data[] = {angle};
    ret += this->writeReg(DEV_ADDR, SERVO_REG+id, data, 1);
    return ret;
}

int s4s_mainBoard::gyro_enable(uint8_t enable)
{
    int ret = 0;
    uint8_t data[] = {enable};
    ret += this->writeReg(DEV_ADDR, GYROSCOPE_REG+0, data, 1);
    return ret;
}

int s4s_mainBoard::gyro_set_state(uint8_t state)
{
    int ret = 0;
    uint8_t data[] = {state};
    ret += this->writeReg(DEV_ADDR, GYROSCOPE_REG+1, data, 1);
    return ret;
}

int s4s_mainBoard::gyro_get_state(uint8_t * state)
{
    int ret = 0;
    uint8_t data = 0;
    ret += this->readReg(DEV_ADDR, GYROSCOPE_REG+1, &data, 1);
    POINTER_SPACE_COPY(uint8_t, state, &data, 1);
    return ret;
}

int s4s_mainBoard::gyro_get_acc(int16_t * accX, int16_t * accY, int16_t * accZ)
{
    int ret = 0;
    uint8_t data[6] = {0};
    ret += this->readReg(DEV_ADDR, GYROSCOPE_REG+2, data, 6);
    if (accX)
    {
        *accX = (int16_t)((data[0] << 8) | data[1]);
    }
    if (accY)
    {
        *accY = (int16_t)((data[2] << 8) | data[3]);
    }
    if (accZ)
    {
        *accZ = (int16_t)((data[4] << 8) | data[5]);
    }
    return ret;
}

int s4s_mainBoard::gyro_get_gyro(int16_t * gyroX, int16_t * gyroY, int16_t * gyroZ)
{
    int ret = 0;
    uint8_t data[6] = {0};
    ret += this->readReg(DEV_ADDR, GYROSCOPE_REG+3, data, 6);
    if (gyroX)
    {
        *gyroX = (int16_t)((data[0] << 8) | data[1]);
    }
    if (gyroY)
    {
        *gyroY = (int16_t)((data[2] << 8) | data[3]);
    }
    if (gyroZ)
    {
        *gyroZ = (int16_t)((data[4] << 8) | data[5]);
    }
    return ret;
}

int s4s_mainBoard::gyro_get_angle(int16_t * angleX, int16_t * angleY, int16_t * angleZ)
{
    int ret = 0;
    uint8_t data[6] = {0};
    ret += this->readReg(DEV_ADDR, GYROSCOPE_REG+4, data, 6);
    if (angleX)
    {
        *angleX = (int16_t)((data[0] << 8) | data[1]);
    }
    if (angleY)
    {
        *angleY = (int16_t)((data[2] << 8) | data[3]);
    }
    if (angleZ)
    {
        *angleZ = (int16_t)((data[4] << 8) | data[5]);
    }
    return ret;
}

int s4s_mainBoard::encoder_motor_set_mode(uint8_t id, uint8_t mode)
{
    if (id > 3) return -1;
    const uint8_t encoder_motor_reg_addr = ENCODER_MOTOR_REG[id];
    int ret = 0;
    uint8_t data[] = {mode};
    ret += this->writeReg(DEV_ADDR, encoder_motor_reg_addr+0, data, 1);
    return ret;
}

int s4s_mainBoard::encoder_motor_set_power(uint8_t id, int16_t power)
{
    if (id > 3) return -1;
    const uint8_t encoder_motor_reg_addr = ENCODER_MOTOR_REG[id];
    int ret = 0;
    uint8_t data[2] = {0};
    data[0] = (uint8_t)(power >> 8);
    data[1] = (uint8_t)(power & 0xFF);
    ret += this->writeReg(DEV_ADDR, encoder_motor_reg_addr+1, data, 2);
    return ret;
}

int s4s_mainBoard::encoder_motor_set_position(uint8_t id, int32_t position)
{
    if (id > 3) return -1;
    const uint8_t encoder_motor_reg_addr = ENCODER_MOTOR_REG[id];
    int ret = 0;
    uint8_t data[4] = {0};
    data[0] = (uint8_t)(position >> 24);
    data[1] = (uint8_t)(position >> 16);
    data[2] = (uint8_t)(position >> 8);
    data[3] = (uint8_t)(position & 0xFF);
    ret += this->writeReg(DEV_ADDR, encoder_motor_reg_addr+2, data, 4);
    return ret;
}

int32_t s4s_mainBoard::encoder_motor_get_position(uint8_t id)
{
    if (id > 3) return -1;
    const uint8_t encoder_motor_reg_addr = ENCODER_MOTOR_REG[id];
    int ret = 0;
    int32_t position=0;
    uint8_t data[4] = {0};
    this->readReg(DEV_ADDR, encoder_motor_reg_addr+3, data, 4);
    position = (int32_t)((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);
    return position;
}

int s4s_mainBoard::encoder_motor_set_position_pid(uint8_t id, int8_t kp, int8_t ki, int8_t kd)
{
    if (id > 3) return -1;
    const uint8_t encoder_motor_reg_addr = ENCODER_MOTOR_REG[id];
    int ret = 0;
    uint8_t data[3] = {0};
    data[0] = kp;
    data[1] = ki;
    data[2] = kd;
    ret += this->writeReg(DEV_ADDR, encoder_motor_reg_addr+4, data, 3);
    return ret;
}

int s4s_mainBoard::encoder_motor_set_velocity_pid(uint8_t id, int8_t kp, int8_t ki, int8_t kd)
{
    if (id > 3) return -1;
    const uint8_t encoder_motor_reg_addr = ENCODER_MOTOR_REG[id];
    int ret = 0;
    uint8_t data[3] = {0};
    data[0] = kp;
    data[1] = ki;
    data[2] = kd;
    ret += this->writeReg(DEV_ADDR, encoder_motor_reg_addr+5, data, 3);
    return ret;
}

uint8_t s4s_mainBoard::voice_get_state(void)
{
    uint8_t data = 0;
    delay(50);
    this->readReg(DEV_ADDR, VOICE_REG, &data, 1);
    return data;
}
