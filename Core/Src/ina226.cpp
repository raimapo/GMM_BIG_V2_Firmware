#include <ina226.h>


INA226::INA226(I2C_HandleTypeDef* hi2c){
	_i2c = hi2c;
}

bool INA226::init(uint8_t address)
{
    inaAddress = address;
    return true;
}

bool INA226::configure(ina226_averages_t avg, ina226_busConvTime_t busConvTime, ina226_shuntConvTime_t shuntConvTime, ina226_mode_t mode)
{
    uint16_t config = 0;

    config |= (avg << 9 | busConvTime << 6 | shuntConvTime << 3 | mode);

    vBusMax = 36;
    vShuntMax = 0.08192f;

    writeRegister16(INA226_REG_CONFIG, config);

    return true;
}

bool INA226::calibrate(float rShuntValue, float iMaxExpected)
{
    uint16_t calibrationValue;
    rShunt = rShuntValue;

    float  minimumLSB;
    //float iMaxPossible,

    //iMaxPossible = vShuntMax / rShunt;

    minimumLSB = iMaxExpected / 32767;

    currentLSB = (uint16_t)(minimumLSB * 100000000);
    currentLSB /= 100000000;
    currentLSB /= 0.0001;
    currentLSB = ceil(currentLSB);
    currentLSB *= 0.0001;

    powerLSB = currentLSB * 25;

    calibrationValue = (uint16_t)((0.00512) / (currentLSB * rShunt));

    writeRegister16(INA226_REG_CALIBRATION, calibrationValue);

    return true;
}

void INA226::checkConfig()
{

  printf("Mode: ");
  switch (getMode())
  {
    case INA226_MODE_POWER_DOWN:      printf("Power-Down\n"); break;
    case INA226_MODE_SHUNT_TRIG:      printf("Shunt Voltage, Triggered\n"); break;
    case INA226_MODE_BUS_TRIG:        printf("Bus Voltage, Triggered\n"); break;
    case INA226_MODE_SHUNT_BUS_TRIG:  printf("Shunt and Bus, Triggered\n"); break;
    case INA226_MODE_ADC_OFF:         printf("ADC Off\n"); break;
    case INA226_MODE_SHUNT_CONT:      printf("Shunt Voltage, Continuous\n"); break;
    case INA226_MODE_BUS_CONT:        printf("Bus Voltage, Continuous\n"); break;
    case INA226_MODE_SHUNT_BUS_CONT:  printf("Shunt and Bus, Continuous\n"); break;
    default: printf("unknown\n");
  }

  printf("Samples average: ");
  switch (getAverages())
  {
    case INA226_AVERAGES_1:           printf("1 sample\n"); break;
    case INA226_AVERAGES_4:           printf("4 samples\n"); break;
    case INA226_AVERAGES_16:          printf("16 samples\n"); break;
    case INA226_AVERAGES_64:          printf("64 samples\n"); break;
    case INA226_AVERAGES_128:         printf("128 samples\n"); break;
    case INA226_AVERAGES_256:         printf("256 samples\n"); break;
    case INA226_AVERAGES_512:         printf("512 samples\n"); break;
    case INA226_AVERAGES_1024:        printf("1024 samples\n"); break;
    default: printf("unknown\n");
  }

  printf("Bus conversion time: ");
  switch (getBusConversionTime())
  {
    case INA226_BUS_CONV_TIME_140US:  printf("140uS\n"); break;
    case INA226_BUS_CONV_TIME_204US:  printf("204uS\n"); break;
    case INA226_BUS_CONV_TIME_332US:  printf("332uS\n"); break;
    case INA226_BUS_CONV_TIME_588US:  printf("558uS\n"); break;
    case INA226_BUS_CONV_TIME_1100US: printf("1.100ms\n"); break;
    case INA226_BUS_CONV_TIME_2116US: printf("2.116ms\n"); break;
    case INA226_BUS_CONV_TIME_4156US: printf("4.156ms\n"); break;
    case INA226_BUS_CONV_TIME_8244US: printf("8.244ms\n"); break;
    default: printf("unknown\n");
  }

  printf("Shunt conversion time: ");
  switch (getShuntConversionTime())
  {
    case INA226_SHUNT_CONV_TIME_140US:  printf("140uS\n"); break;
    case INA226_SHUNT_CONV_TIME_204US:  printf("204uS\n"); break;
    case INA226_SHUNT_CONV_TIME_332US:  printf("332uS\n"); break;
    case INA226_SHUNT_CONV_TIME_588US:  printf("558uS\n"); break;
    case INA226_SHUNT_CONV_TIME_1100US: printf("1.100ms\n"); break;
    case INA226_SHUNT_CONV_TIME_2116US: printf("2.116ms\n"); break;
    case INA226_SHUNT_CONV_TIME_4156US: printf("4.156ms\n"); break;
    case INA226_SHUNT_CONV_TIME_8244US: printf("8.244ms\n"); break;
    default: printf("unknown\n\n");
  }

  printf("Max possible current: %f A\n", getMaxPossibleCurrent());
  printf("Max current: %f A\n", getMaxCurrent());
  printf("Max shunt voltage: %f V\n", getMaxShuntVoltage());
  printf("Max power: %f W\n\n", getMaxPower());
}



float INA226::getMaxPossibleCurrent(void)
{
    return (vShuntMax / rShunt);
}

float INA226::getMaxCurrent(void)
{
    float maxCurrent = (currentLSB * 32767);
    float maxPossible = getMaxPossibleCurrent();

    if (maxCurrent > maxPossible)
    {
        return maxPossible;
    } else
    {
        return maxCurrent;
    }
}

float INA226::getMaxShuntVoltage(void)
{
    float maxVoltage = getMaxCurrent() * rShunt;

    if (maxVoltage >= vShuntMax)
    {
        return vShuntMax;
    } else
    {
        return maxVoltage;
    }
}

float INA226::getMaxPower(void)
{
    return (getMaxCurrent() * vBusMax);
}

float INA226::readBusPower(void)
{
    return (readRegister16(INA226_REG_POWER) * powerLSB);
}

float INA226::readShuntCurrent(void)
{
    return (readRegister16(INA226_REG_CURRENT) * currentLSB);
}

float INA226::readShuntVoltage(void)
{
    float voltage;

    voltage = readRegister16(INA226_REG_SHUNTVOLTAGE);

    return (voltage * 0.0000025);
}

float INA226::readBusVoltage(void)
{
    int16_t voltage;

    voltage = readRegister16(INA226_REG_BUSVOLTAGE);

    return (voltage * 0.00125);
}

ina226_averages_t INA226::getAverages(void)
{
    uint16_t value;

    value = readRegister16(INA226_REG_CONFIG);
    value &= 0b0000111000000000;
    value >>= 9;

    return (ina226_averages_t)value;
}

ina226_busConvTime_t INA226::getBusConversionTime(void)
{
    uint16_t value;

    value = readRegister16(INA226_REG_CONFIG);
    value &= 0b0000000111000000;
    value >>= 6;

    return (ina226_busConvTime_t)value;
}

ina226_shuntConvTime_t INA226::getShuntConversionTime(void)
{
    uint16_t value;

    value = readRegister16(INA226_REG_CONFIG);
    value &= 0b0000000000111000;
    value >>= 3;

    return (ina226_shuntConvTime_t)value;
}

ina226_mode_t INA226::getMode(void)
{
    uint16_t value;

    value = readRegister16(INA226_REG_CONFIG);
    value &= 0b0000000000000111;

    return (ina226_mode_t)value;
}

void INA226::setMaskEnable(uint16_t mask)
{
    writeRegister16(INA226_REG_MASKENABLE, mask);
}

uint16_t INA226::getMaskEnable(void)
{
    return readRegister16(INA226_REG_MASKENABLE);
}

void INA226::enableShuntOverLimitAlert(void)
{
    writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_SOL);
}

void INA226::enableShuntUnderLimitAlert(void)
{
    writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_SUL);
}

void INA226::enableBusOvertLimitAlert(void)
{
    writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_BOL);
}

void INA226::enableBusUnderLimitAlert(void)
{
    writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_BUL);
}

void INA226::enableOverPowerLimitAlert(void)
{
    writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_POL);
}

void INA226::enableConversionReadyAlert(void)
{
    writeRegister16(INA226_REG_MASKENABLE, INA226_BIT_CNVR);
}

void INA226::setBusVoltageLimit(float voltage)
{
    uint16_t value = voltage / 0.00125;
    writeRegister16(INA226_REG_ALERTLIMIT, value);
}

void INA226::setShuntVoltageLimit(float voltage)
{
    uint16_t value = voltage * 25000;
    writeRegister16(INA226_REG_ALERTLIMIT, value);
}

void INA226::setPowerLimit(float watts)
{
    uint16_t value = watts / powerLSB;
    writeRegister16(INA226_REG_ALERTLIMIT, value);
}

void INA226::setAlertInvertedPolarity(bool inverted)
{
    uint16_t temp = getMaskEnable();

    if (inverted)
    {
        temp |= INA226_BIT_APOL;
    } else
    {
        temp &= ~INA226_BIT_APOL;
    }

    setMaskEnable(temp);
}

void INA226::setAlertLatch(bool latch)
{
    uint16_t temp = getMaskEnable();

    if (latch)
    {
        temp |= INA226_BIT_LEN;
    } else
    {
        temp &= ~INA226_BIT_LEN;
    }

    setMaskEnable(temp);
}

bool INA226::isMathOverflow(void)
{
    return ((getMaskEnable() & INA226_BIT_OVF) == INA226_BIT_OVF);
}

bool INA226::isAlert(void)
{
    return ((getMaskEnable() & INA226_BIT_AFF) == INA226_BIT_AFF);
}

int16_t INA226::readRegister16(uint8_t reg)
{
    int16_t value;
    uint8_t data[2]={0};

    I2C_receive(_i2c, inaAddress, reg, data, 2, 0XFFFF);

    value = data[0] << 8 | data[1];

    return value;
}

void INA226::writeRegister16(uint8_t reg, uint16_t val)
{
    uint8_t vla;
    vla = (uint8_t)val;
    val >>= 8;
    uint8_t data[3]={0};
    data[0]=reg;
    data[1]=val;
    data[2]=vla;

    I2C_Write(_i2c, inaAddress, data, 3, 10);
}

//void I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c,  uint16_t DevAddress, uint32_t TestCount, uint32_t timeout);
//void I2C_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *data, uint16_t Size, uint32_t Timeout);
//void I2C_receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t startreg, uint8_t *data, uint16_t Size, uint32_t Timeout);
