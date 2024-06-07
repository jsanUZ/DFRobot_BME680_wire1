/**
 * @file DFRobot_BME680_I2C.cpp
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author Frank(jiehan.guo@dfrobot.com)
 * @version  V1.0
 * @date  2017-12-04
 * @url https://github.com/DFRobot/DFRobot_BME680
 */

#include "DFRobot_BME680_I2C.h"


static int8_t bme680_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  Wire1.begin();
  Wire1.beginTransmission(dev_id);
  Wire1.write(reg_addr);
  Wire1.endTransmission();
  Wire1.requestFrom(dev_id, (uint8_t)len);
  while(Wire1.available()) {
    *data = Wire1.read();
    data ++;
  }
  return 0;
}


static int8_t bme680_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  Wire1.begin();
  Wire1.beginTransmission(dev_id);
  Wire1.write(reg_addr);
  while(len --) {
    Wire1.write(*data);
    data ++;
  }
  Wire1.endTransmission();
  return 0;
}


DFRobot_BME680_I2C::DFRobot_BME680_I2C(uint8_t I2CAddr_) :
                    DFRobot_BME680(bme680_i2c_read, bme680_i2c_write, bme680_delay_ms, eBME680_INTERFACE_I2C)
{
  bme680_I2CAddr = I2CAddr_;
}


void DFRobot_BME680_I2C::setConvertAndUpdate()
{
  bsec_iot_init(0.33333f, 0, bme680_i2c_write, bme680_i2c_read, bme680_delay_ms, bme680_I2CAddr, BME680_I2C_INTF);
}

