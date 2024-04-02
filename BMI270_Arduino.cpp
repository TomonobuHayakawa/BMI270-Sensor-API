/***************************************************************************/
/*!             Header files
 ****************************************************************************/
#include "BMI270_Arduino.h"

/***************************************************************************/
/*!               Callbacks for the device access
 ****************************************************************************/

extern "C"{

int8_t bmi2_spi_read(uint8_t reg_addr, uint8_t *data, uint16_t len, void *intf_ptr)
{
   uint8_t dev_id = *(uint8_t*)intf_ptr;

  if ((data == NULL) || (len == 0))
    return BMI2_E_NULL_PTR;
  digitalWrite(dev_id, LOW);
  SPI.transfer(reg_addr);
  for (uint16_t i = 0; i < len; i++)
  {
    data[i] = SPI.transfer(0xff);
  }
  digitalWrite(dev_id, HIGH);
  return BMI2_OK;
}

int8_t bmi2_spi_write(uint8_t reg_addr, const uint8_t *data, uint16_t len, void *intf_ptr)
{
   uint8_t dev_id = *(uint8_t*)intf_ptr;

  if ((data == NULL) || (len == 0))
    return BMI2_E_NULL_PTR;
  digitalWrite(dev_id, LOW);
  SPI.transfer(reg_addr);
  for (uint16_t i = 0; i < len; i++)
    SPI.transfer(data[i]);
  digitalWrite(dev_id, HIGH);
  return BMI2_OK;
}

int8_t bmi2_i2c_read(uint8_t reg_addr, uint8_t *data, uint16_t len, void *intf_ptr)
{
   uint8_t dev_id = *(uint8_t*)intf_ptr;

  if ((data == NULL) || (len == 0) || (len > 32)) {
    return BMI2_E_NULL_PTR;
  }
  uint8_t bytes_received;

  Wire.beginTransmission(dev_id);
  Wire.write(reg_addr);
  if (Wire.endTransmission() == 0) {
    bytes_received = Wire.requestFrom(dev_id, len);
    // Optionally, throw an error if bytes_received != len
    for (uint16_t i = 0; i < bytes_received; i++)
    {
      data[i] = Wire.read();
    }
  } else {
    return BMI2_E_NULL_PTR;
  }

  return BMI2_OK;
}

int8_t bmi2_i2c_write(uint8_t reg_addr, const uint8_t *data, uint16_t len, void *intf_ptr)
{
   uint8_t dev_id = *(uint8_t*)intf_ptr;

  if ((data == NULL) || (len == 0) || (len > 32)) {
    return BMI2_E_NULL_PTR;
  }

  Wire.beginTransmission(dev_id);
  Wire.write(reg_addr);
  for (uint16_t i = 0; i < len; i++)
  {
    Wire.write(data[i]);
  }
  if (Wire.endTransmission() != 0) {
    return BMI2_E_NULL_PTR;
  }

  return BMI2_OK;
}

void bmi2_delay_us(uint32_t period)
{
  delayMicroseconds(period);
}

}

/***************************************************************************/
/*!               Class method
 ****************************************************************************/

int8_t BMI270Class::begin(BMI270Mode mode,uint8_t addr)
{

  if(mode == BMI270_I2C){

    Wire.begin();

    bmi2.dev_id = addr;
    bmi2.read = bmi2_i2c_read;
    bmi2.write = bmi2_i2c_write;
    bmi2.delay_us =  bmi2_delay_us;
    bmi2.intf = BMI2_I2C_INTF;
    bmi2.read_write_len = 30; // Limitation of the Wire library
    bmi2.config_file_ptr = NULL; // Use the default BMI270 config file
  	bmi2.intf_ptr = &bmi2.dev_id;

  }else{

  //  int cs_pin = addr;

  //  /* Start of SPI configuration */
  //  SPI.begin();
  //  pinMode(cs_pin, OUTPUT);
  //  digitalWrite(cs_pin, LOW);
  //  delay(1);
  //  digitalWrite(cs_pin, HIGH); // Toggle the chip select to switch into SPI mode
  //  delay(10);
  //
  //  bmi2.dev_id = cs_pin;
  //  bmi2.read = bmi2_spi_read;
  //  bmi2.write = bmi2_spi_write;
  //  bmi2.delay_us = bmi2_delay_us;
  //  bmi2.intf = BMI2_SPI_INTERFACE;
  //  bmi2.read_write_len = 8192;
  //  bmi2.config_file_ptr = NULL; // Use the default BMI270 config file
  //  /* End of SPI configuration */

  /* Start of I2C configuration */
  // pinMode(PIN_SPI_MISO, OUTPUT);
  // digitalWrite(PIN_SPI_MISO, LOW); //  Expected state of the SDO line
  // pinMode(cs_pin, OUTPUT);
  // digitalWrite(cs_pin, HIGH); //  Expected state of the CS line

  }

  return bmi270_init(&bmi2);

}

int8_t BMI270Class::set_sensor_config(struct bmi2_sens_config *config, uint8_t n_sens)
{
  int8_t rslt = bmi270_get_sensor_config(config, n_sens, &bmi2);
  if (rslt != BMI2_OK) return rslt;

  for(int i= 0;i < n_sens;i++){
    switch(config[i].type) {
    case BMI2_ACCEL:
      acc_range = 0x1 << (config[i].cfg.acc.range + 1);
      break;
    case BMI2_GYRO:
      gyr_range = 2000 >> config[i].cfg.gyr.range ;
      break;
    default:
      ;
    }
  }

  return bmi270_set_sensor_config(config, n_sens, &bmi2);

}

int8_t BMI270Class::set_int_pin_config(uint8_t pin_type, bmi2_int_pin_cfg* pin_cfg)
{

  bmi2_int_pin_config config;
  int8_t rslt = bmi2_get_int_pin_config(&config,&bmi2);
  if (rslt != BMI2_OK) return rslt;

  config.pin_type  = pin_type;
  config.int_latch = BMI2_INT_NON_LATCH;

  if(pin_type == BMI2_INT1){
    config.pin_cfg[0]   = *pin_cfg;
  }else if(pin_type == BMI2_INT2){
    config.pin_cfg[1]   = *pin_cfg;
  }else{
    config.pin_cfg[0]   = *pin_cfg;
    config.pin_cfg[1]   = *pin_cfg;
  }

  return bmi2_set_int_pin_config(&config, &bmi2);

}

int8_t BMI270Class::get_int_pin_config(bmi2_int_pin_config* config)
{
  return bmi2_get_int_pin_config(config, &bmi2);
}

int8_t BMI270Class::get_sensor_config(struct bmi2_sens_config *sens_cfg, uint8_t n_sens)
{
  return bmi270_get_sensor_config(sens_cfg, n_sens, &bmi2);

}

int8_t BMI270Class::sensor_enable(const uint8_t *sens_list, uint8_t n_sens)
{
  return bmi270_sensor_enable(sens_list, n_sens, &bmi2);

}

int8_t BMI270Class::sensor_disable(const uint8_t *sens_list, uint8_t n_sens)
{
  return bmi270_sensor_disable(sens_list, n_sens, &bmi2);

}

int8_t BMI270Class::get_feature_data(struct bmi2_feat_sensor_data *feature_data, uint8_t n_sens)
{
  return bmi270_get_feature_data(feature_data, n_sens, &bmi2);

}

int8_t BMI270Class::bmi2_get_sensor_float(struct bmi2_sens_float *data)
{
  struct bmi2_sens_data sensor_data;

  int8_t rslt = bmi2_get_sensor_data(&sensor_data, &bmi2);
  if(rslt != BMI2_OK) return rslt;

  data->acc.x = lsb_to_mps2(sensor_data.acc.x);
  data->acc.y = lsb_to_mps2(sensor_data.acc.y);
  data->acc.z = lsb_to_mps2(sensor_data.acc.z);

  data->gyr.x = lsb_to_dps(sensor_data.gyr.x);
  data->gyr.y = lsb_to_dps(sensor_data.gyr.y);
  data->gyr.z = lsb_to_dps(sensor_data.gyr.z);

  return rslt;

}

int8_t BMI270Class::update_gyro_user_gain(const struct bmi2_gyro_user_gain_config *user_gain)
{
  return bmi270_update_gyro_user_gain(user_gain, &bmi2);

}

int8_t BMI270Class::read_gyro_user_gain(struct bmi2_gyro_user_gain_data *gyr_usr_gain)
{
  return bmi270_read_gyro_user_gain(gyr_usr_gain, &bmi2);

}

int8_t BMI270Class::map_feat_int(const struct bmi2_sens_int_config *sens_int, uint8_t n_sens)
{
  return bmi270_map_feat_int(sens_int, n_sens, &bmi2);

}

int8_t BMI270Class::map_data_int(uint8_t data_int, enum bmi2_hw_int_pin int_pin)
{
  return bmi2_map_data_int(data_int, int_pin, &bmi2);

}

float BMI270Class::lsb_to_mps2(int16_t val)
{
  float half_scale = ((float)(1 << bmi2.resolution) / 2.0f);

  return (GRAVITY_EARTH * val * acc_range) / half_scale;
}

float BMI270Class::lsb_to_dps(int16_t val)
{
  return ((float)val*(gyr_range / 32768.0));
}

