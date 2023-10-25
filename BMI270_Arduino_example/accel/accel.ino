#include "BMI270_Arduino.h"

BMI270Class BMI270;

/* Other functions */
int8_t configure_sensor(struct bmi2_dev *dev);
void panic_led_trap(void);
void print_rslt(int8_t rslt);

/* Sensor configuration */
int8_t configure_sensor()
{
  int8_t rslt;
  uint8_t sens_list[1] = { BMI2_ACCEL };

  struct bmi2_sens_config config;

  /* Configure the type of feature. */
  config.type = BMI2_ACCEL;

  /* NOTE: The user can change the following configuration parameters according to their requirement. */
  /* Set Output Data Rate */
  config.cfg.acc.odr = BMI2_ACC_ODR_200HZ;

  /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
  config.cfg.acc.range = BMI2_ACC_RANGE_2G;

  /* The bandwidth parameter is used to configure the number of sensor samples that are averaged
   * if it is set to 2, then 2^(bandwidth parameter) samples
   * are averaged, resulting in 4 averaged samples.
   * Note1 : For more information, refer the datasheet.
   * Note2 : A higher number of averaged samples will result in a lower noise level of the signal, but
   * this has an adverse effect on the power consumed.
   */
  config.cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;

  /* Enable the filter performance mode where averaging of samples
   * will be done based on above set bandwidth and ODR.
   * There are two modes
   *  0 -> Ultra low power mode
   *  1 -> High performance mode(Default)
   * For more info refer datasheet.
   */
  config.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

  /* Set the accel configurations. */
  rslt = BMI270.set_sensor_config(&config, 1);
  if (rslt != BMI2_OK) return rslt;

  rslt = BMI270.sensor_enable(sens_list, 1);
  if (rslt != BMI2_OK) return rslt;

  return rslt;
}

void setup(void)
{
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);

  int8_t rslt = BMI270.begin(BMI270_I2C,BMI2_I2C_SEC_ADDR);
  print_rslt(rslt);

  rslt = configure_sensor();
  print_rslt(rslt);
}

void loop(void)
{
  digitalWrite(LED_BUILTIN, LOW); // Flash the LED to show activity

  struct bmi2_sens_float sensor_data;
  int8_t rslt = BMI270.bmi2_get_sensor_float(&sensor_data);
  print_rslt(rslt);

  Serial.print(micros()); // Comment out this line if using the Serial plotter
  Serial.print(","); // Comment out this line if using the Serial plotter
  Serial.print(sensor_data.acc.x);
  Serial.print(",");
  Serial.print(sensor_data.acc.y);
  Serial.print(",");
  Serial.print(sensor_data.acc.z);
  Serial.println();
  digitalWrite(LED_BUILTIN, HIGH);

  usleep(10*1000);

}

void panic_led_trap(void)
{
  while (1)
  {
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
  }
}

void print_rslt(int8_t rslt)
{
  switch (rslt)
  {
    case BMI2_OK: return; /* Do nothing */ break;
    case BMI2_E_NULL_PTR:
      Serial.println("Error [" + String(rslt) + "] : Null pointer");
      panic_led_trap();
      break;
    case BMI2_E_COM_FAIL:
      Serial.println("Error [" + String(rslt) + "] : Communication failure");
      panic_led_trap();
      break;
    case BMI2_E_DEV_NOT_FOUND:
      Serial.println("Error [" + String(rslt) + "] : Device not found");
      panic_led_trap();
      break;
    case BMI2_E_OUT_OF_RANGE:
      Serial.println("Error [" + String(rslt) + "] : Out of range");
      panic_led_trap();
      break;
    case BMI2_E_ACC_INVALID_CFG:
      Serial.println("Error [" + String(rslt) + "] : Invalid accel configuration");
      panic_led_trap();
      break;
    case BMI2_E_GYRO_INVALID_CFG:
      Serial.println("Error [" + String(rslt) + "] : Invalid gyro configuration");
      panic_led_trap();
      break;
    case BMI2_E_ACC_GYR_INVALID_CFG:
      Serial.println("Error [" + String(rslt) + "] : Invalid accel/gyro configuration");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_SENSOR:
      Serial.println("Error [" + String(rslt) + "] : Invalid sensor");
      panic_led_trap();
      break;
    case BMI2_E_CONFIG_LOAD:
      Serial.println("Error [" + String(rslt) + "] : Configuration loading error");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_PAGE:
      Serial.println("Error [" + String(rslt) + "] : Invalid page ");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_FEAT_BIT:
      Serial.println("Error [" + String(rslt) + "] : Invalid feature bit");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_INT_PIN:
      Serial.println("Error [" + String(rslt) + "] : Invalid interrupt pin");
      panic_led_trap();
      break;
    case BMI2_E_SET_APS_FAIL:
      Serial.println("Error [" + String(rslt) + "] : Setting advanced power mode failed");
      panic_led_trap();
      break;
    case BMI2_E_AUX_INVALID_CFG:
      Serial.println("Error [" + String(rslt) + "] : Invalid auxilliary configuration");
      panic_led_trap();
      break;
    case BMI2_E_AUX_BUSY:
      Serial.println("Error [" + String(rslt) + "] : Auxilliary busy");
      panic_led_trap();
      break;
    case BMI2_E_SELF_TEST_FAIL:
      Serial.println("Error [" + String(rslt) + "] : Self test failed");
      panic_led_trap();
      break;
    case BMI2_E_REMAP_ERROR:
      Serial.println("Error [" + String(rslt) + "] : Remapping error");
      panic_led_trap();
      break;
    case BMI2_E_GYR_USER_GAIN_UPD_FAIL:
      Serial.println("Error [" + String(rslt) + "] : Gyro user gain update failed");
      panic_led_trap();
      break;
    case BMI2_E_SELF_TEST_NOT_DONE:
      Serial.println("Error [" + String(rslt) + "] : Self test not done");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_INPUT:
      Serial.println("Error [" + String(rslt) + "] : Invalid input");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_STATUS:
      Serial.println("Error [" + String(rslt) + "] : Invalid status");
      panic_led_trap();
      break;
    case BMI2_E_CRT_ERROR:
      Serial.println("Error [" + String(rslt) + "] : CRT error");
      panic_led_trap();
      break;
    case BMI2_E_ST_ALREADY_RUNNING:
      Serial.println("Error [" + String(rslt) + "] : Self test already running");
      panic_led_trap();
      break;
    case BMI2_E_CRT_READY_FOR_DL_FAIL_ABORT:
      Serial.println("Error [" + String(rslt) + "] : CRT ready for DL fail abort");
      panic_led_trap();
      break;
    case BMI2_E_DL_ERROR:
      Serial.println("Error [" + String(rslt) + "] : DL error");
      panic_led_trap();
      break;
    case BMI2_E_PRECON_ERROR:
      Serial.println("Error [" + String(rslt) + "] : PRECON error");
      panic_led_trap();
      break;
    case BMI2_E_ABORT_ERROR:
      Serial.println("Error [" + String(rslt) + "] : Abort error");
      panic_led_trap();
      break;
    case BMI2_E_GYRO_SELF_TEST_ERROR:
      Serial.println("Error [" + String(rslt) + "] : Gyro self test error");
      panic_led_trap();
      break;
    case BMI2_E_GYRO_SELF_TEST_TIMEOUT:
      Serial.println("Error [" + String(rslt) + "] : Gyro self test timeout");
      panic_led_trap();
      break;
    case BMI2_E_WRITE_CYCLE_ONGOING:
      Serial.println("Error [" + String(rslt) + "] : Write cycle ongoing");
      panic_led_trap();
      break;
    case BMI2_E_WRITE_CYCLE_TIMEOUT:
      Serial.println("Error [" + String(rslt) + "] : Write cycle timeout");
      panic_led_trap();
      break;
    case BMI2_E_ST_NOT_RUNING:
      Serial.println("Error [" + String(rslt) + "] : Self test not running");
      panic_led_trap();
      break;
    case BMI2_E_DATA_RDY_INT_FAILED:
      Serial.println("Error [" + String(rslt) + "] : Data ready interrupt failed");
      panic_led_trap();
      break;
    case BMI2_E_INVALID_FOC_POSITION:
      Serial.println("Error [" + String(rslt) + "] : Invalid FOC position");
      panic_led_trap();
      break;
    default:
      Serial.println("Error [" + String(rslt) + "] : Unknown error code");
      panic_led_trap();
      break;
  }
}
