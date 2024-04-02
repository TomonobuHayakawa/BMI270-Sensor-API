#ifndef BMI270_ARDUINO_H_
#define BMI270_ARDUINO_H_

/***************************************************************************/
/*!             Header files
 ****************************************************************************/
#include <Arduino.h>

#include <SPI.h>
#include <Wire.h>

#include "bmi270.h"

/***************************************************************************/
/*!               Macro definitions
 ****************************************************************************/

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)

/*! Access mode */
typedef enum {
    BMI270_I2C = 0,
    BMI270_SPI,
} BMI270Mode;

/*! @name Structure to define accelerometer and gyroscope sensor axes and
 * sensor time for virtual frames
 */
struct bmi2_sens_axes_float
{
    /*! Data in x-axis */
    float x;

    /*! Data in y-axis */
    float y;

    /*! Data in z-axis */
    float z;

    /*! Sensor time for virtual frames */
    uint32_t virt_sens_time;
};

/*! @name Structure to define BMI2 sensor data */
struct bmi2_sens_float
{
    /*! Accelerometer axes data */
    struct bmi2_sens_axes_float acc;

    /*! Gyroscope axes data */
    struct bmi2_sens_axes_float gyr;

    /*! Auxiliary sensor data */
    uint8_t aux_data[BMI2_AUX_NUM_BYTES];

    /*! Sensor time */
    uint32_t sens_time;
};

/***************************************************************************/
/*!               Class
 ****************************************************************************/

class BMI270Class
{
public:

/*!
 * @details This API:
 *  Initializes BMI270 sensor.
 *
 * @param[in] mode      : Structure instance of bmi2_dev.
 * @param[in] addr      : I2C Address
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
  int8_t begin(BMI270Mode mode, uint8_t addr);


/**
 * @brief Enable / Disable features of the sensor
 */

/*!
 * @details This API selects the sensors/features to be enabled.
 *
 * @param[in]       sens_list   : Pointer to select the sensor/feature.
 * @param[in]       n_sens      : Number of sensors selected.
 *
 * @note Sensors/features that can be enabled.
 *
 *@verbatim
 *    sens_list                |  Values
 * ----------------------------|-----------
 * BMI2_ACCEL                  |  0
 * BMI2_GYRO                   |  1
 * BMI2_AUX                    |  2
 * BMI2_SIG_MOTION             |  3
 * BMI2_ANY_MOTION             |  4
 * BMI2_NO_MOTION              |  5
 * BMI2_STEP_DETECTOR          |  6
 * BMI2_STEP_COUNTER           |  7
 * BMI2_STEP_ACTIVITY          |  8
 * BMI2_GYRO_GAIN_UPDATE       |  9
 * BMI2_WRIST_GESTURE          |  19
 * BMI2_WRIST_WEAR_WAKE_UP     |  20
 * BMI2_GYRO_SELF_OFF          |  33
 *@endverbatim
 *
 * @note :
 * example  uint8_t sens_list[2]  = {BMI2_ACCEL, BMI2_GYRO};
 *           uint8_t n_sens        = 2;
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t sensor_enable(const uint8_t *sens_list, uint8_t n_sens);

/*!
 * @details This API selects the sensors/features to be disabled.
 *
 * @param[in]       sens_list   : Pointer to select the sensor/feature.
 * @param[in]       n_sens      : Number of sensors selected.
 *
 * @note Sensors/features that can be disabled.
 *
 *@verbatim
 *    sens_list                |  Values
 * ----------------------------|-----------
 * BMI2_ACCEL                  |  0
 * BMI2_GYRO                   |  1
 * BMI2_AUX                    |  2
 * BMI2_SIG_MOTION             |  3
 * BMI2_ANY_MOTION             |  4
 * BMI2_NO_MOTION              |  5
 * BMI2_STEP_DETECTOR          |  6
 * BMI2_STEP_COUNTER           |  7
 * BMI2_STEP_ACTIVITY          |  8
 * BMI2_GYRO_GAIN_UPDATE       |  9
 * BMI2_WRIST_GESTURE          |  19
 * BMI2_WRIST_WEAR_WAKE_UP     |  20
 * BMI2_GYRO_SELF_OFF          |  33
 *@endverbatim
 *
 * @note :
 * example  uint8_t sens_list[2]  = {BMI2_ACCEL, BMI2_GYRO};
 *           uint8_t n_sens        = 2;
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t sensor_disable(const uint8_t *sens_list, uint8_t n_sens);

/**
 * @brief Enable / Disable feature configuration of the sensor
 */

/*!
 * @details This API sets the sensor/feature configuration.
 *
 * @param[in]       sens_cfg     : Structure instance of bmi2_sens_config.
 * @param[in]       n_sens       : Number of sensors selected.
 *
 * @note Sensors/features that can be configured
 *
 *@verbatim
 *    sens_list                |  Values
 * ----------------------------|-----------
 * BMI2_SIG_MOTION             |  3
 * BMI2_ANY_MOTION             |  4
 * BMI2_NO_MOTION              |  5
 * BMI2_STEP_DETECTOR          |  6
 * BMI2_STEP_COUNTER           |  7
 * BMI2_STEP_ACTIVITY          |  8
 * BMI2_WRIST_GESTURE          |  19
 * BMI2_WRIST_WEAR_WAKE_UP     |  20
 * BMI2_STEP_COUNTER_PARAMS    |  28
 *@endverbatim
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t set_sensor_config(struct bmi2_sens_config *sens_cfg, uint8_t n_sens);

/*!
 * @details This API gets the sensor/feature configuration.
 *
 * @param[in]       sens_cfg     : Structure instance of bmi2_sens_config.
 * @param[in]       n_sens       : Number of sensors selected.
 *
 * @note Sensors/features whose configurations can be read.
 *
 *@verbatim
 *    sens_list                |  Values
 * ----------------------------|-----------
 * BMI2_SIG_MOTION             |  3
 * BMI2_ANY_MOTION             |  4
 * BMI2_NO_MOTION              |  5
 * BMI2_STEP_DETECTOR          |  6
 * BMI2_STEP_COUNTER           |  7
 * BMI2_STEP_ACTIVITY          |  8
 * BMI2_WRIST_GESTURE          |  19
 * BMI2_WRIST_WEAR_WAKE_UP     |  20
 * BMI2_STEP_COUNTER_PARAMS    |  28
 *@endverbatim
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t get_sensor_config(struct bmi2_sens_config *sens_cfg, uint8_t n_sens);

/**
 * @brief Get feature sensor data
 */

/*!
 * @details This API gets the feature data.
 *
 * @param[out] feature_data   : Structure instance of bmi2_feat_sensor_data.
 * @param[in]  n_sens         : Number of sensors selected.
 *
 * @note Sensors/features whose data can be read
 *
 *@verbatim
 *    sens_list                |  Values
 * ----------------------------|-----------
 * BMI2_STEP_COUNTER           |  7
 * BMI2_STEP_ACTIVITY          |  8
 * BMI2_WRIST_GESTURE          |  19
 * BMI2_NVM_STATUS             |  38
 * BMI2_VFRM_STATUS            |  39
 *@endverbatim
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t get_feature_data(struct bmi2_feat_sensor_data *feature_data, uint8_t n_sens);

/*!
 * @details This API gets the feature float data.
 *
 * @param[out] feature_data   : Structure instance of bmi2_feat_sensor_data.
 *
 * @note Sensors/features whose data can be read
 *
 *@verbatim
 *    sens_list                |  Values
 * ----------------------------|-----------
 * BMI2_STEP_COUNTER           |  7
 * BMI2_STEP_ACTIVITY          |  8
 * BMI2_WRIST_GESTURE          |  19
 * BMI2_NVM_STATUS             |  38
 * BMI2_VFRM_STATUS            |  39
 *@endverbatim
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bmi2_get_sensor_float(struct bmi2_sens_float *feature_data);

/**
 * @brief Set / Get Gyro User Gain of the sensor
 */

/*!
 * @details This API updates the gyroscope user-gain.
 *
 * @param[in] user_gain      : Structure that stores user-gain configurations.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t update_gyro_user_gain(const struct bmi2_gyro_user_gain_config *user_gain);

/*!
 * @details This API reads the compensated gyroscope user-gain values.
 *
 * @param[out] gyr_usr_gain   : Structure that stores gain values.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t read_gyro_user_gain(struct bmi2_gyro_user_gain_data *gyr_usr_gain);

/*!
 * @details This API maps/unmaps feature interrupts to that of interrupt pins.
 *
 * @param[in] sens_int     : Structure instance of bmi2_sens_int_config.
 * @param[in] n_sens       : Number of interrupts to be mapped.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t map_feat_int(const struct bmi2_sens_int_config *sens_int, uint8_t n_sens);

int8_t map_data_int(uint8_t data_int, enum bmi2_hw_int_pin int_pin);

/*!
 * @details This API sets:
 *        1) The input output configuration of the selected interrupt pin:
 *           INT1 or INT2.
 *        2) The interrupt mode: permanently latched or non-latched.
 *
 * @param[in] pin_type      : BMI2_INT1 or BMI2_INT2 or BMI2_BOTH
 * @param[in] pin_cfg       : Structure instance of bmi2_int_pin_cfg.
 * @param[in] dev           : Structure instance of bmi2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
  int8_t set_int_pin_config(uint8_t pin_type, bmi2_int_pin_cfg* pin_cfg);

/*!
 * @details This API gets:
 *        1) The input output configuration of the selected interrupt pin:
 *           INT1 or INT2.
 *        2) The interrupt mode: permanently latched or non-latched.
 *
 * @param[in,out] config  : Structure instance of bmi2_int_pin_config.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
  int8_t get_int_pin_config(bmi2_int_pin_config* config);

private:

  struct bmi2_dev bmi2;
  uint8_t          acc_range;
  uint16_t         gyr_range;

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
  float lsb_to_mps2(int16_t val);

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
  float lsb_to_dps(int16_t val);

};

#endif /* BMI270_ARDUINO_H_ */
