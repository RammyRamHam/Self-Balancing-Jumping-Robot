#include <limits.h>
#include <math.h>

#include "BNO055.h"
//#include "driver/i2c.h"
#include "i2c.h"
#include "config.h"
#include "utils.h"
#include "string.h"

/*!
 *  @brief  Sets up the HW
 *  @param  mode
 *          mode values
 *           [OPERATION_MODE_CONFIG,
 *            OPERATION_MODE_ACCONLY,
 *            OPERATION_MODE_MAGONLY,
 *            OPERATION_MODE_GYRONLY,
 *            OPERATION_MODE_ACCMAG,
 *            OPERATION_MODE_ACCGYRO,
 *            OPERATION_MODE_MAGGYRO,
 *            OPERATION_MODE_AMG,
 *            OPERATION_MODE_IMUPLUS,
 *            OPERATION_MODE_COMPASS,
 *            OPERATION_MODE_M4G,
 *            OPERATION_MODE_NDOF_FMC_OFF,
 *            OPERATION_MODE_NDOF]
 *  @return true if process is successful
 */
bool bno055Begin(bno055_opmode_t mode, bno055_t* dev) {
    i2cInit();
    // if (!i2c_dev->begin()) {
    //     return false;
    // }

    /* Make sure we have the right device */
    uint8_t id = bno055RegReadByte(BNO055_CHIP_ID_ADDR, dev);
    if (id != BNO055_ID) {
        delayMs(1000); // hold on for boot
        id = bno055RegReadByte(BNO055_CHIP_ID_ADDR, dev);
        if (id != BNO055_ID) {
            return false; // still not? ok bail
        }
    }


    /* Switch to config mode (just in case since this is the default) */
    bno055SetMode(OPERATION_MODE_CONFIG, dev);

    /* Reset */
    bno055RegWriteByte(BNO055_SYS_TRIGGER_ADDR, 0x20, dev);
    /* Delay incrased to 30ms due to power issues https://tinyurl.com/y375z699 */
    delayMs(30);
    while (bno055RegReadByte(BNO055_CHIP_ID_ADDR, dev) != BNO055_ID) {
        delayMs(10);
    }
    delayMs(50);

    /* Set to normal power mode */
    bno055RegWriteByte(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL, dev);
    delayMs(10);

    bno055RegWriteByte(BNO055_PAGE_ID_ADDR, 0, dev);

    /* Set the output units */
    /*
    uint8_t unitsel = (0 << 7) | // Orientation = Android
                        (0 << 4) | // Temperature = Celsius
                        (0 << 2) | // Euler = Degrees
                        (1 << 1) | // Gyro = Rads
                        (0 << 0);  // Accelerometer = m/s^2
    bno055RegWriteByte(BNO055_UNIT_SEL_ADDR, unitsel);
    */

    /* Configure axis mapping (see section 3.4) */
    /*
    bno055RegWriteByte(BNO055_AXIS_MAP_CONFIG_ADDR, REMAP_CONFIG_P2); // P0-P7, Default is P1
    delayMs(10);
    bno055RegWriteByte(BNO055_AXIS_MAP_SIGN_ADDR, REMAP_SIGN_P2); // P0-P7, Default is P1
    delayMs(10);
    */

    bno055RegWriteByte(BNO055_SYS_TRIGGER_ADDR, 0x0, dev);
    delayMs(10);
    // /* Set the requested operating mode (see section 3.3) */
    bno055SetMode(mode, dev);
    delayMs(20);

    return true;
}
  

void bno055Reset(bno055_t* dev) {
    bno055_opmode_t prevMode = dev->mode;

    bno055SetMode(OPERATION_MODE_CONFIG, dev);

    bno055RegWriteByte(BNO055_SYS_TRIGGER_ADDR, 0x20, dev);
    delayMs(30);
    while (bno055RegReadByte(BNO055_CHIP_ID_ADDR, dev) != BNO055_ID) {
        delayMs(10);
    }
    delayMs(50);

    bno055RegWriteByte(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL, dev);
    delayMs(10);

    bno055RegWriteByte(BNO055_PAGE_ID_ADDR, 0, dev);

    bno055RegWriteByte(BNO055_SYS_TRIGGER_ADDR, 0x0, dev);
    delayMs(10);

    bno055SetMode(prevMode, dev);
    delayMs(20);
}

/*!
 *  @brief  Puts the chip in the specified operating mode
 *  @param  mode
 *          mode values
 *           [OPERATION_MODE_CONFIG,
 *            OPERATION_MODE_ACCONLY,
 *            OPERATION_MODE_MAGONLY,
 *            OPERATION_MODE_GYRONLY,
 *            OPERATION_MODE_ACCMAG,
 *            OPERATION_MODE_ACCGYRO,
 *            OPERATION_MODE_MAGGYRO,
 *            OPERATION_MODE_AMG,
 *            OPERATION_MODE_IMUPLUS,
 *            OPERATION_MODE_COMPASS,
 *            OPERATION_MODE_M4G,
 *            OPERATION_MODE_NDOF_FMC_OFF,
 *            OPERATION_MODE_NDOF]
 */
void bno055SetMode(bno055_opmode_t mode, bno055_t* dev) {
  dev->mode = mode;
  bno055RegWriteByte(BNO055_OPR_MODE_ADDR, dev->mode, dev);
  delayMs(30);
}

/*!
 *  @brief  Changes the chip's axis remap
 *  @param  remapcode
 *          remap code possible values
 *          [REMAP_CONFIG_P0
 *           REMAP_CONFIG_P1 (default)
 *           REMAP_CONFIG_P2
 *           REMAP_CONFIG_P3
 *           REMAP_CONFIG_P4
 *           REMAP_CONFIG_P5
 *           REMAP_CONFIG_P6
 *           REMAP_CONFIG_P7]
 */
void bno055SetAxisRemap(bno055_axis_remap_config_t remapcode, bno055_t* dev) {
  bno055_opmode_t modeback = dev->mode;

  bno055SetMode(OPERATION_MODE_CONFIG, dev);
  delayMs(25);
  bno055RegWriteByte(BNO055_AXIS_MAP_CONFIG_ADDR, remapcode, dev);
  delayMs(10);
  /* Set the requested operating mode (see section 3.3) */
  bno055SetMode(modeback, dev);
  delayMs(20);
}

/*!
 *  @brief  Changes the chip's axis signs
 *  @param  remapsign
 *          remap sign possible values
 *          [REMAP_SIGN_P0
 *           REMAP_SIGN_P1 (default)
 *           REMAP_SIGN_P2
 *           REMAP_SIGN_P3
 *           REMAP_SIGN_P4
 *           REMAP_SIGN_P5
 *           REMAP_SIGN_P6
 *           REMAP_SIGN_P7]
 */
void bno055SetAxisSign(bno055_axis_remap_sign_t remapsign, bno055_t* dev) {
  bno055_opmode_t modeback = dev->mode;

  bno055SetMode(OPERATION_MODE_CONFIG, dev);
  delayMs(25);
  bno055RegWriteByte(BNO055_AXIS_MAP_SIGN_ADDR, remapsign, dev);
  delayMs(10);
  /* Set the requested operating mode (see section 3.3) */
  bno055SetMode(modeback, dev);
  delayMs(20);
}

/*!
 *  @brief  Use the external 32.768KHz crystal
 *  @param  usextal
 *          use external crystal boolean
 */
void bno055SetExtCrystalUse(bool usextal, bno055_t* dev) {
  bno055_opmode_t modeback = dev->mode;

  /* Switch to config mode (just in case since this is the default) */
  bno055SetMode(OPERATION_MODE_CONFIG, dev);
  delayMs(25);
  bno055RegWriteByte(BNO055_PAGE_ID_ADDR, 0, dev);
  if (usextal) {
    bno055RegWriteByte(BNO055_SYS_TRIGGER_ADDR, 0x80, dev);
  } else {
    bno055RegWriteByte(BNO055_SYS_TRIGGER_ADDR, 0x00, dev);
  }
  delayMs(10);
  /* Set the requested operating mode (see section 3.3) */
  bno055SetMode(modeback, dev);
  delayMs(20);
}

/*!
 *   @brief  Gets the latest system status info
 *   @param  system_status
 *           system status info
 *   @param  self_test_result
 *           self test result
 *   @param  system_error
 *           system error info
 */
void bno055GetSystemStatus(uint8_t *system_status, uint8_t *self_test_result, uint8_t *system_error, bno055_t* dev) {
  bno055RegWriteByte(BNO055_PAGE_ID_ADDR, 0, dev);

  /* System Status (see section 4.3.58)
     0 = Idle
     1 = System Error
     2 = Initializing Peripherals
     3 = System Iniitalization
     4 = Executing Self-Test
     5 = Sensor fusio algorithm running
     6 = System running without fusion algorithmzs
   */

  if (system_status != 0)
    *system_status = bno055RegReadByte(BNO055_SYS_STAT_ADDR, dev);

  /* Self Test Results
     1 = test passed, 0 = test failed

     Bit 0 = Accelerometer self test
     Bit 1 = Magnetometer self test
     Bit 2 = Gyroscope self test
     Bit 3 = MCU self test

     0x0F = all good!
   */

  if (self_test_result != 0)
    *self_test_result = bno055RegReadByte(BNO055_SELFTEST_RESULT_ADDR, dev);

  /* System Error (see section 4.3.59)
     0 = No error
     1 = Peripheral initialization error
     2 = System initialization error
     3 = Self test result failed
     4 = Register map value out of range
     5 = Register map address out of range
     6 = Register map write error
     7 = BNO low power mode not available for selected operat ion mode
     8 = Accelerometer power mode not available
     9 = Fusion algorithm configuration error
     A = Sensor configuration error
   */

  if (system_error != 0)
    *system_error = bno055RegReadByte(BNO055_SYS_ERR_ADDR, dev);

  delayMs(200);
}

/*!
 *  @brief  Gets the chip revision numbers
 *  @param  info
 *          revision info
 */
void bno055GetRevInfo(bno055_rev_info_t *info, bno055_t* dev) {
  uint8_t a, b;

  memset(info, 0, sizeof(bno055_rev_info_t));

  /* Check the accelerometer revision */
  info->accel_rev = bno055RegReadByte(BNO055_ACCEL_REV_ID_ADDR, dev);

  /* Check the magnetometer revision */
  info->mag_rev = bno055RegReadByte(BNO055_MAG_REV_ID_ADDR, dev);

  /* Check the gyroscope revision */
  info->gyro_rev = bno055RegReadByte(BNO055_GYRO_REV_ID_ADDR, dev);

  /* Check the SW revision */
  info->bl_rev = bno055RegReadByte(BNO055_BL_REV_ID_ADDR, dev);

  a = bno055RegReadByte(BNO055_SW_REV_ID_LSB_ADDR, dev);
  b = bno055RegReadByte(BNO055_SW_REV_ID_MSB_ADDR, dev);
  info->sw_rev = (((uint16_t)b) << 8) | ((uint16_t)a);
}

/*!
 *  @brief  Gets current calibration state.  Each value should be a uint8_t
 *          pointer and it will be set to 0 if not calibrated and 3 if
 *          fully calibrated.
 *          See section 34.3.54
 *  @param  sys
 *          Current system calibration status, depends on status of all sensors,
 * read-only
 *  @param  gyro
 *          Current calibration status of Gyroscope, read-only
 *  @param  accel
 *          Current calibration status of Accelerometer, read-only
 *  @param  mag
 *          Current calibration status of Magnetometer, read-only
 */
void bno055GetCalibration(uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag, bno055_t* dev) {
  uint8_t calData = bno055RegReadByte(BNO055_CALIB_STAT_ADDR, dev);
  if (sys != NULL) {
    *sys = (calData >> 6) & 0x03;
  }
  if (gyro != NULL) {
    *gyro = (calData >> 4) & 0x03;
  }
  if (accel != NULL) {
    *accel = (calData >> 2) & 0x03;
  }
  if (mag != NULL) {
    *mag = calData & 0x03;
  }
}

/*!
 *  @brief  Gets the temperature in degrees celsius
 *  @return temperature in degrees celsius
 */
int8_t bno055GetTemp(bno055_t* dev) {
  int8_t temp = (int8_t)(bno055RegReadByte(BNO055_TEMP_ADDR, dev));
  return temp;
}

/*!
 *  @brief   Gets a vector reading from the specified source
 *  @param   vector_type
 *           possible vector type values
 *           [VECTOR_ACCELEROMETER
 *            VECTOR_MAGNETOMETER
 *            VECTOR_GYROSCOPE
 *            VECTOR_EULER
 *            VECTOR_LINEARACCEL
 *            VECTOR_GRAVITY]
 *  @return  vector from specified source
 */
void bno055GetVector(double* xyz, bno055_vector_type_t vector_type, bno055_t* dev) {
  uint8_t buffer[6];
  memset(buffer, 0, 6);

  int16_t x, y, z;
  x = y = z = 0;

  /* Read vector data (6 bytes) */
  bno055RegReadBytes((bno055_reg_t)vector_type, buffer, 6, dev);

  x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
  y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
  z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

  /*!
   * Convert the value to an appropriate range (section 3.6.4)
   * and assign the value to the Vector type
   */
    switch (vector_type) {
        case VECTOR_MAGNETOMETER:
            /* 1uT = 16 LSB */
            xyz[0] = ((double)x) / 16.0;
            xyz[1] = ((double)y) / 16.0;
            xyz[2] = ((double)z) / 16.0;
            break;
        case VECTOR_GYROSCOPE:
            /* 1dps = 16 LSB */
            xyz[0] = ((double)x) / 16.0;
            xyz[1] = ((double)y) / 16.0;
            xyz[2] = ((double)z) / 16.0;
            break;
        case VECTOR_EULER:
            /* 1 degree = 16 LSB */
            xyz[0] = ((double)x) / 16.0;
            xyz[1] = ((double)y) / 16.0;
            xyz[2] = ((double)z) / 16.0;
            break;
        case VECTOR_ACCELEROMETER:
            /* 1m/s^2 = 100 LSB */
            xyz[0] = ((double)x) / 100.0;
            xyz[1] = ((double)y) / 100.0;
            xyz[2] = ((double)z) / 100.0;
            break;
        case VECTOR_LINEARACCEL:
            /* 1m/s^2 = 100 LSB */
            xyz[0] = ((double)x) / 100.0;
            xyz[1] = ((double)y) / 100.0;
            xyz[2] = ((double)z) / 100.0;
            break;
        case VECTOR_GRAVITY:
            /* 1m/s^2 = 100 LSB */
            xyz[0] = ((double)x) / 100.0;
            xyz[1] = ((double)y) / 100.0;
            xyz[2] = ((double)z) / 100.0;
            break;
    }
}

// /*!
//  *  @brief  Gets a quaternion reading from the specified source
//  *  @return quaternion reading
//  */
// Quaternion getQuat() {
//   uint8_t buffer[8];
//   memset(buffer, 0, 8);

//   int16_t x, y, z, w;
//   x = y = z = w = 0;

//   /* Read quat data (8 bytes) */
//   bno055RegReadBytes(BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8, dev);
//   w = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
//   x = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
//   y = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);
//   z = (((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]);

//   /*!
//    * Assign to Quaternion
//    * See
//    * https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf
//    * 3.6.5.5 Orientation (Quaternion)
//    */
//   const double scale = (1.0 / (1 << 14));
//   Quaternion quat(scale * w, scale * x, scale * y, scale * z);
//   return quat;
// }

// /*!
//  *  @brief  Provides the sensor_t data for this sensor
//  *  @param  sensor
//  *          Sensor description
//  */
// void getSensor(sensor_t *sensor) {
//   /* Clear the sensor_t object */
//   memset(sensor, 0, sizeof(sensor_t));

//   /* Insert the sensor name in the fixed length char array */
//   strncpy(sensor->name, "BNO055", sizeof(sensor->name) - 1);
//   sensor->name[sizeof(sensor->name) - 1] = 0;
//   sensor->version = 1;
//   sensor->sensor_id = _sensorID;
//   sensor->type = SENSOR_TYPE_ORIENTATION;
//   sensor->min_delay = 0;
//   sensor->max_value = 0.0F;
//   sensor->min_value = 0.0F;
//   sensor->resolution = 0.01F;
// }

// /*!
//  *  @brief  Reads the sensor and returns the data as a sensors_event_t
//  *  @param  event
//  *          Event description
//  *  @return always returns true
//  */
// bool getEvent(sensors_event_t *event) {
//   /* Clear the event */
//   memset(event, 0, sizeof(sensors_event_t));

//   event->version = sizeof(sensors_event_t);
//   event->sensor_id = _sensorID;
//   event->type = SENSOR_TYPE_ORIENTATION;
//   event->timestamp = millis();

//   /* Get a Euler angle sample for orientation */
//   imu::Vector<3> euler = getVector(VECTOR_EULER);
//   event->orientation.x = euler.x();
//   event->orientation.y = euler.y();
//   event->orientation.z = euler.z();

//   return true;
// }

// /*!
//  *  @brief  Reads the sensor and returns the data as a sensors_event_t
//  *  @param  event
//  *          Event description
//  *  @param  vec_type
//  *          specify the type of reading
//  *  @return always returns true
//  */
// bool getEvent(sensors_event_t *event,
//                                bno055_vector_type_t vec_type) {
//   /* Clear the event */
//   memset(event, 0, sizeof(sensors_event_t));

//   event->version = sizeof(sensors_event_t);
//   event->sensor_id = _sensorID;
//   event->timestamp = millis();

//   // read the data according to vec_type
//   imu::Vector<3> vec;
//   if (vec_type == VECTOR_LINEARACCEL) {
//     event->type = SENSOR_TYPE_LINEAR_ACCELERATION;
//     vec = getVector(VECTOR_LINEARACCEL);

//     event->acceleration.x = vec.x();
//     event->acceleration.y = vec.y();
//     event->acceleration.z = vec.z();
//   } else if (vec_type == VECTOR_ACCELEROMETER) {
//     event->type = SENSOR_TYPE_ACCELEROMETER;
//     vec = getVector(VECTOR_ACCELEROMETER);

//     event->acceleration.x = vec.x();
//     event->acceleration.y = vec.y();
//     event->acceleration.z = vec.z();
//   } else if (vec_type == VECTOR_GRAVITY) {
//     event->type = SENSOR_TYPE_GRAVITY;
//     vec = getVector(VECTOR_GRAVITY);

//     event->acceleration.x = vec.x();
//     event->acceleration.y = vec.y();
//     event->acceleration.z = vec.z();
//   } else if (vec_type == VECTOR_EULER) {
//     event->type = SENSOR_TYPE_ORIENTATION;
//     vec = getVector(VECTOR_EULER);

//     event->orientation.x = vec.x();
//     event->orientation.y = vec.y();
//     event->orientation.z = vec.z();
//   } else if (vec_type == VECTOR_GYROSCOPE) {
//     event->type = SENSOR_TYPE_GYROSCOPE;
//     vec = getVector(VECTOR_GYROSCOPE);

//     event->gyro.x = vec.x() * SENSORS_DPS_TO_RADS;
//     event->gyro.y = vec.y() * SENSORS_DPS_TO_RADS;
//     event->gyro.z = vec.z() * SENSORS_DPS_TO_RADS;
//   } else if (vec_type == VECTOR_MAGNETOMETER) {
//     event->type = SENSOR_TYPE_MAGNETIC_FIELD;
//     vec = getVector(VECTOR_MAGNETOMETER);

//     event->magnetic.x = vec.x();
//     event->magnetic.y = vec.y();
//     event->magnetic.z = vec.z();
//   }

//   return true;
// }

/*!
 *  @brief  Reads the sensor's offset registers into a byte array
 *  @param  calibData
 *          Calibration offset (buffer size should be 22)
 *  @return true if read is successful
 */
bool bno055GetSensorOffsetsArr(uint8_t* calibData, bno055_t* dev) {
  if (bno055IsFullyCalibrated(dev)) {
    bno055_opmode_t lastMode = dev->mode;
    bno055SetMode(OPERATION_MODE_CONFIG, dev);

    bno055RegReadBytes(ACCEL_OFFSET_X_LSB_ADDR, calibData, NUM_BNO055_OFFSET_REGISTERS, dev);

    bno055SetMode(lastMode, dev);
    return true;
  }
  return false;
}

/*!
 *  @brief  Reads the sensor's offset registers into an offset struct
 *  @param  offsets_type
 *          type of offsets
 *  @return true if read is successful
 */
bool bno055GetSensorOffsets(bno055_offsets_t* offsets_type, bno055_t* dev) {
    if (bno055IsFullyCalibrated(dev)) {
      bno055_opmode_t lastMode = dev->mode;
      bno055SetMode(OPERATION_MODE_CONFIG, dev);
      delayMs(25);

      /* Accel offset range depends on the G-range:
        +/-2g  = +/- 2000 mg
        +/-4g  = +/- 4000 mg
        +/-8g  = +/- 8000 mg
        +/-1§g = +/- 16000 mg */
      offsets_type->accel_offset_x = (bno055RegReadByte(ACCEL_OFFSET_X_MSB_ADDR, dev) << 8) |
                                    (bno055RegReadByte(ACCEL_OFFSET_X_LSB_ADDR, dev));
      offsets_type->accel_offset_y = (bno055RegReadByte(ACCEL_OFFSET_Y_MSB_ADDR, dev) << 8) |
                                    (bno055RegReadByte(ACCEL_OFFSET_Y_LSB_ADDR, dev));
      offsets_type->accel_offset_z = (bno055RegReadByte(ACCEL_OFFSET_Z_MSB_ADDR, dev) << 8) |
                                    (bno055RegReadByte(ACCEL_OFFSET_Z_LSB_ADDR, dev));

      /* Magnetometer offset range = +/- 6400 LSB where 1uT = 16 LSB */
      offsets_type->mag_offset_x =
          (bno055RegReadByte(MAG_OFFSET_X_MSB_ADDR, dev) << 8) | (bno055RegReadByte(MAG_OFFSET_X_LSB_ADDR, dev));
      offsets_type->mag_offset_y =
          (bno055RegReadByte(MAG_OFFSET_Y_MSB_ADDR, dev) << 8) | (bno055RegReadByte(MAG_OFFSET_Y_LSB_ADDR, dev));
      offsets_type->mag_offset_z =
          (bno055RegReadByte(MAG_OFFSET_Z_MSB_ADDR, dev) << 8) | (bno055RegReadByte(MAG_OFFSET_Z_LSB_ADDR, dev));

      /* Gyro offset range depends on the DPS range:
        2000 dps = +/- 32000 LSB
        1000 dps = +/- 16000 LSB
        500 dps = +/- 8000 LSB
        250 dps = +/- 4000 LSB
        125 dps = +/- 2000 LSB
        ... where 1 DPS = 16 LSB */
      offsets_type->gyro_offset_x =
          (bno055RegReadByte(GYRO_OFFSET_X_MSB_ADDR, dev) << 8) | (bno055RegReadByte(GYRO_OFFSET_X_LSB_ADDR, dev));
      offsets_type->gyro_offset_y =
          (bno055RegReadByte(GYRO_OFFSET_Y_MSB_ADDR, dev) << 8) | (bno055RegReadByte(GYRO_OFFSET_Y_LSB_ADDR, dev));
      offsets_type->gyro_offset_z =
          (bno055RegReadByte(GYRO_OFFSET_Z_MSB_ADDR, dev) << 8) | (bno055RegReadByte(GYRO_OFFSET_Z_LSB_ADDR, dev));

      /* Accelerometer radius = +/- 1000 LSB */
      offsets_type->accel_radius =
          (bno055RegReadByte(ACCEL_RADIUS_MSB_ADDR, dev) << 8) | (bno055RegReadByte(ACCEL_RADIUS_LSB_ADDR, dev));

      /* Magnetometer radius = +/- 960 LSB */
      offsets_type->mag_radius =
          (bno055RegReadByte(MAG_RADIUS_MSB_ADDR, dev) << 8) | (bno055RegReadByte(MAG_RADIUS_LSB_ADDR, dev));

      bno055SetMode(lastMode, dev);
      return true;
    }

    return false;
}

/*!
 *  @brief  Writes an array of calibration values to the sensor's offset
 *  @param  calibData
 *          calibration data
 */
void bno055SetSensorOffsetsArr(const uint8_t* calibData, bno055_t* dev) {
  bno055_opmode_t lastMode = dev->mode;
  bno055SetMode(OPERATION_MODE_CONFIG, dev);
  delayMs(25);

  /* Note: Configuration will take place only when user writes to the last
     byte of each config data pair (ex. ACCEL_OFFSET_Z_MSB_ADDR, etc.).
     Therefore the last byte must be written whenever the user wants to
     changes the configuration. */

  /* A writeLen() would make this much cleaner */
  bno055RegWriteByte(ACCEL_OFFSET_X_LSB_ADDR, calibData[0], dev);
  bno055RegWriteByte(ACCEL_OFFSET_X_MSB_ADDR, calibData[1], dev);
  bno055RegWriteByte(ACCEL_OFFSET_Y_LSB_ADDR, calibData[2], dev);
  bno055RegWriteByte(ACCEL_OFFSET_Y_MSB_ADDR, calibData[3], dev);
  bno055RegWriteByte(ACCEL_OFFSET_Z_LSB_ADDR, calibData[4], dev);
  bno055RegWriteByte(ACCEL_OFFSET_Z_MSB_ADDR, calibData[5], dev);

  bno055RegWriteByte(MAG_OFFSET_X_LSB_ADDR, calibData[6], dev);
  bno055RegWriteByte(MAG_OFFSET_X_MSB_ADDR, calibData[7], dev);
  bno055RegWriteByte(MAG_OFFSET_Y_LSB_ADDR, calibData[8], dev);
  bno055RegWriteByte(MAG_OFFSET_Y_MSB_ADDR, calibData[9], dev);
  bno055RegWriteByte(MAG_OFFSET_Z_LSB_ADDR, calibData[10], dev);
  bno055RegWriteByte(MAG_OFFSET_Z_MSB_ADDR, calibData[11], dev);

  bno055RegWriteByte(GYRO_OFFSET_X_LSB_ADDR, calibData[12], dev);
  bno055RegWriteByte(GYRO_OFFSET_X_MSB_ADDR, calibData[13], dev);
  bno055RegWriteByte(GYRO_OFFSET_Y_LSB_ADDR, calibData[14], dev);
  bno055RegWriteByte(GYRO_OFFSET_Y_MSB_ADDR, calibData[15], dev);
  bno055RegWriteByte(GYRO_OFFSET_Z_LSB_ADDR, calibData[16], dev);
  bno055RegWriteByte(GYRO_OFFSET_Z_MSB_ADDR, calibData[17], dev);

  bno055RegWriteByte(ACCEL_RADIUS_LSB_ADDR, calibData[18], dev);
  bno055RegWriteByte(ACCEL_RADIUS_MSB_ADDR, calibData[19], dev);

  bno055RegWriteByte(MAG_RADIUS_LSB_ADDR, calibData[20], dev);
  bno055RegWriteByte(MAG_RADIUS_MSB_ADDR, calibData[21], dev);

  bno055SetMode(lastMode, dev);
}

/*!
 *  @brief  Writes to the sensor's offset registers from an offset struct
 *  @param  offsets_type
 *          accel_offset_x = acceleration offset x
 *          accel_offset_y = acceleration offset y
 *          accel_offset_z = acceleration offset z
 *
 *          mag_offset_x   = magnetometer offset x
 *          mag_offset_y   = magnetometer offset y
 *          mag_offset_z   = magnetometer offset z
 *
 *          gyro_offset_x  = gyroscrope offset x
 *          gyro_offset_y  = gyroscrope offset y
 *          gyro_offset_z  = gyroscrope offset z
 */
void bno055SetSensorOffsets(const bno055_offsets_t* offsets_type, bno055_t* dev) {
  bno055_opmode_t lastMode = dev->mode;
  bno055SetMode(OPERATION_MODE_CONFIG, dev);
  delayMs(25);

  /* Note: Configuration will take place only when user writes to the last
     byte of each config data pair (ex. ACCEL_OFFSET_Z_MSB_ADDR, etc.).
     Therefore the last byte must be written whenever the user wants to
     changes the configuration. */

  bno055RegWriteByte(ACCEL_OFFSET_X_LSB_ADDR, (offsets_type->accel_offset_x) & 0x0FF, dev);
  bno055RegWriteByte(ACCEL_OFFSET_X_MSB_ADDR, (offsets_type->accel_offset_x >> 8) & 0x0FF, dev);
  bno055RegWriteByte(ACCEL_OFFSET_Y_LSB_ADDR, (offsets_type->accel_offset_y) & 0x0FF, dev);
  bno055RegWriteByte(ACCEL_OFFSET_Y_MSB_ADDR, (offsets_type->accel_offset_y >> 8) & 0x0FF, dev);
  bno055RegWriteByte(ACCEL_OFFSET_Z_LSB_ADDR, (offsets_type->accel_offset_z) & 0x0FF, dev);
  bno055RegWriteByte(ACCEL_OFFSET_Z_MSB_ADDR, (offsets_type->accel_offset_z >> 8) & 0x0FF, dev);

  bno055RegWriteByte(MAG_OFFSET_X_LSB_ADDR, (offsets_type->mag_offset_x) & 0x0FF, dev);
  bno055RegWriteByte(MAG_OFFSET_X_MSB_ADDR, (offsets_type->mag_offset_x >> 8) & 0x0FF, dev);
  bno055RegWriteByte(MAG_OFFSET_Y_LSB_ADDR, (offsets_type->mag_offset_y) & 0x0FF, dev);
  bno055RegWriteByte(MAG_OFFSET_Y_MSB_ADDR, (offsets_type->mag_offset_y >> 8) & 0x0FF, dev);
  bno055RegWriteByte(MAG_OFFSET_Z_LSB_ADDR, (offsets_type->mag_offset_z) & 0x0FF, dev);
  bno055RegWriteByte(MAG_OFFSET_Z_MSB_ADDR, (offsets_type->mag_offset_z >> 8) & 0x0FF, dev);

  bno055RegWriteByte(GYRO_OFFSET_X_LSB_ADDR, (offsets_type->gyro_offset_x) & 0x0FF, dev);
  bno055RegWriteByte(GYRO_OFFSET_X_MSB_ADDR, (offsets_type->gyro_offset_x >> 8) & 0x0FF, dev);
  bno055RegWriteByte(GYRO_OFFSET_Y_LSB_ADDR, (offsets_type->gyro_offset_y) & 0x0FF, dev);
  bno055RegWriteByte(GYRO_OFFSET_Y_MSB_ADDR, (offsets_type->gyro_offset_y >> 8) & 0x0FF, dev);
  bno055RegWriteByte(GYRO_OFFSET_Z_LSB_ADDR, (offsets_type->gyro_offset_z) & 0x0FF, dev);
  bno055RegWriteByte(GYRO_OFFSET_Z_MSB_ADDR, (offsets_type->gyro_offset_z >> 8) & 0x0FF, dev);

  bno055RegWriteByte(ACCEL_RADIUS_LSB_ADDR, (offsets_type->accel_radius) & 0x0FF, dev);
  bno055RegWriteByte(ACCEL_RADIUS_MSB_ADDR, (offsets_type->accel_radius >> 8) & 0x0FF, dev);

  bno055RegWriteByte(MAG_RADIUS_LSB_ADDR, (offsets_type->mag_radius) & 0x0FF, dev);
  bno055RegWriteByte(MAG_RADIUS_MSB_ADDR, (offsets_type->mag_radius >> 8) & 0x0FF, dev);

  bno055SetMode(lastMode, dev);
}

/*!
 *  @brief  Checks of all cal status values are set to 3 (fully calibrated)
 *  @return status of calibration
 */
bool bno055IsFullyCalibrated(bno055_t* dev) {
    uint8_t system, gyro, accel, mag;
    bno055GetCalibration(&system, &gyro, &accel, &mag, dev);

    switch (dev->mode) {
        case OPERATION_MODE_ACCONLY:
            return (accel == 3);
        case OPERATION_MODE_MAGONLY:
            return (mag == 3);
        case OPERATION_MODE_GYRONLY:
        case OPERATION_MODE_M4G: /* No magnetometer calibration required. */
            return (gyro == 3);
        case OPERATION_MODE_ACCMAG:
        case OPERATION_MODE_COMPASS:
            return (accel == 3 && mag == 3);
        case OPERATION_MODE_ACCGYRO:
        case OPERATION_MODE_IMUPLUS:
            return (accel == 3 && gyro == 3);
        case OPERATION_MODE_MAGGYRO:
            return (mag == 3 && gyro == 3);
        default:
            return (system == 3 && gyro == 3 && accel == 3 && mag == 3);
    }
}

/*!
 *  @brief  Enter Suspend mode (i.e., sleep)
 */
void bno055EnterSuspendMode(bno055_t* dev) {
  bno055_opmode_t modeback = dev->mode;

  /* Switch to config mode (just in case since this is the default) */
  bno055SetMode(OPERATION_MODE_CONFIG, dev);
  delayMs(25);
  bno055RegWriteByte(BNO055_PWR_MODE_ADDR, 0x02, dev);
  /* Set the requested operating mode (see section 3.3) */
  bno055SetMode(modeback, dev);
  delayMs(20);
}

/*!
 *  @brief  Enter Normal mode (i.e., wake)
 */
void bno055EnterNormalMode(bno055_t* dev) {
  bno055_opmode_t modeback = dev->mode;

  /* Switch to config mode (just in case since this is the default) */
  bno055SetMode(OPERATION_MODE_CONFIG, dev);
  delayMs(25);
  bno055RegWriteByte(BNO055_PWR_MODE_ADDR, 0x00, dev);
  /* Set the requested operating mode (see section 3.3) */
  bno055SetMode(modeback, dev);
  delayMs(20);
}

// /*!
//  *  @brief  Writes an 8 bit value over I2C
//  */
// bool bno055RegWriteByte(bno055_reg_t reg, byte value) {
//   uint8_t buffer[2] = {(uint8_t)reg, (uint8_t)value};
//   return i2c_dev->write(buffer, 2);
// }

void bno055RegWriteByte(uint8_t regAdd, uint8_t data, bno055_t* dev) {
    i2cWriteByte(dev->devAdd, regAdd, data);
}

uint8_t bno055RegReadByte(uint8_t regAdd, bno055_t* dev) {
    return i2cReadByte(dev->devAdd, regAdd);
}

void bno055RegReadBytes(uint8_t regAdd, uint8_t* data, uint8_t length, bno055_t* dev) {
    i2cReadBytes(dev->devAdd, regAdd, data, length);
}
