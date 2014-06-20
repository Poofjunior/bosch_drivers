/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

/*********************************************************************
 *
 * Disclaimer
 *
 * This source code is not officially released or supported by Bosch Sensortec.
 * Please contact the listed authors with bugs or other modifications.
 * If you would like the official Bosch Sensortec drivers, please contact:
 * contact@bosch-sensortec.com
 *
 *********************************************************************/
 
//\Author Joshua Vasquez and  Philip Roan, Robert Bosch LLC

#ifndef BMA280_H_
#define BMA280_H_

#include <cmath>  // for atan2
#include <unistd.h> // for the sleep function

#include <bosch_drivers_common/bosch_drivers_sensor_driver.hpp>
#include <bosch_drivers_common/bosch_drivers_hardware_interface.hpp>

using namespace bosch_drivers_common;

/**
 * \brief Driver the the BMA280 accelerometer.
 *
 * This class provides access to both the acceleration and temperature data.
 * Users can change the sensor's basic settings, such as bandwidth and range;
 * however, the user cannot change modes and cannot directly implement the
 * sensor's built-in slope-detection routines.
 */
class BMA280: public sensor_driver
{
public:
  /**
   * I2C address (1 of 2)
   * \note BMA280 uses this I2C address if SDO is connected to VSS
   */
  static const uint8_t SLAVE_ADDRESS0 = 0x18;

  /**
   * I2C address (2 of 2)
   * \note BMA280 uses this I2C address if SDO is connected to VDDIO
   */
  static const uint8_t SLAVE_ADDRESS1 = 0x19; 
  
  /**
   * \brief Write these to <3:1> in  ADDRESS_RANGE_REG to change sensitivity
   */
  enum accel_range
  {
    /** \note ±2 [g] at 0.244 [mg/LSB] */  
    RANGE_2 = 0b0011,
    /** \note ±4 [g] at 0.488 [mg/LSB] */
    RANGE_4 = 0b0101,
    /** \note ±8 [g] at 0.977 [mg/LSB] */
    RANGE_8 = 0b1000,
    /** \note ±16 [g] at 1.953 [mg/LSB] */
    RANGE_16 = 0b1100 
  };
  
  /**
   * \brief Write these to  <7:4> in ADDRESS_BWTCS to change bandwidth filter
   */
  enum bandwidth 
  {
    BW_7_81 = 0b1000,
    BW_15_63 = 0b1001,
    BW_31_25 = 0b1010,
    BW_62_5  = 0b1011,
    BW_125 = 0b1100,
    BW_250 = 0b1101,
    BW_500 = 0b1110,
    BW_UNFILTERED = 0b10000,
  };

  /**
   * \brief input arguments for selecting an axis
   */
  enum axis
  {
      X = 0x00,
      Y = 0x01,
      Z = 0x02
  };


  BMA280( bosch_hardware_interface* hw ); 

  ~BMA280();
    
  // Initialize the hardware interface so it can communicate with the bma280:
  bool initialize();
   
  // Measurement Methods
  bool takeMeasurement();
  bool getAccelData();
  double getAccelX();
  double getAccelY();
  double getAccelZ();
  double getStaticPitch(); // returns the pitch.  Measuring while moving is a bad idea, for now.
  double getStaticRoll();  
   
  double getTemperature();
  
  // Preferance Adjustments
  bool calibrate();
  bool softReset();
  
  /**
   * \brief resets the current offset values to zero
   */  
  void resetOffsets();
  /**
   * \brief save current offset values to non-volatile memory
   */  
  void saveOffsets();

  /**
   * \brief set the device address for the Arduino. This device address
   *        is the chip-select pin on the Arduino.
   */
    // TODO: verify that the device address actually IS the chip-select
  bool setDeviceAddress( uint8_t address );
  uint8_t getDeviceAddress();

  /**
   * \brief Set the sensing range in [g].
   */
  bool setAccelerationRange( accel_range measurement_range ); 

  double getSensitivity(); // returns sensitivity  
  //accel_range getAccelerationRange();

  /**
   * \brief Set the internal filter on the sensor to the specified bandwidth.
   */
  void setBandwidth( bandwidth bw );
  bool changeBandwidth();

/**
 * void setOffset(axis n, double val)
 * \brief set offset on the chip
 * \param val is the value (in [g]s) subtracted from the raw sensor value
 * \details this offset is written directly to the chip but deleted unless it
 *          is first stored to NVM
 */
  void setOffset( axis n, double val);
  
  bool setFrequency( unsigned int frequency );
  bool setProtocol( interface_protocol protocol_name );
  bool setParameters( bosch_drivers_communication_properties properties );



  
protected:
  uint8_t slave_address_bit_;
  double sensitivity_;
  accel_range accel_range_;
  bandwidth bandwidth_;
  bool useFilter_;
  bool offsetsEnabled_;

  double AccelX_;
  double AccelY_;
  double AccelZ_;
  double Temperature_;
  double StaticPitch_;
  double StaticRoll_;  
  
  double TempSlope_;


  // BMA280 Register Definitions

  // The Acceleration Data Registers
  static const uint8_t ADDRESS_ACCLXYZ     = 0x02;
  static const uint8_t ADDRESS_ACCLX_MSB   = 0x03;
  static const uint8_t ADDRESS_ACCLY_LSB   = 0x04;
  static const uint8_t ADDRESS_ACCLY_MSB   = 0x05;
  static const uint8_t ADDRESS_ACCLZ_LSB   = 0x06;
  static const uint8_t ADDRESS_ACCLZ_MSB   = 0x07;

/**
 * \brief temperature in 2's complement. A temp of 8'b0 
 *        corresponds to 0 deg C.
 */
  static const uint8_t ADDRESS_TEMPERATURE = 0x08; 

  static const uint8_t ADDRESS_VER         = 0x00;  
  static const uint8_t ADDRESS_STATUS_REG1 = 0x09;
  static const uint8_t ADDRESS_STATUS_REG2 = 0x0A;
  static const uint8_t ADDRESS_STATUS_REG3 = 0x0B;
  static const uint8_t ADDRESS_STATUS_REG4 = 0x0C;

  static const uint8_t ADDRESS_RANGE = 0x0F;


  /// Calibration addresses NOT in EEPROM. Deleted every power cycle.
  static const uint8_t ADDRESS_OFFSET_Z    = 0x3A;
  static const uint8_t ADDRESS_OFFSET_Y    = 0x39;
  static const uint8_t ADDRESS_OFFSET_X    = 0x38;
  static const uint8_t ADDRESS_OFFSET_TARGET    = 0x37;
  static const uint8_t ADDRESS_OFFSET_RESET = 0x36;
  static const uint8_t ADDRESS_NVM         = 0x33;

  /// Soft-Reset
  static const uint8_t ADDRESS_SOFTRESET = 0x14;
  static const uint8_t CMD_SOFTRESET     = 0xB6;    

  /// (relevant) BITFLAGS for changing settings:

  // Bitflags for ADDRESS_OFFSET_RESET
  static const uint8_t hp_x_en_      = 0;
  static const uint8_t hp_y_en_      = 1;
  static const uint8_t hp_z_en_      = 2;
  static const uint8_t cal_rdy_      = 4;
  static const uint8_t cal_trigger_  = 5;
  static const uint8_t offset_reset_ = 7;

  // BitFlag Offsets: NVM 
  static const uint8_t trim_nvm_ctrl_ = 4; // 4 bits of data at bits  < 7:4 > 
  static const uint8_t nvm_prog_mode_ = 0; // 1 bit of data at bit  < 0 > 
  static const uint8_t nvm_prog_trig_ = 1; // 1 bit of data at bit  < 1 > 
  static const uint8_t nvm_ready_     = 2; // 1 bit of data at bit  < 2 > 
  static const uint8_t nvm_load_      = 3; // 1 bit of data at bit  < 3 > 

  // BitFlags: ADDRESS_BW_TCS
  // Changing Bandwidth
  static const uint8_t bw = 4;
 
  // BitFlags: ADDRESS_STATUS_REG1
  // Check if calibration has been completed:
  static const uint8_t offset_st_s = 1;
 
  // BitFlags: ADDRESS_CTRL_REG4
  // Begin a calibration routine:
  static const uint8_t offset_finetuning = 0; // 2 bits of data: < 1:0 >


  bool readReg( uint8_t reg, uint8_t* value );             
  bool writeToReg( uint8_t reg, uint8_t value );
  bool writeToRegAndVerify( uint8_t reg, uint8_t value, uint8_t expected_value );
  bool readSensorData( uint8_t reg, uint8_t* value, uint8_t num_bytes );  


  /**
   * \brief Set the SPI mode (0 -- 4).
   */
  bool setSpiMode( uint8_t mode );
};

#endif // BMA280_H_
