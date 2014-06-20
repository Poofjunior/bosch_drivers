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

//\Author Joshua Vasquez and Philip Roan, Robert Bosch LLC

// For calibration routine
#include <iostream> 

// ROS headers for debugging output
#include <ros/console.h>
#include "bma280_driver/bma280.hpp"


/**********************************************************************/
// Constructor
/**********************************************************************/
BMA280::BMA280( bosch_hardware_interface* hw ) :
  sensor_driver( hw, EXTERNAL_DEVICE ),
  TempSlope_( 0.5 ),
  accel_range_( RANGE_2),
  sensitivity_( 0.00025 ),
  bandwidth_( BW_150 ),
  slave_address_bit_( 0 )
{
  communication_properties_->device_address = SLAVE_ADDRESS0;
  communication_properties_->protocol = I2C;
  communication_properties_->frequency = 400000;
  //byte_order( MSB_FIRST ),
  //spi_mode( SPI_MODE_3 )
  //communication_properties_->flags = ;

}


/**********************************************************************/
// Destructor
/**********************************************************************/
BMA280::~BMA280()
{
}


/**********************************************************************/
/**********************************************************************/
bool BMA280::setDeviceAddress( uint8_t address )
{
  communication_properties_->device_address = address;
  return true;
}


/**********************************************************************/
/**********************************************************************/
uint8_t BMA280::getDeviceAddress()
{
  // depends on the protocol:
  switch( communication_properties_->protocol )
  {
  case I2C:
    // depends on hardware configuration:
    if( slave_address_bit_ == 0 )
      return SLAVE_ADDRESS0; 
    else
      if( slave_address_bit_ == 1 )
        return SLAVE_ADDRESS1;
      else   
        ROS_ERROR( "BMA280::getDeviceAddress(): invalid I2C address" );
    break;
  case SPI:
    return communication_properties_->device_address;
  default:
    ROS_ERROR( "BMA280::getDeviceAddress(): sensor has no identification. Either setPin(uint8_t pin) for SPI or setSlaveAddress( 0 or 1) for I2C." );
    return 255;
  }
  ROS_ERROR( "BMA280::getDeviceAddress(): sensor has no identification. Either setPin(uint8_t pin) for SPI or setSlaveAddress( 0 or 1) for I2C." );
  return 255;
}


/**********************************************************************/
/**********************************************************************/
bool BMA280::setParameters( bosch_drivers_communication_properties properties )
{
  *communication_properties_ = properties;
  return true;
}


/**********************************************************************/
/**********************************************************************/
bool BMA280::setFrequency( unsigned int frequency )
{
  communication_properties_->frequency = frequency;
  return true;
}

/**********************************************************************/
// Initialize sensor and hardware interface on user-requested properties
/**********************************************************************/
bool BMA280::initialize()
{
  ROS_INFO( " " );
  ROS_INFO( "BMA280::initialize(): Device Address (hex): %x",getDeviceAddress() );
  ROS_INFO( "BMA280::initialize(): Protocol:             %d",getProtocol() );
  ROS_INFO( "BMA280::initialize(): Frequency:            %d",getFrequency() );
 
  // Initialize the hardware interface
  if( hardware_->initialize() == false )
    return false;

  // Reset the sensor:
  if( softReset() == false )
    return false;
  ROS_INFO( "BMA280::initialize(): soft reset applied." );

// Disable I2C, if sensor is setup for SPI mode:
  switch( getProtocol() )
  {
  case I2C:
    break;
  case SPI:
    // disable i2c to prevent accidental malfunctions:
    if( DisableI2C() == false )
      return false;
    ROS_INFO( "BMA280::initialize(): Disabled I2C mode." );
    break;
  default: // shouldn't happen. User should know to select either SPI or I2C.
    ROS_ERROR( "BMA280::initialize(): BMA280 cannot be read with selected protocol." );
    return false;
  }
  
  // Enable EEPROM and Register Writing:
  if( EnableWriting() == false )
    return false;
  
  // Change accel_range on the sensor to match requested parameter range:
  if( setAccelerationRange( accel_range_ ) == false )
    return false;

  // Change bandwidth_ on the sensor to match requested parameter bandwidth:
  if( changeBandwidth() == false )
    return false;

  ROS_INFO( "BMA280 initialized." );
  sleep( .1 );

  return true;
}


/**********************************************************************/
/**********************************************************************/
bool BMA280::takeMeasurement()
{
  uint8_t Data[7];
  if( readSensorData( ADDRESS_ACCLXYZ, Data, 7 ) == false )
  {
    ROS_ERROR(" BMA280::takeMeasurement(): Unable to read accelerometer data from sensor." );
    return false;
  }
  // TODO: Change typecasts to functional notation!
  AccelX_ = sensitivity_ * int( ( ((int16_t)(Data[1] << 8)) | 
                                ( ((int16_t)Data[0]) ) ) >> 2 ); // [g]
  AccelY_ = sensitivity_ * int( ( ((int16_t)(Data[3] << 8)) | 
                                ( ((int16_t)Data[2]) ) ) >> 2 ); // [g]
  AccelZ_ = sensitivity_ * int( ( ((int16_t)(Data[5] << 8)) | 
                                ( ((int16_t)Data[4]) ) ) >> 2 ); // [g]
  Temperature_ = ( TempSlope_* (int8_t)Data[6] ) + 23; // [C]
  return true;
}

/**********************************************************************/
/**********************************************************************/
double BMA280::getStaticPitch()
{
  // returns the pitch based on the most recent measurements
  StaticPitch_ = atan2( AccelY_, AccelZ_ ) ;
  return StaticPitch_;
}

/**********************************************************************/
/**********************************************************************/
double BMA280::getStaticRoll()
{
  StaticRoll_ = atan2( AccelX_, AccelZ_ );
  return ( StaticRoll_ );
}


/**********************************************************************/

/**********************************************************************/
bool BMA280::getAccelData()
{
  uint8_t Data[6];
  if( readSensorData( ADDRESS_ACCLXYZ, Data, 6 ) == false )
  {
    ROS_ERROR( "BMA280::getAccelData(): Unable to read accelerometer data from sensor." );
    return false;
  }
  AccelX_ = getSensitivity() * int( ( ((int16_t)(Data[1] << 8)) | 
                                    ( ((int16_t)Data[0]) ) ) >> 2 ); // [g]
  AccelY_ = getSensitivity() * int( ( ((int16_t)(Data[3] << 8)) | 
                                    ( ((int16_t)Data[2]) ) ) >> 2 ); // [g]
  AccelZ_ = getSensitivity() * int( ( ((int16_t)(Data[5] << 8)) | 
                                    ( ((int16_t)Data[4]) ) ) >> 2 ); // [g]
  return true;
}


/**********************************************************************/

/**********************************************************************/
double BMA280::getAccelX()  
{
  uint8_t Data[2];
   
  // Must read LSB first.  MSB and LSB  must be read in one transaction:
  if( readSensorData( ADDRESS_ACCLXYZ, Data, 2 ) == false )
  {
    ROS_ERROR("BMA280: cannot read from this protocol.");
    return -9999.9;
  }
  int raw_data = int(( ((int16_t)(Data[1] << 8)) | 
                     ( ((int16_t)Data[0]) ) ) >> 2);
  AccelX_ = raw_data * getSensitivity();

  return AccelX_;
}



/**********************************************************************/

/**********************************************************************/
double BMA280::getAccelY()
{
  uint8_t Data[2];
 
  if( readSensorData( ADDRESS_ACCLY_LSB, Data, 2 ) == false )
  {
    ROS_ERROR("BMA280::getAccelY(): failed.");
    return -9999.9;
  }
  double raw_data = (( ((int16_t)(Data[1] << 8)) | 
                      ( (int16_t)(Data[0]) ) ) >> 2);
  AccelY_ = raw_data * getSensitivity();
  
  return AccelY_;
}


/**********************************************************************/

/**********************************************************************/
double BMA280::getAccelZ()
{
  uint8_t Data[2];
  if( readSensorData( ADDRESS_ACCLZ_LSB, Data, 2 ) == false ) 
  {
    ROS_ERROR( "BMA280: cannot read from this protocol." );
    return -9999.9;
  }
  
  double raw_data = (( ((int16_t)(Data[1] << 8)) | 
                      ( (int16_t)(Data[0]) ) ) >> 2);
  AccelZ_ = raw_data * getSensitivity();
    
  return AccelZ_;
}


/**********************************************************************/

/**********************************************************************/
double BMA280::getTemperature()
{ 
  uint8_t Temperature;

  if( readReg( ADDRESS_TEMPERATURE, &Temperature ) == false )
  {
    ROS_ERROR("BMA280::getTemperature(): failed.");
    return -9999.9;
  }
  // convert raw temperature to actual temperature:
  Temperature_ = ( TempSlope_* (int8_t)Temperature ) + 23;   
  return Temperature_;
}


/**********************************************************************/

/**********************************************************************/
bool BMA280::softReset()
{
// not sure why the 5th argument has to be set in this way:
  uint8_t Request_SoftReset = CMD_SOFTRESET;
 
  // write 0xB6 to 0x14
  if( writeToReg( ADDRESS_SOFTRESET, Request_SoftReset ) == false )
  {
    ROS_ERROR("BMA280::softReset(): write failed.");
    return false;
  }      

  return true;
}


/**********************************************************************/
// perform a fine-calibration ---  see datahseet page 44.
/**********************************************************************/
//TODO: upgrade to BMA280
bool BMA280::calibrate()
{
  return true;
}


/**********************************************************************/

/**********************************************************************/
bool BMA280::setAccelerationRange(accel_range measurement_range )
{
  uint8_t local_range;
 
  // read current accel range register value for a local copy.
  if( readReg( ADDRESS_OFFSET_LSB1, &local_range ) == false )
  {
    ROS_ERROR("bma280_driver: setAccelerationRange() failed to read current range.");
    return false;
  }
 
  ROS_DEBUG( "bma280_driver: Acceleration range bits before changing: %d.  Default:  %d", ( (local_range & (0x07 << range)) >> range), 2); // Defaults on page 27 of datasheet

  // add our command to change range:
  local_range &= ~(0x07 << range); // clear old range value. Mask: b11110001
  local_range |= (accel_range_ << range); // insert new range value.

  // write the adjusted register value back to the sensor's register:
  if( this->writeToReg( ADDRESS_OFFSET_LSB1, local_range ) == false )
  {
    ROS_ERROR( "bma280_driver: setAccelerationRange() failed to write new range to sensor." );
    return false;
  }
 
  // read back that register to make sure that changes worked:
  if( this->readReg( ADDRESS_OFFSET_LSB1, &local_range ) == false )
  {
    ROS_ERROR( "bma280_driver: setAccelerationRange() failed to read new range from sensor." );
    return false;
  }
 
  // Compare register values to what we expect:
  uint8_t range_actual = ( local_range & (0x07 << range) ) >> range; // mask: b00001110
  uint8_t range_expected = (uint8_t) accel_range_; // This is the value set in the properties. 
 
  ROS_DEBUG( "bma280_driver: Acceleration range bits after change:  %d.  Expected: %d", range_actual, range_expected );
 
  if( range_expected != range_actual )
  {
    ROS_ERROR( "bma280_driver: setAccelerationRange() failed verification step." );
    return false;
  }


  // After changing the sensor, change sensitivity
  switch( measurement_range )
  {
  case RANGE_1:
    sensitivity_ = 0.00013;
    break;
  case RANGE_1_5:
    sensitivity_ = 0.00019;
    break;
  case RANGE_2:
    sensitivity_ = 0.00025;
    break;
  case RANGE_3:
    sensitivity_ = 0.00038;
    break;
  case RANGE_4:
    sensitivity_ = 0.00050;
    break;
  case RANGE_8:
    sensitivity_ = 0.00099;
    break;
  case RANGE_16:
    sensitivity_ = 0.00198;
    break;
  default: // shouldn't happen because input argument is only an accel_range data type.
    ROS_ERROR( "bma280_properties: invalid range setting." );
    return false;
  }

 
  // if new settings are correct:
  return true;
}


/**********************************************************************/
/**********************************************************************/
bool BMA280::changeBandwidth()
{
  uint8_t local_bw_reg;
  // read register with the current bandwidth flags for a local copy.
  if( this->readReg( ADDRESS_BW_TCS, &local_bw_reg ) == false )
  {
    ROS_ERROR( "BMA280::changeBandwidth(): read failed." );
    return false;
  }
 
  // #ifdef DEBUG
  ROS_INFO( "Bandwidth bits before: %d.  Default:  %d", ( (local_bw_reg & (0x0F << bw)) >> bw), 4 ); // defaults on page 27
  // #endif 
 
  // add our command to change range:
  local_bw_reg &= ~(0x0F << bw); // clear old value. Mask: b00001111
  local_bw_reg |= bandwidth_ << bw; // insert new value.

  // write the adjusted register value back to the sensor's register
  if( writeToReg( ADDRESS_BW_TCS, local_bw_reg ) == false )
  {
    ROS_ERROR( "BMA280::changeBandwidth(): write failed." );
    return false;
  }
 
  // read back that register to make sure that changes worked:
  if( readReg( ADDRESS_BW_TCS, &local_bw_reg ) == false )
  {
    ROS_ERROR( "BMA280::changeBandwidth(): read failed." );
    return false;
  }
 
  // Compare register values to what we expect:
  uint8_t bandwidth_actual = ( local_bw_reg & (0x0F << bw) ) >> bw; // mask: b11110000
  uint8_t bandwidth_expected = (uint8_t) bandwidth_; // This is the value set in the properties. 
 
  // #ifdef DEBUG
  ROS_INFO("Bandwidth bits after:  %d.  Expected: %d", bandwidth_actual, bandwidth_expected);
  // #endif 
 
  if( bandwidth_expected != bandwidth_actual )
  {
    ROS_ERROR( "BMA280::changeBandwidth(): failed." );
    return false;
  }
 
  // if new settings are correct:
  return true;
}


/**********************************************************************/
/**********************************************************************/
bool BMA280::setOffset( axis n, double val)
{
    int8_t bitsPerG = (255 / 2);   /// (max byte range) / (max g range)
    int8_t offsetRegVal = bitsPerG * val; 

    /// Note: Offsets are stored sequentially in memory.
    return writeToReg((ADDRESS_OFFSET_X + n), offsetRegVal);
}


/**********************************************************************/
// reset offset values to zero. 
/**********************************************************************/
void BMA280::resetOffsets()
{
  uint8_t offset_rst_reg;
  readReg(ADDRESS_OFFSET_RESET, &offset_rst_reg);
  offset_rst_reg |= (1 << offset_reset_);
  writeToReg(ADDRESS_NVM, &nvm_reg_data);
}

/**********************************************************************/
// save offset values to non-volatile memory
/**********************************************************************/
void BMA280::saveOffsets()
{
  // Datasheet suggests two separate writes to ADDRESS_NVM.
  uint8_t nvm_reg_data;
  readReg(ADDRESS_NVM, &nvm_reg_data);
  nvm_reg_data |= (1 << nvm_prog_mode_);
  writeToReg(ADDRESS_NVM, &nvm_reg_data);

  readReg(ADDRESS_NVM, &nvm_reg_data);
  nvm_reg_data |= (1 << nvm_prog_trig_);
  writeToReg(ADDRESS_NVM, &nvm_reg_data);
}

/**********************************************************************/
// read a register and return its value. 
/**********************************************************************/
bool BMA280::readReg( uint8_t reg, uint8_t* value )
{
  std::vector<uint8_t> data(1);

  // Reading depends on the protocol.
  switch( getProtocol() )
  {
  case I2C:
    if( hardware_->read( *communication_properties_, reg, data ) < 0 ) 
    {
      ROS_ERROR( "bma280_driver: Error reading register via I2C!" );
      return false;
    } 
    break;
  case SPI:
    // we must prepend the SPI_READ_FLAG.
    if( hardware_->read( *communication_properties_, ( 1 << SPI_READ_FLAG ) | reg, data ) < 0 ) 
    {
      ROS_ERROR( "bma280_driver: Error reading register via SPI!" );
      return false;
    } 
    break;
  default:
    // shouldn't happen:
    ROS_ERROR( "bma280_driver:readReg(...): invalid protocol." );
    return false;
  }

  *value = data[0];
  return true;
}

/**********************************************************************/
// writes a byte to a register.
/**********************************************************************/
bool BMA280::writeToReg( uint8_t reg, uint8_t value )
{
  std::vector<uint8_t> data(1,value);

  /// Technically, writing depends on the protocol.
  switch( getProtocol() )
  {
  case I2C:
    if( hardware_->write( *communication_properties_, reg, data ) < 0 )
    {
      ROS_ERROR( "bma280_driver: Error writing to register via I2C!" );
      return false;
    } 
    break;
  case SPI:
    /// We must prepend the SPI_WRITE_FLAG, although, technically it's already 
    /// there, since it's zero.
    if( hardware_->write( *communication_properties_, (~(1 << SPI_WRITE_FLAG)&reg), data) < 0 ) 
    {
      ROS_ERROR( "bma280_driver: Error writing to register via SPI!" );
      return false;
    } 
    break;
  default:
    ROS_ERROR("bma280_driver: invalid protocol.");
    return false;
  }
  return true;
}


/**********************************************************************/
// writes a byte to a register.
/**********************************************************************/
bool BMA280::writeToRegAndVerify( uint8_t reg, uint8_t value, uint8_t expected )
{
  uint8_t actual;
 
  if( this->writeToReg( reg, value ) == false )
    return false;
  // read it back to make sure it worked.
  if( this->readReg( reg, &actual ) == false )
    return false;

  if( expected != actual )
  {
    ROS_ERROR( "BMA280::writeToRegAndVerify(...): failed." );
    ROS_ERROR( "(in Hex) expected:  %x  actual: %x", expected, actual );
    return false;
  }

  return true;
}


bool BMA280::readSensorData( uint8_t reg, uint8_t* sensor_data, uint8_t num_bytes )
{
  std::vector<uint8_t> data(num_bytes);
  // Reading depends on the protocol.
  switch( this->getProtocol() )
  {
  case I2C:
    if( hardware_->read( *communication_properties_, reg, data ) < 0 ) 
    {
      ROS_ERROR( "bma280_driver: Error reading register via I2C!" );
      return false;
    } 
    break;
  case SPI:
    // we must prepend the SPI_READ_FLAG.
    if( hardware_->read( *communication_properties_, ((1 << SPI_READ_FLAG)|reg), data ) < 0 ) 
    {
      ROS_ERROR( "bma280_driver: Error reading register via SPI!" );
      return false;
    } 
    break;
  default:
    // shouldn't happen:
    ROS_ERROR( "bma280_driver: invalid protocol." );
    return false;
  }

  sensor_data = &data[0];
  return true; 
}


/**********************************************************************/
/**********************************************************************/
bool BMA280::setProtocol( interface_protocol protocol )
{
  switch( protocol )
  {
  case I2C:
  case SPI:
    communication_properties_->protocol = protocol;
    break;
  default:
    ROS_ERROR( "bma280_properties:Unsupported protocol." );
    return false;
  }
  return true;
}


/**********************************************************************/
/**********************************************************************/
bool BMA280::setSpiMode( uint8_t mode )
{
  // adjust the flags
  communication_properties_->flags = ( (0xFC & communication_properties_->flags) | (mode) ); // 111111xx, where xx is the mode.
 
  switch( mode )
  {
  case SPI_MODE_3:
    return true;
  case SPI_MODE_0:
  case SPI_MODE_1:
  case SPI_MODE_2:
  default:
    ROS_ERROR( "bma280_driver: BMA280 can only be read in SPI_MODE_3." );
    return false;
  }
}


/**********************************************************************/
/**********************************************************************/
void BMA280::setBandwidth( bandwidth bw )
{ 
  bandwidth_ = bw;
}


/**********************************************************************/
/**********************************************************************/
double BMA280::getSensitivity()
{
  return sensitivity_;
}


/**********************************************************************/
/**********************************************************************/
void BMA280::setPreCalOffsets( bool choice )
{
  offsetsEnabled_ = choice;
}
