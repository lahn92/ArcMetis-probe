#include "EC_OEM.h"

void delayForMillis(unsigned long timeout)
{
  unsigned long t = millis();
  while(millis() - t <= timeout){}    // do nothing  
}

// basic function for I2C/SMbus access
bool EC_OEM::i2c_write_long(byte reg, unsigned long data) {                                     //used to write a 4 bytes to a register: i2c_write_longe(register to start at,unsigned long data )                                                                                 
  Wire.beginTransmission(device_addr);                                                          
  Wire.write(reg);                                                                              
  for (byte i = 3; i >= 0; i--) {                                                                    //with this code we write multiple bytes in reverse
    Wire.write(move_data.i2c_data[i]);
  }
  if(Wire.endTransmission() == 0)return true;
  else return false;                                                                         //end the I2C data transmission
}

bool EC_OEM::i2c_write_byte(byte reg, byte data) {                                              //used to write a single byte to a register: i2c_write_byte(register to write to, byte data)
  Wire.beginTransmission(device_addr);                                                         
  Wire.write(reg);                                                                              
  Wire.write(data);                                                                             
  if(Wire.endTransmission() == 0)return true;
  else return false;                                                                         //end the I2C data transmission
}

void EC_OEM::i2c_read(byte reg, byte number_of_bytes_to_read, unsigned long timeout) {                                           //used to read 1,2,and 4 bytes: i2c_read(starting register,number of bytes to read)
  move_data.answ = 0;
  Wire.beginTransmission(device_addr);                                                            
  Wire.write(reg);                                                                                
  if( Wire.endTransmission() == 0)
  {
    unsigned long dt = millis();
    Wire.requestFrom(device_addr, (byte)number_of_bytes_to_read);                                   //call the device and request to read X bytes
    while( millis() - dt <= timeout)
    {
      if(Wire.available() ==  (byte)number_of_bytes_to_read)
      {
          for (byte i = number_of_bytes_to_read; i > 0; i--)
            move_data.i2c_data[i - 1] = Wire.read();
          break;  
      }  
    }
    Wire.endTransmission();     
  }                                                                        
}


// main methods of lib

EC_OEM::EC_OEM(uint8_t pin, uint8_t addr_)
{
  this->int_pin               = pin;
  this->device_addr           = addr_;
  this->param.salinity        = 0;
  this->param.conductivity    = 0;
  this->param.tds             = 0;
  this->new_reading_available = false;
  this->hibernation_status    = false;
  this->firm_version          = UNKNOWN_VERSION;
  this->int_control           = 0;
  this->status_presence       = false;
  this->type_device           = UNKNOWN_DEVICE;
  this->SalinityStability.setPrecision(0.02f);
}

void EC_OEM::init(bool led_, bool hibernate_, uint8_t int_CTRL)
{
  Wire.beginTransmission(device_addr);                                                            
  Wire.write(device_type);                                                                                
  if( Wire.endTransmission() == 0 )
  {
    if(getDeviceType() == EC_OEM_DEVICE )
    {
      this->status_presence = true;
    }
    else
    {
      this->device_addr = getAddrressDevice(EC_OEM_DEVICE);
      Serial.println(device_addr);
      if ( device_addr == EC_OEM_DEVICE )
        this->status_presence = true;
      else
        this->status_presence = false;
    }     
  }
  if(status_presence)
  {
    if(led_) setLedOn(led_);
    if(hibernate_)
    {
      if(!isHibernate()) setHibernate();
    }
    else
    {
      if(isHibernate()) wakeUp();
    }
    setInterruptAvailable(int_CTRL);
    clearNewDataRegister();
  }
}

byte EC_OEM::getDeviceType(void)
{
  this->type_device = UNKNOWN_DEVICE;
  i2c_read(device_type, one_byte_read);
  this->type_device = move_data.i2c_data[0];
  return type_device;
}

byte EC_OEM::getFirmwareVersion(void)
{
  this->firm_version = UNKNOWN_VERSION;
  i2c_read(firmware_version, one_byte_read);
  this->firm_version = move_data.i2c_data[0];
  return firm_version;
}

bool EC_OEM::isLockedAddress(void)
{
  i2c_read(addr_lock, one_byte_read);
  if(move_data.i2c_data[0]>0) return true;
  else                        return false;
}

bool EC_OEM::setLockedAddress(bool lock)
{
  if(lock)    // lock address
  {
    if ( i2c_write_byte(addr_lock, 0x00))
    {
      i2c_read(addr_lock, one_byte_read);               
      if (move_data.i2c_data[0])return true;
    }
  }
  else        // unlock address
  {
    if(i2c_write_byte(addr_lock, address_unlock_A))
    {
      if( i2c_write_byte(addr_lock, address_unlock_B) )
      {
        i2c_read(addr_lock, one_byte_read);
        if (move_data.i2c_data[0] == 0) return true;
      }
    }
  }
  return false;
}

bool EC_OEM::setDeviceAddress(uint8_t new_address)
{
  if (new_address < 1 || new_address > 127)return false;
  else
  {
      i2c_read(addr_lock, one_byte_read);                                    
      if (move_data.i2c_data[0])            // addr is locked
      {
        if ( !setLockedAddress(false) ) return false;   // can't unlock
      };
      if(i2c_write_byte(new_addr_register, new_address))  return true;
      else                                                return false;
  }
}

uint8_t EC_OEM::isInterruptAvailable(void)
{
  i2c_read(int_ctrl, one_byte_read); 
  if( move_data.i2c_data[0] == HIGH_ON_INTERRUPT || move_data.i2c_data[0] == LOW_ON_INTERRUPT || move_data.i2c_data[0] == CHANGE_ON_INTERRUPT )
    return move_data.i2c_data[0];
  else 
    return DISABLED_INTERRUPT;
}

bool EC_OEM::setInterruptAvailable(uint8_t mode)
{
  if( mode == DISABLED_INTERRUPT || mode == HIGH_ON_INTERRUPT || mode == LOW_ON_INTERRUPT || mode == CHANGE_ON_INTERRUPT )
  {
    if ( isInterruptAvailable() == mode ) return true;
    else
    {
      if( i2c_write_byte(int_ctrl, mode) )
      {
        i2c_read(int_ctrl, one_byte_read);
        if( move_data.i2c_data[0] == mode ) return true;
        else                                return false;
      }
      else
        return false;
    }
  }
}

bool EC_OEM::isLedOn(void)
{
   i2c_read(led_ctrl, one_byte_read);
   if (move_data.i2c_data[0])       return on_EC;   
   if (move_data.i2c_data[0] == 0)  return off_EC;
}

bool EC_OEM::setLedOn(bool stat)
{
  byte stat_byte = stat?0x01:0x00;
  if(i2c_write_byte(led_ctrl, stat_byte))
  {
    i2c_read(led_ctrl, one_byte_read);
    if (move_data.i2c_data[0] == stat_byte)  return true;
    else                                     return false;
  }
  else
    return false;
}

bool EC_OEM::isHibernate(void)
{
  i2c_read(sleep_ctrl, one_byte_read);
  if( move_data.i2c_data[0] == 0 ) return EC_HIBERNATES;
  if( move_data.i2c_data[0] == 1 ) return EC_ACTIVES;
}

bool EC_OEM::setHibernate(void)
{
    if(i2c_write_byte(sleep_ctrl, 0x00))
    {
      i2c_read(sleep_ctrl, one_byte_read);
      if (move_data.i2c_data[0] == 0)   return true;
      else                              return false;
    }
    else
      return false;
}

bool EC_OEM::wakeUp(void)
{
  if(i2c_write_byte(sleep_ctrl, 0x01))
  {
    i2c_read(sleep_ctrl, one_byte_read);
    if (move_data.i2c_data[0] == 1) return true;
    else                            return false;
  }
  else
    return false;
}

bool EC_OEM::isNewDataAvailable(void)
{
  i2c_read(data_available, one_byte_read);
  if (move_data.i2c_data[0])  return true;
  if (move_data.i2c_data[0]=0)return false;
}

bool EC_OEM::clearNewDataRegister(void)
{
  if ( !isNewDataAvailable() ) return true;
  else
  {
    if (i2c_write_byte(data_available, 0x00))
    if ( !isNewDataAvailable() ) return true;
    else                         return false;
  }
}

float EC_OEM::getProbeType(void)
{
  float k_value = 0;
  i2c_read(probe_type, two_byte_read);                     //I2C_read(OEM register, number of bytes to read)
  k_value = move_data.two_byte_answ;                                    //move the 2 bytes read into a float
  k_value /= 100;   
  return k_value;  
}

bool  EC_OEM::setProbeType(float type)
{
    type *=100;
    move_data.answ = type;                                             
    if(i2c_write_byte(probe_type, move_data.i2c_data[1]))
    {
      if(i2c_write_byte(probe_type + 1, move_data.i2c_data[0])) return true;
      else                                                      return false;
    }
    else
      return false;
}

uint8_t EC_OEM::getStatusCalibration(void)
{
  i2c_read(calib_confirm, one_byte_read);
  if (move_data.i2c_data[0] == 0) return NO_CALIBRATION;
  if (bitRead(move_data.i2c_data[0], 0) == 1) return DRY_CALIBRATION;
  if (bitRead(move_data.i2c_data[0], 1) == 1) return SINGLE_POINT_CALIBRATION;
  if (bitRead(move_data.i2c_data[0], 2) == 1) return LOW_POINT_CALIBRATION;
  if (bitRead(move_data.i2c_data[0], 3) == 1) return HIGH_POINT_CALIBRATION;
}

bool EC_OEM::clearCalibrationData(void)
{
  if( i2c_write_byte(calib_request, 0x01) )
  {
    delayForMillis(10);    // don't use delay-function
    if(getStatusCalibration() == NO_CALIBRATION ) return true;
    else                                          return false;
  }
  else
    return false;
}

bool EC_OEM::setCalibration(uint8_t mode, float value)
{
  if (mode != DRY_CALIBRATION || mode != SINGLE_POINT_CALIBRATION || mode != LOW_POINT_CALIBRATION || mode != HIGH_POINT_CALIBRATION) return false;
  value*=100;
  switch(mode)
  {
    case DRY_CALIBRATION:
                          if( i2c_write_byte(calib_request, 0x02) )
                          {
                            delayForMillis(15); 
                            if ( getStatusCalibration() == DRY_CALIBRATION ) return true;
                            else                                             return false; 
                          }
                          break;
    case SINGLE_POINT_CALIBRATION:
                                    move_data.answ = value;                                                                   
                                    if( i2c_write_long(calib_reg, move_data.answ) )
                                    {
                                      if( i2c_write_byte(calib_request, 0x03) )
                                      {
                                        delayForMillis(100);
                                        if( getStatusCalibration() == SINGLE_POINT_CALIBRATION ) return true;
                                        else                                                     return false;                                        
                                      }
                                    }
                                    break;
    case LOW_POINT_CALIBRATION:
                                move_data.answ = value;                        
                                if (i2c_write_long(calib_reg, move_data.answ) )
                                {
                                  if( i2c_write_byte(calib_request, 0x04) )
                                  {
                                    delayForMillis(100);
                                    if ( getStatusCalibration() ==  LOW_POINT_CALIBRATION ) return true;
                                    else                                                    return false; 
                                  }
                                }
                                break;
    case HIGH_POINT_CALIBRATION:
                                  move_data.answ = value;  
                                  if( i2c_write_long(calib_reg, move_data.answ) )
                                  {
                                    if( i2c_write_byte(calib_request, 0x05) )
                                    {
                                      delayForMillis(100);
                                      if ( getStatusCalibration() == HIGH_POINT_CALIBRATION ) return true;
                                      else                                                    return false;
                                    };
                                  };
                                  break;
    default:
              return false; break; // will returned false value, break if system can't giving return value
  }
  return false;
}

float EC_OEM::getTempCompensationValue(void)
{
  float compensation = 0;
  if( isHibernate() )   // check for device status hibernation, waking up device if hibernates
  {
    if ( !wakeUp() ) return compensation;
  };
  i2c_read(temp_confirm, four_byte_read);
  compensation = move_data.answ;         
  compensation /= 100;                   
  return compensation;          
}

bool EC_OEM::setTempCompensation(float value)
{
  value*=100;
  move_data.answ = value;
  if ( i2c_write_long(temp_compen, move_data.answ) ) return true;
  else                                               return false;
}

bool EC_OEM::singleReading(void)
{
  if(isHibernate())
  {
    wakeUp();
    NOP;
    if(isHibernate()) return false;
  };
  if(isNewDataAvailable())
  {
    i2c_read(ec_read_reg, four_byte_read);                                                  
    param.conductivity = move_data.answ;                                                                    
    param.conductivity /= 100;                                                                              
    NOP;
    i2c_read(pss_read_reg, four_byte_read);
    param.salinity = move_data.answ;
    param.salinity /= 100;
    SalinityStability.pushToBuffer(param.salinity);
    NOP;
    i2c_read(tds_read_reg, four_byte_read);
    param.tds = move_data.answ;
    param.tds/=100;
    NOP;
    clearNewDataRegister();
    return true;
  }
  else
    return false;
}

float EC_OEM::getSalinity(bool fromStored)
{
   if(fromStored)
   {
      if(isHibernate())
      {
        wakeUp();
        NOP;
      }
      return param.salinity;
   }
   else
   {
      if(isHibernate())
      {
        wakeUp();
        NOP; NOP;
        if ( isHibernate() ) return sqrt(-1);
      }
      if(isNewDataAvailable())
      {
        NOP;
        i2c_read(pss_read_reg, four_byte_read);
        param.salinity = move_data.answ;
        param.salinity /= 100;
        SalinityStability.pushToBuffer(param.salinity);
        clearNewDataRegister();
      }
   }
   return param.salinity;
}

float EC_OEM::getConductivity(bool fromStored) 
{
  if(fromStored)
   {
      if(isHibernate())
      {
        wakeUp();
        NOP;
      }
      return param.salinity;
   }
   else
   {
      if(isHibernate())
      {
        wakeUp();
        NOP; NOP;
        if ( isHibernate() ) return sqrt(-1);
      }
      if(isNewDataAvailable())
      {
        NOP;
        i2c_read(ec_read_reg, four_byte_read);
        param.conductivity = move_data.answ;
        param.conductivity /= 100;
        clearNewDataRegister();
      }
   }
   return param.conductivity;
}

float EC_OEM::getTDS(bool fromStored)
{
  if(fromStored)
   {
      if(isHibernate())
      {
        wakeUp();
        NOP;
      }
      return param.salinity;
   }
   else
   {
      if(isHibernate())
      {
        wakeUp();
        NOP; NOP;
        if ( isHibernate() ) return sqrt(-1);
      }
      if(isNewDataAvailable())
      {
        NOP;
        i2c_read(tds_read_reg, four_byte_read);
        param.tds = move_data.answ;
        param.tds /= 100;
        clearNewDataRegister();
      }
   }
   return param.tds;
}

param_OEM_EC EC_OEM::getAllParam(void)
{
  if(isHibernate())
  {
    wakeUp();
    NOP;
  }
  return param;
}
