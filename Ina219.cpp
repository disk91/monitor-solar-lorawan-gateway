/******************************************************************************
* TI INA219 hi-side i2c current/power monitor Library
*
* http://www.ti.com/product/ina219
*
* 6 May 2012 by John De Cristofaro
*
*
* Tested at standard i2c 100kbps signaling rate.
*
* This library does not handle triggered conversion modes. It uses the INA219
* in continuous conversion mode. All reads are from continous conversions.
*
* A note about the gain (PGA) setting:
*  The gain of the ADC pre-amplifier is programmable in the INA219, and can
* be set between 1/8x (default) and unity. This allows a shunt voltage 
* range of +/-320mV to +/-40mV respectively. Something to keep in mind,
* however, is that this change in gain DOES NOT affect the resolution
* of the ADC, which is fixed at 1uV. What it does do is increase noise
* immunity by exploiting the integrative nature of the delta-sigma ADC.
* For the best possible reading, you should set the gain to the range
* of voltages that you expect to see in your particular circuit. See
* page 15 in the datasheet for more info about the PGA.
*
* Known bugs:
*     * may return unreliable values if not connected to a bus or at
* bus currents below 10uA.
*
* Arduino 1.0 compatible as of 6/6/2012
*
* Dependencies:
*    * Arduino Wire library
*
* MIT license
******************************************************************************/

#include "Ina219.h"

namespace{
// config. register bit labels
const uint8_t RST = 15;
const uint8_t BRNG = 13;
const uint8_t PG_1 = 12;
const uint8_t PG_0 = 11;
const uint8_t BADC4 = 10;
const uint8_t BADC3 = 9;
const uint8_t BADC2 = 8;
const uint8_t BADC1 = 7;
const uint8_t SADC4 = 6;
const uint8_t SADC3 = 5;
const uint8_t SADC2 = 4;
const uint8_t SADC1 = 3;
const uint8_t MODE3 = 2;
const uint8_t MODE2 = 1;
const uint8_t MODE1 = 0;
};


INA219b::INA219b() {
}


void INA219b::getData( int16_t * mVolt, int16_t * mAmpere, int32_t * mWatt ) {

  // configure
  uint16_t config = 0;
  config |= (RANGE_32V << BRNG | GAIN_8_320MV << PG_0 | ADC_12BIT << BADC1 | ADC_12BIT << SADC1 | CONT_SH_BUS);
  write16(CONFIG_R, config);

  // calibrate - precalculate value for 0.05 ohm, 320mV, 32V, 6.4A max
  // Cal value : 4096
  // Current_Lsb = 2/10_000 => 2 / 10 for mA unit
  // Power_lsb = 40/10_000 => 40 / 10 for mW  => *4
  write16(CAL_R, 4096);

  delay(20);

  // voltage
  *mVolt = read16(V_BUS_R);
  *mVolt >>= 1;               // ( >> 3 then * 4 (<< 2) / 1000 pour des volts) 
  *mVolt &= 0xFFFC;

  // current - raw * Current_Lsb -1 overflow at 16A
  *mAmpere = read16(I_SHUNT_R);
  *mAmpere <<= 1;
  *mAmpere /= 10;

  // Power - raw * power_lsb
  int16_t mw = read16(P_BUS_R);
  *mWatt = mw;
  *mWatt <<= 2;

  config |= (RANGE_32V << BRNG | GAIN_8_320MV << PG_0 | ADC_12BIT << BADC1 | ADC_12BIT << SADC1 | PWR_DOWN);
  write16(CONFIG_R, config);
  
  return ; 
}

/*
void INA219b::calibrate(float shunt_val, float v_shunt_max, float v_bus_max, float i_max_expected) {
    uint16_t digits,cal;
    float min_lsb, swap, r_shunt,current_lsb,power_lsb;

    r_shunt = shunt_val;

    min_lsb = i_max_expected / 32767;

    current_lsb = min_lsb;
    digits=0;

    // From datasheet: This value was selected to be a round number near the Minimum_LSB.
    // This selection allows for good resolution with a rounded LSB.
    // eg. 0.000610 -> 0.000700
    //
    while( current_lsb > 0.0 ){//If zero there is something weird...
        if( (uint16_t)current_lsb / 1){
          current_lsb = (uint16_t) current_lsb + 1;
          current_lsb /= pow(10,digits);
          break;
        }
        else{
          digits++;
            current_lsb *= 10.0;
        }
    };

    swap = (0.04096)/(current_lsb*r_shunt);
    cal = (uint16_t)swap;
    power_lsb = current_lsb * 20;
    write16(CAL_R, cal);

   Serial.print("cal : ");Serial.println(cal);
   Serial.print("curent_lsb : ");Serial.println(100000.0*current_lsb);
   Serial.print("power_lsb : ");Serial.println(100000.0*power_lsb);

    
}

*/
/*

int16_t INA219b::shuntVoltageRaw() const {
  return read16(V_SHUNT_R);
}

float INA219b::shuntVoltage() const {
  float temp;
  temp = read16(V_SHUNT_R);
  return (temp / 100000);
}

int16_t INA219b::busVoltageRaw() {
  _bus_voltage_register = read16(V_BUS_R);
  _overflow = bitRead(_bus_voltage_register, OVF_B);     // overflow bit
  _ready    = bitRead(_bus_voltage_register, CNVR_B);    // ready bit
  return _bus_voltage_register;
}


float INA219b::busVoltage() {
  int16_t temp;
  temp = busVoltageRaw();
  temp >>= 3;
  return (temp * 0.004);
}

int16_t INA219b::shuntCurrentRaw() const {
  return (read16(I_SHUNT_R));
}

float INA219b::shuntCurrent() const {
  return (read16(I_SHUNT_R) * current_lsb);
}

float INA219b::busPower() const {
  return (read16(P_BUS_R) * power_lsb);
}
*/

/**************************************************************************/
/*! 
    @brief  returns conversion ready bit from last bus voltage read
    
    @note page 30:
          Although the data from the last conversion can be read at any time,
          the INA219 Conversion Ready bit (CNVR) indicates when data from
          a conversion is available in the data output registers.
          The CNVR bit is set after all conversions, averaging, 
          and multiplications are complete.
          CNVR will clear under the following conditions:
          1.) Writing a new mode into the Operating Mode bits in the 
              Configuration Register (except for Power-Down or Disable)
          2.) Reading the Power Register
          
          page 15:
          The Conversion Ready bit clears under these
          conditions:
          1. Writing to the Configuration Register, except
          when configuring the MODE bits for Power Down
          or ADC off (Disable) modes;
          2. Reading the Status Register;
          3. Triggering a single-shot conversion with the
          Convert pin.
*/
/**************************************************************************/
/*
bool INA219b::ready() const {
  return _ready;
}
*/
/**************************************************************************/
/*! 
    @brief  returns overflow bite from last bus voltage read
    
    @note The Math Overflow Flag (OVF) is set when the Power or
          Current calculations are out of range. It indicates that
          current and power data may be meaningless.
*/
/**************************************************************************/
/*
bool INA219b::overflow() const {
  return _overflow;
}
*/

/**********************************************************************
*             INTERNAL I2C FUNCTIONS                  *
**********************************************************************/

void INA219b::write16(t_reg a, uint16_t d) const {
  uint8_t temp;
  temp = (uint8_t)d;
  d >>= 8;
  Wire.beginTransmission(I2C_ADDR_40); // start transmission to device
  Wire.write(a); // sends register address to read from
  Wire.write((uint8_t)d);  // write data hibyte 
  Wire.write(temp); // write data lobyte;
  Wire.endTransmission(); // end transmission
  delay(1);
}

int16_t INA219b::read16(t_reg a) const {
  uint16_t ret;
  // move the pointer to reg. of interest, null argument
  write16(a, 0);
  
  Wire.requestFrom((int)I2C_ADDR_40, 2);    // request 2 data bytes
  ret = Wire.read(); // rx hi byte
  ret <<= 8;
  ret |= Wire.read(); // rx lo byte
  return ret;
}
