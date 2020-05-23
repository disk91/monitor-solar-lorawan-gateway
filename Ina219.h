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
*    The gain of the ADC pre-amplifier is programmable in the INA219, and can
*    be set between 1/8x (default) and unity. This allows a shunt voltage
*    range of +/-320mV to +/-40mV respectively. Something to keep in mind,
*    however, is that this change in gain DOES NOT affect the resolution
*    of the ADC, which is fixed at 1uV. What it does do is increase noise
*    immunity by exploiting the integrative nature of the delta-sigma ADC.
*    For the best possible reading, you should set the gain to the range
*    of voltages that you expect to see in your particular circuit. See
*    page 15 in the datasheet for more info about the PGA.
*
* Known bugs:
*     * may return unreliable values if not connected to a bus or at
*    bus currents below 10uA.
*
* Arduino 1.0 compatible as of 6/6/2012
*
* Dependencies:
*    * Arduino Wire library
*
* MIT license
******************************************************************************/

#ifndef ina219b_h
#define ina219b_h


#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>

#define D_SHUNT            500      // mOhn
#define D_V_BUS_MAX        32       // Volt
#define D_V_SHUNT_MAX      320      // mVolt
#define D_I_MAX_EXPECTED   6400     // mA      

class INA219b
{
  public:

    ///I2C address definations.
    /// Adafruit breakout board have two jumppers to set unique
    /// address for each board ( in case there is more than one in Your system)
    /// base address is 0x40, no jumpers needed)
    enum t_i2caddr{
      I2C_ADDR_40 = 0x40, ///< address 0x40 no jumpers required.
      I2C_ADDR_41 = 0x41, ///< address 0x41 bridge A0.
      I2C_ADDR_44 = 0x44, ///< address 0x44 bridge A1.
      I2C_ADDR_45 = 0x45 ///< address 0x45 bridge A0 & A1.

    };

    ///Sets PGA gain and range. Note that the PGA defaults to be divided by 8 (320mV range).
    ///Configuration reister bits 11, 12 are used for this.
    enum t_gain{
        GAIN_1_40MV = 0,
        GAIN_2_80MV = 1,
        GAIN_4_160MV = 2,
        GAIN_8_320MV = 3
    };

    ///Bus voltage range.
    ///Configuration register bit 13 defines this.
    enum t_range{
        RANGE_16V = 0, ///< Range 0-16 volts
        RANGE_32V = 1  ///< Range 0-32 volts
    };

    ///ADC resolution (9-, 10-, 11-, or 12-bit) or set the number of samples.
    ///Same definations are used both shunt and bus adc.
    ///Configuration register bits 3 to 6 are used for averaging results for the Shunt Voltage Register (01h).
    ///And bits 7-10 are used averaging results for the Bus Voltage Register (02h).
    /// \see t_reg
    enum t_adc{
        ADC_9BIT    = 0,  ///<  9bit converion time  84us.
        ADC_10BIT   = 1,  ///< 10bit converion time 148us.
        ADC_11BIT   = 2,  ///< 11bit converion time 2766us.
        ADC_12BIT   = 3,  ///< 12bit converion time 532us.
        ADC_2SAMP   = 9,  ///< 2 samples converion time 1.06ms.
        ADC_4SAMP   = 10, ///< 4 samples converion time 2.13ms.
        ADC_8SAMP   = 11, ///< 8 samples converion time 4.26ms.
        ADC_16SAMP  = 12, ///< 16 samples converion time 8.51ms
        ADC_32SAMP  = 13, ///< 32 samples converion time 17.02ms.
        ADC_64SAMP  = 14, ///< 64 samples converion time 34.05ms.
        ADC_128SAMP = 15, ///< 128 samples converion time 68.10ms.
    };

    ///Selects continuous, triggered, or power-down mode of operation. These bits default to continuous shunt and bus measurement mode.
    ///\n Please note: <b>Thus library does not support triggered converion I have comment out values to set trigged mode</b>.\n
    ///Configuration register bits 0-2 are used for this. \see t_reg
    enum t_mode{
        PWR_DOWN    = 0,
        /*
        TRIG_SH     = 1,
        TRIG_BUS    = 2,
        TRIG_SH_BUS = 3,
        */
        ADC_OFF     = 4,
        CONT_SH     = 5, ///<Shunt Continuous.
        CONT_BUS    = 6, ///<Bus Continuous.
        CONT_SH_BUS = 7  ///<Shunt and Bus, Continuous.
    };
    
    ///Constructor
    INA219b();


    /// calibration of equations and device.
    /// default values are for a 0.25 Ohm shunt on a 5V bus with max current of 1A
   /*
    void calibrate( float r_shunt        = D_SHUNT,         ///< Value of shunt in Omhms.
                float v_shunt_max    = D_V_SHUNT_MAX,   ///< Maximum value of voltage across shunt.
                float v_bus_max      = D_V_BUS_MAX,     ///< Maximum voltage of bus.
                float i_max_expected = D_I_MAX_EXPECTED ///< Maximum current draw of bus + shunt.
                );
*/
     void getData( int16_t * mVolt, int16_t * mAmpere, int16_t * mWatt ) ;
    
  private:
    /// INA219 memory registers.
    enum t_reg{
        CONFIG_R  = 0x00,    ///< configuration register.
        V_SHUNT_R = 0x01,    ///< differential shunt voltage.
        V_BUS_R   = 0x02,    ///< bus voltage (wrt to system/chip GND).
        P_BUS_R   = 0x03,    ///< system power draw (= V_BUS * I_SHUNT).
        I_SHUNT_R = 0x04,    ///< shunt current.
        CAL_R     = 0x05     ///< calibration register.
    };


    ///Read 16 word from given register address.
    int16_t read16( t_reg addr ///< Register address.
              ) const;

    /// Writes a 16-bit word (d) to register pointer (a).
    /// When selecting a register pointer to read from, (d) = 0
    void write16( t_reg addr,   ///< Register address.
              uint16_t data ///< Data to be writen.
             ) const;

};

#endif
