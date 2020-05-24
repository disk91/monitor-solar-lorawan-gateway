#include "Arduino.h"
#include <SPI.h>
#include "LoRaCom.h"
#include "LowPower.h"
#include <Wire.h>
#include "Ina219.h"

//#define DEBUG 
INA219b ina219;

#define EXTERNAL_VOLT_PIN A0    // 5V external volatge sensor (verify if the external 5V powering is On)
#define TRANSMIT_FREQ_S   300   // Time between transmission in seconde

#ifdef DEBUG
  #define LOGLN(x)  Serial.println x
  #define LOG(x) Serial.print x
#else
  #define LOGLN(x) 
  #define LOG(x)
#endif

static uint8_t myFrame[] = {
  0x07, 0x67, 0x00, 0x00,     // Temperature 16bit x10 High bit first           0 ..  3
  0x02, 0x02, 0x00, 0x00,     // Bat Current Signed x100 High bits first        4 ..  7
  0x03, 0x02, 0x00, 0x00,     // Bat Voltage Signed x100 High bits first        8 .. 11 
  0x04, 0x02, 0x00, 0x00,     // Internal Voltage Signed x100 High bits first  12 .. 15
  0x05, 0x66, 0x00,           // 5V powering status                            16 .. 18
  0x06, 0x02, 0x00, 0x00      // Bat Power                                     19 .. 22 
};

void setup() {
    // put your setup code here, to run once:
    #ifdef DEBUG
    Serial.begin(9600);
    #endif
    LOGLN((F("Boot")));
    
    Wire.begin();
    //ina219.calibrate(0.05, 0.32, 32, 6.4);

    loraSetup();

    pinMode(EXTERNAL_VOLT_PIN,INPUT);
}

/**
 * Read Arduino internal temperature
 */
int32_t readTemperature() {
 ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX3); 
 delay(100);
 int32_t rawTemp = 0; 
 for (int i=1; i< 1024; i++) {
   ADCSRA &= ~(_BV(ADATE) |_BV(ADIE));            // Clear auto trigger and interrupt enable
   ADCSRA |= _BV(ADEN) | _BV(ADSC);               // Enable AD and start conversion
   while((ADCSRA & _BV(ADSC)));                   // Wait until conversion is finished
   rawTemp += (ADCL | (ADCH << 8));
 }
 rawTemp >>= 10;
 const int32_t chipTempOffset = -309;       
 const int32_t chipTempCoeff = 881;      // x1000
 return (rawTemp*chipTempCoeff) / 1000 + chipTempOffset ;// (1000*(rawTemp - chipTempOffset) / chipTempCoeff);
}



long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

/**
 * Return 0 if the external voltage is 0 => voltage OFF
 * Return 1 otherwise
 */
uint8_t readExternalVoltage() {
   uint16_t ext= analogRead(EXTERNAL_VOLT_PIN);
   if ( ext < 200 ) return 0;   // usualy 5V = 800
   return 1;
}

/**
 * Read Battery line Current and Voltage
 * voltage is 0.01 V unit / current is 0.01 A unit / power is 0.01 W
 */
void readBatteryCurrentAndVoltage(int16_t * voltage, int16_t * current, int16_t * pwr) {
    int32_t _pwr;
    ina219.getData( voltage, current, &_pwr );
    *voltage /= 10;
    *current /= 10;
    *pwr = (int16_t)(_pwr / 10);
}

void loop() {
    static uint32_t duration = 0;

    long start = millis(); 

    if ( duration > TRANSMIT_FREQ_S ) {
      duration -= TRANSMIT_FREQ_S;
      int temp = readTemperature();
      int vcc = readVcc();
      int16_t batI;
      int16_t batU;
      int16_t batP;
      readBatteryCurrentAndVoltage(&batU,&batI,&batP);
      uint8_t extStatus = readExternalVoltage();
      
      LOG((F("Temp : ")));LOGLN((temp));
      LOG((F("Vcc : ")));LOGLN((vcc));
      LOG((F("BatU : ")));LOGLN((batU*10)); // mV
      LOG((F("BatI : ")));LOGLN((batI*10)); // mA
      LOG((F("BatP : ")));LOGLN((batP*10)); // mW
      LOG((F("Ext  : ")));LOGLN((extStatus)); // true / false
      
      myFrame[2]  = ((temp*10) >> 8) & 0xFF;
      myFrame[3]  = ((temp*10)     ) & 0xFF;
      myFrame[6]  = ((batI)    >> 8) & 0xFF;
      myFrame[7]  = ((batI)        ) & 0xFF;
      myFrame[10] = ((batU)    >> 8) & 0xFF;
      myFrame[11] = ((batU)        ) & 0xFF;
      myFrame[14] = ((vcc/10)  >> 8) & 0xFF;
      myFrame[15] = ((vcc/10)      ) & 0xFF;
      myFrame[18] = extStatus;
      myFrame[21] = ((batP)    >> 8) & 0xFF;
      myFrame[22] = ((batP)        ) & 0xFF;
      
      do_send(myFrame, sizeof(myFrame));
    }
    // put your main code here, to run repeatedly:
    loraLoop();
    
    // clock adjustment
    long stop = millis();
    if (stop > start) {
      duration += (500 + stop - start)/1000;
    }

    // deep sleep mode
    if ( canLoraSleep() ) {
      #ifdef DEBUG
      Serial.flush();
      #endif
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); 
      duration += 8; 
      #ifdef DEBUG
      Serial.begin(9600);
      LOG((F(".")));     
      #endif
    }
}

void dummy() {
  // This function will be called once on device wakeup
  // You can do some little operations here (like changing variables which will be used in the loop)
  // Remember to avoid calling delay() and long running functions since this functions executes in interrupt context
}
