#include "EEPROM.h"
#include "Wire.h"
#include "ZUNO_OLED_I2C.h"
//#include <ZUNO_OLED_FONT_NUMB16.h>
#include <ZUNO_OLED_FONT_NUMB24.h>
//#include <ZUNO_OLED_FONT_NUMB40.h>
// You can use another builtin fonts for numbers..._NUMB24, _NUMB40

#define SW_VERSION "SW 1.00"
#define DEVICE_NAME "Spis"

#ifndef OLED_SMALLFONT_MODIFIED
  // If this flag is not set, it means that the my modified ZUNO_OLED_I2C.cpp and .h have been overwritten
  // The flag is set in ZUNO_OLED_I2C.h
  // If flag missing then generate a compile error
  foo();

#endif



// Set this flag to enable debug output in the serial monitor
//#define DEBUGMODE

#ifdef DEBUGMODE
  #define DEBUG(s) Serial.print(s)
  #define DEBUGLN(s) Serial.println(s)
  #define DEBUG2(s,f) Serial.print(s,f)
  #define DEBUGLN2(s,f) Serial.println(s,f)

#else
  #define DEBUG(s)
  #define DEBUGLN(s)
  #define DEBUG2(s,f)
  #define DEBUGLN2(s,f)
#endif

#define PULSE_LENGTH 100


/* How many pulses between calculations
 * The higher the number, the smaller the variance
 */

#define CALC_SPAN 2



/* Constants for EEPROM access
 *
 */
#define EEPROM_ADDR             0x800
#define EEPROM_UPDATE_INTERVAL  3600000

/* Energy meter ticks
 *
 * How many pulses/ticks per 1 kWh
 * Default setting: 1000
 */
#define TICKS_PER_KWH_PARAMETER 64
#define TICKS_PER_KWH_DEFAULT 1000
unsigned int ppwh;
unsigned int ppkwh;

/* Power reports
 *
 * Minimum change in consumed power that will result in sending new power report to the main controller.
 * 0 – reports are disabled
 * 1-100 (1-100%) – change in power
 * Default setting: 5 (5%)
 */
#define POWER_CHANGE_REPORT_PARAMETER 65
#define POWER_CHANGE_REPORT_DEFAULT 5
unsigned int p_changereport;

/* Time between power reports
 *
 * Minimum time that has to elapse before sending new power report to the main controller.
 * 0 – reports are disabled
 * 1-3600 (1-3600s) – report interval
 * Default setting: 32 (32s)
 */
#define POWER_TIME_REPORT_PARAMETER 66
#define POWER_TIME_REPORT_DEFAULT 30
unsigned int p_timereport;

/* Energy reports
 *
 * The minimum change in consumed energy that will result in sending new energy report to the main controller.
 * 0 – reports are disabled
 * 1-32000 (1 – 32000 kWh) – change in energy
 * Default setting: 10 (10kWh)
 */
#define ENERGY_CHANGE_REPORT_PARAMETER 67
#define ENERGY_CHANGE_REPORT_DEFAULT 10
unsigned int e_changereport;

/* Time between energy reports
 *
 * Minimum time that has to elapse before sending new energy report to the main controller.
 * 0 – reports are disabled
 * 1-3600 (1-3600s) – report interval
 * Default setting: 240 (240s)
 */
#define ENERGY_TIME_REPORT_PARAMETER 68
#define ENERGY_TIME_REPORT_DEFAULT 240
unsigned int e_timereport;

/* Energy seed
 *
 * For forcing the z-uno to initialize the energy counter at a specif value
 * After the seed has been received, this paramter will be cleared in the controller
 *
 * 1-32000 (1-32000 kWh) - seed value
 * Default setting: 0 (0 kWh)
 */
#define ENERGY_SEED_PARAMETER 69
#define ENERGY_SEED_DEFAULT 0
unsigned int e_seed;


unsigned int currentpower;
unsigned int previouspower;
unsigned int powerincrease;
unsigned int powerdelta;
unsigned long previouspowerreport;

unsigned int currentenergy;
unsigned int previousenergy;
unsigned int energyincrease;
unsigned long previousenergyreport;

// Timestammps and counters
unsigned long pulsecount = 0;
unsigned long previouspulsecount = 0;
unsigned int pulsessincelastcalc = 0;
unsigned long pulsestart = 0; // timestamp when the pulse started
unsigned long timesincelastcalc = 0;
unsigned long currenttime = 0;
unsigned long lastEEPROMsave = 0;

//unsigned long timediff = 0;
unsigned long pulses[CALC_SPAN];
unsigned long oldesttimestamp = 0;
int currenttimepos = 0;
int oldesttimepos = 0;
int i;

int EEPROMreturnval; // For capturing the return value from EEPROM operations

bool pulseactive = false;

bool sendpowerreport = false;
bool sendenergyreport = false;

// Interrupt pin where the pulse counter is attached
int pulsepin = 3;

// Output LED which will blnk at same frequency as meter signal
s_pin signalpin = 14;

// Driver for the oled display
OLED oled;

// Function to save the current timestamp in an array to be used later for calculation
void savetime(unsigned long time) {

    pulses[currenttimepos] = time;
    oldesttimepos = currenttimepos +1;
    if (oldesttimepos == CALC_SPAN) {oldesttimepos = 0;}

    currenttimepos++;
    if (currenttimepos == CALC_SPAN) {currenttimepos = 0;}
    return;
}

// Returns the oldest timestamp in the array
unsigned long oldesttime() {

    return pulses[oldesttimepos];
}

void inittimearray() {
  currenttimepos = 0;
  for (i=0 ; i < CALC_SPAN ; i++) {pulses[i] = 0;}

}

// EEPROM data structure
struct meter_data {
    unsigned long pulsecount;
    byte          crc8;
};

meter_data my_meter_data;

byte crc8(byte * data, byte count) {
    byte result = 0xDF;

    while(count--) {
        result ^= *data;
        data++;
    }
    return result;
}

void save_meter_data() {
    my_meter_data.crc8 = crc8((byte*)&my_meter_data, sizeof(meter_data) - 1);
    EEPROMreturnval = EEPROM.put(EEPROM_ADDR, &my_meter_data, sizeof(meter_data));
    DEBUG("Saving to EEPROM. code = ");
    DEBUG(EEPROMreturnval);
    DEBUG(", Pulsecount = ");
    DEBUGLN(my_meter_data.pulsecount);

}

/* Z-Wave channels
 *
 */
ZUNO_SETUP_CHANNELS(
    ZUNO_METER(
      ZUNO_METER_TYPE_ELECTRIC,
      METER_RESET_ENABLE,
      ZUNO_METER_ELECTRIC_SCALE_WATTS,
      METER_SIZE_FOUR_BYTES,
      METER_PRECISION_ZERO_DECIMALS,
      getter_power,
      resetter_power
    ),
    ZUNO_METER(
      ZUNO_METER_TYPE_ELECTRIC,
      METER_RESET_ENABLE,
      ZUNO_METER_ELECTRIC_SCALE_KWH,
      METER_SIZE_FOUR_BYTES,
      METER_PRECISION_ZERO_DECIMALS,
      getter_energy,
      resetter_energy
    )
  );

#define ZUNO_CHANNEL_NUMBER_POWER 1
#define ZUNO_CHANNEL_NUMBER_ENERGY 2


DWORD getter_power(){
  DEBUG("Getter - Power - Reported value = ");
  DEBUGLN(currentpower);
  return (DWORD)currentpower;
}

DWORD getter_energy(){
  DEBUG("Getter - Energy - Reported value = ");
  DEBUGLN(currentenergy);
  return (DWORD)currentenergy;
}

void resetter_power(byte v) {
  // This function must also reset the energy meter beaseuse the Fibaro HC2 does not call the resetter for the energy meter
  DEBUGLN("Resetter - Power");
  currentpower = 0;
  previouspower = 0;
  DEBUGLN("Calling resetter_energy");
  resetter_energy(0);
}

void resetter_energy(byte v) {
  // This funtion is not used when connected to a Fibaro HC2. Use the resetter_power instead
  DEBUGLN("Resetter - Energy");
  pulsecount = 0;
  previouspulsecount = 0;
  currentenergy = 0;
  previousenergy = 0;
  my_meter_data.pulsecount = 0;
  save_meter_data();
}

// For handling of z-uno parameters via controller
word paramvalue;

ZUNO_SETUP_CFGPARAMETER_HANDLER(config_parameter_changed);


void config_parameter_changed(byte param, word * value) {
  paramvalue = *value;

  DEBUG("Z-wave parameter received: ");
  DEBUG(param);
  DEBUG(" = ");
  DEBUGLN(paramvalue);



  switch (param) {
  case TICKS_PER_KWH_PARAMETER:
    ppkwh = (unsigned int)*value;
    break;
  case POWER_CHANGE_REPORT_PARAMETER:
    p_changereport = (unsigned int)*value;
    break;
  case POWER_TIME_REPORT_PARAMETER:
    p_timereport = (unsigned int)*value;
    break;
  case ENERGY_CHANGE_REPORT_PARAMETER:
    e_changereport = (unsigned int)*value;
    break;
  case ENERGY_TIME_REPORT_PARAMETER:
    e_timereport = (unsigned int)*value;
    break;
  case ENERGY_SEED_PARAMETER:
    e_seed = (unsigned int)*value;
    pulsecount = (unsigned long)e_seed* (unsigned long)ppkwh;
    DEBUG("kWh seed = ");
    DEBUG(e_seed);
    DEBUG(", ppkwh = ");
    DEBUG(ppkwh);
    DEBUG(", pulsecount = ");
    DEBUGLN(pulsecount);

    previouspulsecount = pulsecount;
    currentenergy = pulsecount / ppkwh;
    previousenergy = currentenergy;
    my_meter_data.pulsecount = pulsecount;
    save_meter_data();
    // Reset the value
    paramvalue = (word)0;

  }
  // Save/report back to the controller
  DEBUG("Z-wave parameter sent: ");
  DEBUG(param);
  DEBUG(" = ");
  DEBUGLN(paramvalue);

  //zunoSaveCFGParam(param, &paramvalue);


}

// Set up the interrupt handler on the ZEROX pin (3)
ZUNO_SETUP_ISR_ZEROX(pulse);


/* Interrupt proceure to be called when pulse is recevied
 *
 */
void pulse (){
  // 1.03
  currenttimepos++;
  if (currenttimepos == CALC_SPAN) {currenttimepos = 0;}
  pulses[currenttimepos] = millis();

  oldesttimepos = currenttimepos +1;
  if (oldesttimepos == CALC_SPAN) {oldesttimepos = 0;}


  // end 1.03
  pulsecount++;

}


void setup() {

  //#ifdef DEBUGMODE
  //Serial.begin();
  //#endif

  inittimearray();

  /* Set up values for counters
   * fetch from z-wave parameters
   */
  zunoLoadCFGParam(TICKS_PER_KWH_PARAMETER, &paramvalue);
  if (paramvalue == 0) {
    ppkwh = TICKS_PER_KWH_DEFAULT;
  } else {
    ppkwh = (unsigned int) paramvalue;
  }
  zunoLoadCFGParam(POWER_CHANGE_REPORT_PARAMETER, &paramvalue);
  if (paramvalue == 0) {
    p_changereport = POWER_CHANGE_REPORT_DEFAULT;
  } else {
    p_changereport = (unsigned int) paramvalue;
  }
  zunoLoadCFGParam(POWER_TIME_REPORT_PARAMETER, &paramvalue);
  if (paramvalue == 0) {
    p_timereport = POWER_TIME_REPORT_DEFAULT;
  } else {
    p_timereport = (unsigned int) paramvalue;
  }
  zunoLoadCFGParam(ENERGY_CHANGE_REPORT_PARAMETER, &paramvalue);
  if (paramvalue == 0) {
    e_changereport = ENERGY_CHANGE_REPORT_DEFAULT;
  } else {
    e_changereport = (unsigned int) paramvalue;
  }
  zunoLoadCFGParam(ENERGY_TIME_REPORT_PARAMETER, &paramvalue);
  if (paramvalue == 0) {
    e_timereport = ENERGY_TIME_REPORT_DEFAULT;
  } else {
    e_timereport = (unsigned int) paramvalue;
  }

  /* Set defualt parameters if not z-uno cfg parameters are used
  ppkwh = TICKS_PER_KWH_DEFAULT;
  p_changereport = POWER_CHANGE_REPORT_DEFAULT;
  p_timereport = POWER_TIME_REPORT_DEFAULT;
  e_changereport = ENERGY_CHANGE_REPORT_DEFAULT;
  e_timereport = ENERGY_TIME_REPORT_DEFAULT;
  */
  ppwh = ppkwh / 1000;


  // Get saved meter data from EEPROM
  EEPROMreturnval = EEPROM.get(EEPROM_ADDR,  &my_meter_data, sizeof(meter_data));

  // Check data consistency
  if (crc8((byte*)&my_meter_data, sizeof(meter_data) - 1) != my_meter_data.crc8) {
    // Invalid data - reset all
    my_meter_data.pulsecount = 0;

    //save_meter_data();
  }

  pulsecount = my_meter_data.pulsecount;
  previouspulsecount = my_meter_data.pulsecount;
  currentenergy = pulsecount / ppkwh;
  previousenergy = currentenergy;


  // Set up output LED which will blnk at same frequency as meter signel
  pinMode(signalpin, OUTPUT);

  // Interrupt pin where the pulse counter is attached
  pinMode(pulsepin, INPUT);


  previouspowerreport = 0;
  previousenergyreport = 0;


  currentpower = 1;
  previouspower = 1;


  DEBUGLN("Starting");


  // Initialize the oled display
  oled.begin();
  oled.clrscr();
  oled.setFont(SmallFont);
  oled.gotoXY(0,0);
  oled.setFont(SmallFont);
  /*oled.print("V");
  oled.write(228);
  oled.print("rmepump");
  */

  oled.print("Spis");

  oled.gotoXY(85,5);
  oled.print("Watt");
  oled.gotoXY(0,7);
  oled.print("Freq: ");
  oled.gotoXY(85,7);
  oled.print(SW_VERSION);

  // initiate the interrupt routine
  currenttime = millis();
  zunoExtIntMode(ZUNO_EXT_ZEROX, RISING);

}

void loop() {

  currenttime = millis();

  // Check if a pulse has been received
  if (pulsecount > previouspulsecount) {
    // Keep track of when the pulse started and switch on the LED
    pulsestart = currenttime;
    pulseactive = true;
    digitalWrite(signalpin,HIGH);

    //Save the timestamp for later calculation
    //savetime(currenttime);


    // Reset the calculation timer if we are starting a new cycle
    //if (pulsessincelastcalc == 0) lastcalc = currenttime;

    DEBUG("Ver 1.03 - ");
    DEBUGLN(pulsecount);

    //DEBUG("pulsessincelastcalc = ");
    //DEBUGLN(pulsessincelastcalc);

    //timesincelastcalc = currenttime - oldesttime();
    timesincelastcalc = pulses[currenttimepos]- pulses[oldesttimepos];

    pulsessincelastcalc = 0;


    currentpower = (3600000/timesincelastcalc)*(CALC_SPAN-1);
    //DEBUGLN("Calculate power");
    //DEBUG("timesincslastcalc= ");
    //DEBUGLN(timesincelastcalc);

    // Show current power consumption on the OLED
    // Secure that the number is 5 positions long by padding space characters in front
    oled.setFont(zuno_font_numbers24);
    oled.gotoXY(0,3);
    if (currentpower < 10) {
      oled.print("    ");
    }
    else if (currentpower < 100) {
      oled.print("   ");
    }
    else if (currentpower < 1000) {
      oled.print("  ");
    }
    else if (currentpower < 10000) {
      oled.print(" ");
    }
    oled.print(currentpower);
    oled.setFont(SmallFont);
    oled.gotoXY(38,7);
    oled.print(p_timereport);
    oled.print("s  ");



    if (currentpower > previouspower) {
      powerdelta = currentpower-previouspower;
    } else {
      powerdelta = previouspower-currentpower;
    }

    powerincrease = 100*powerdelta / previouspower;

    //DEBUG("P = ");
    //DEBUGLN(currentpower);


    previouspower = currentpower;
    //previouspulsecount = pulsecount;

    // Check if power level has increased enough for a report

    if ((powerincrease > p_changereport) && (p_changereport > 0)){
      // Change in power level higher than report threshold. Send report
      DEBUG("Power increase > ");
      DEBUG(p_changereport);
      DEBUGLN("%");
      sendpowerreport = true;

    }

    previouspulsecount = pulsecount;
  }

  // Check if enough time has passed so we should send a report
  if (((currenttime - previouspowerreport) > (p_timereport*1000)) && (p_timereport > 0)) {
    // Enough time has elapsed. Send report
    DEBUG("Power report time limit: ");
    DEBUGLN(p_timereport);
    sendpowerreport = true;
  }

  if (sendpowerreport == true) {
    DEBUG("Sending power report. Power : ");
    DEBUG(currentpower);
    DEBUGLN(" W");
    zunoSendReport(ZUNO_CHANNEL_NUMBER_POWER);
    previouspowerreport = currenttime;
    sendpowerreport = false;



  }

  /* Check energy consumption
   *
   */
  currentenergy = pulsecount / ppkwh;
  energyincrease = currentenergy - previousenergy;
  if ((energyincrease > e_changereport) && (e_changereport > 0)) {
    // Enough energy increase for a report
    DEBUG("Energy change: ");
    DEBUGLN(energyincrease);
    sendenergyreport = true;
  }
  previousenergy = currentenergy;

  // Check if enough time has passed so we should send a report
  if (((currenttime - previousenergyreport) > (e_timereport*1000)) && (e_timereport > 0)) {
    // Enough time has elapsed. Send report
    DEBUG("Energy report time limit: ");
    DEBUGLN(e_timereport);

    sendenergyreport = true;
  }

  if (sendenergyreport == true) {
    DEBUG("Sending energy report. Current energy : ");
    DEBUG(currentenergy);
    DEBUG(" kWh. Last value saved to EEPROM :");
    DEBUG(my_meter_data.pulsecount);
    DEBUG(" pulses (");
    DEBUG(my_meter_data.pulsecount / ppkwh);
    DEBUGLN(" kWh)");

    zunoSendReport(ZUNO_CHANNEL_NUMBER_ENERGY);

    previousenergyreport = currenttime;
    sendenergyreport = false;
  }

  // Check if it is time to save data to EEPROM
  if ((currenttime - lastEEPROMsave) > EEPROM_UPDATE_INTERVAL) {
    my_meter_data.pulsecount = pulsecount;
    save_meter_data();
    lastEEPROMsave = currenttime;
  }

  // Check if the LED should be switched off
  if ((pulseactive == true) && ((currenttime - pulsestart) > PULSE_LENGTH)) {
    digitalWrite(signalpin, LOW);
    pulseactive = false;
  }

}
