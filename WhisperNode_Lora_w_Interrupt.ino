/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the (early prototype version of) The Things Network.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in g1,
 *  0.1% in g2).
 *
 * Change DEVADDR to a unique address!
 * See http://thethingsnetwork.org/wiki/AddressSpace
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 * 29 December 2016 Modified by Zaki to cater for RF95 + Arduino 
 * 29 June 2019     Modified to use with Whisper Node
 *******************************************************************************/

#include <SPI.h>
#include <lmic.h>
#include <hal/hal.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

//These are located in the extras folder of hte Talk2 library https://bitbucket.org/talk2/talk2-library
#include <T2WhisperNode.h>
//#include <LowPower.h>
                        
T2Flash myFlash;  //added for Whisper Node 

int wakePin = 3;
int blueLED = 6;
int yellowLED = 9; 
int count=0;
int Vin_Available = 0; 
int wakeup_by_pin = 0;
int wakeup_by_wdt = 0;
int counter_to_seventyfive = 0;
int heartbeat_count = 4;
volatile float averageVcc = 0.0;
volatile float averageVbat = 0.0;

/********************************* ABP Section : Need to fill these *************************************************************************/
/* LoRaWAN NwkSKey, network session key
   This is the default Semtech key, which is used by the prototype TTN
   network initially. */
static const PROGMEM u1_t NWKSKEY[16] = { 0x4A, 0x58, 0x81, 0x98, 0x12, 0xD1, 0x88, 0xE1, 0x88, 0x44, 0x56, 0x9D, 0x65, 0x0E, 0xAA, 0x55 };
/*Eg : 9FE76F0216C06C4A3449DE5C9AF56162 as got from TTN dashboard*/

/* LoRaWAN AppSKey, application session key
   This is the default Semtech key, which is used by the prototype TTN
   network initially. */
static const u1_t PROGMEM APPSKEY[16] = { 0x4B, 0x17, 0xA9, 0xA1, 0xC3, 0xF6, 0xA7, 0xDE, 0x7D, 0xFA, 0x6C, 0x7F, 0xCC, 0x9D, 0x6C, 0xDF };
/*Eg : 6CA021BC3EF5A903D0CFD65557291CA0 as got from TTN dashboard */

/* LoRaWAN end-device address (DevAddr)
   See http://thethingsnetwork.org/wiki/AddressSpace */
static const u4_t DEVADDR = 0x26011828 ; // <-- Change this address for every node!
/*Eg : 26021F2E as got from TTN dashboard */

/* These callbacks are only used in over-the-air activation, so they are
   left empty here (we cannot leave them out completely unless
   DISABLE_JOIN is set in config.h, otherwise the linker will complain). */
   
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

/********************************************* ABP Section Ends ***************************************************************************/

static osjob_t sendjob;

/* Schedule TX every this many seconds */
const unsigned TX_INTERVAL = 5;  //every 5secs

/* Pin mapping */
const lmic_pinmap lmic_pins = {
        .nss = 10,
        .rxtx = LMIC_UNUSED_PIN,
        .rst = 7,
        .dio = {2, A2, LMIC_UNUSED_PIN},
};

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            delay(100);
            if(LMIC.dataLen) {
                 // data received in rx slot after tx
                 Serial.print(F("Received "));
                 Serial.print(LMIC.dataLen);
                 Serial.print(F(" bytes of payload: 0x"));
                 for (int i = 0; i < LMIC.dataLen; i++) {
                    if (LMIC.frame[LMIC.dataBeg + i] < 0x10) 
                    {
                        Serial.print(F("0"));
                    }
                    Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
                    
                    if (i==0) //check the first byte
                    {
                      if (LMIC.frame[LMIC.dataBeg + 0] == 0x00)
                      {
                          Serial.print(F(" Yes!!!! "));
                      }
                    }
                    
                 }
                 Serial.println();
            }

            wdt_reset();  //reset watchdog 

            if(Vin_Available == 1)
            {
                if(wakeup_by_wdt == 1){
                    Serial.println(F("Need to send heartbeat few times..."));
                    heartbeat_count --;
                    if(heartbeat_count == 0)
                    {
                        heartbeat_count = 4;
                        Serial.println(F("Heartbeat done...Now i am going to sleep..."));
                        delay(100);
                        LMIC_setSleep();
                        sleepNow();     // sleep function called here
                    }
                }
                else
                {
                    Serial.println(F("Vin is available...Now i am going to sleep..."));
                    delay(100);
                    LMIC_setSleep();
                    sleepNow();     // sleep function called here
                }
                
            }
            
            /* Schedule next transmission */
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
                        
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            /* data received in ping slot */
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    
    static uint8_t message[6];
    
    /* Check if there is not a current TX/RX job running */
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        /* Prepare upstream data transmission at the next possible time. */          

        Serial.println(" ");  
        
        /******* Vin Voltage Sensor ***************/
        averageVcc = T2Utils::readVoltage(T2_WPN_VIN_VOLTAGE, T2_WPN_VIN_CONTROL);
        int16_t averageVccInt = round(averageVcc);
        Serial.print(F("Vin :"));
        Serial.print(averageVccInt);
        Serial.println(" mV "); 
        message[0] = highByte(averageVccInt);
        message[1] = lowByte(averageVccInt);
        /**************************************/

        /******* Vbat Voltage Sensor ***************/
        averageVbat = T2Utils::readVoltage(T2_WPN_VBAT_VOLTAGE, T2_WPN_VBAT_CONTROL);
        int16_t averageVbatInt = round(averageVbat);
        Serial.print(F("Vbat :"));
        Serial.print(averageVbatInt);
        Serial.println(" mV "); 
        message[2] = highByte(averageVbatInt);
        message[3] = lowByte(averageVbatInt);
        /**************************************/

        /************** For Internal Temperature ************/
        Serial.print(F("Internal Temperature Sensor :"));
        Serial.println(GetTemp(),1);
        int16_t averageIntTemp = GetTemp() * 100;
        //Serial.println(averageIntTemp);
        message[4] = highByte(averageIntTemp);
        message[5] = lowByte(averageIntTemp);
        /****************************************************/
        
        LMIC_setTxData2(1, message, sizeof(message), 0);        
        Serial.println(F("Packet queued"));
        /*Print Freq being used*/
        Serial.print("Transmit on Channel : ");Serial.println(LMIC.txChnl);
        /*Print mV being supplied*/
        
        if(averageVccInt < 1500){   //less than 1500mV
          digitalWrite(yellowLED,HIGH);                 // Turn ON Yellow LED
          Vin_Available = 0; 
        }
        else if(averageVccInt > 4000){
          digitalWrite(yellowLED,LOW);                 // Turn OFF Yellow LED
          Vin_Available = 1;
        }

        averageVcc = 0;
                
    }
    /* Next TX is scheduled after TX_COMPLETE event. */
}

void setup() {
    Serial.begin(115200);
    Serial.println(F("Starting"));  
    
    // Flash - We're not using, so just power it down to save energy
    myFlash.init(T2_WPN_FLASH_SPI_CS);
    myFlash.powerDown(); 

    //wdt_enable(WDTO_8S);  //set watchdog timer - 8 seconds

    pinMode(blueLED, OUTPUT);                   // Blue LED
    pinMode(yellowLED, OUTPUT);                 // yellow LED
        
    pinMode(wakePin, INPUT);  
    attachInterrupt(1, wakeUpNow, FALLING);

    noInterrupts();                   // disable all the interrupts
    MCUSR = 0;                        // ensure that the reset vectors are off
    WDTCSR |= 0b00011000;             //set WDCE, WDE
    WDTCSR = 0b01000000 | 0b100001;   //Set WDIE and 8secs
    interrupts();                     // re-enable interrupts

    LoraInitialization();  // Do all Lora Init Stuff

    /* Start job */
    do_send(&sendjob);
}

void loop() {  
    os_runloop_once();   
}


double GetTemp(void)
{
  unsigned int wADC;
  double t;

  // The internal temperature has to be used
  // with the internal reference of 1.1V.
  // Channel 8 can not be selected with
  // the analogRead function yet.

  // Set the internal reference and mux.
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADEN);  // enable the ADC

  delay(20);            // wait for voltages to become stable.
  ADCSRA |= _BV(ADSC);  // Start the ADC

  // Detect end-of-conversion
  while (bit_is_set(ADCSRA,ADSC));

  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  wADC = ADCW;

  // The offset of 324.31 could be wrong. It is just an indication.
  t = (wADC - 304.31 ) / 1.22;

  // The returned temperature is in degrees Celcius.
  return (t);
}


void LoraInitialization(){
  /* LMIC init */
    os_init();
    /* Reset the MAC state. Session and pending data transfers will be discarded. */
    LMIC_reset();

    /****************** ABP Only uses this section *****************************************/
    /* Set static session parameters. Instead of dynamically establishing a session
       by joining the network, precomputed session parameters are be provided.*/
    #ifdef PROGMEM
    /* On AVR, these values are stored in flash and only copied to RAM
       once. Copy them to a temporary buffer here, LMIC_setSession will
       copy them into a buffer of its own again. */
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    /* If not running an AVR with PROGMEM, just use the arrays directly */
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif
    /******************* ABP Only Ends ****************************************************/

    /* Disable link check validation */
    LMIC_setLinkCheckMode(0);

    /* Set data rate and transmit power (note: txpow seems to be ignored by the library) */
    LMIC_setDrTxpow(DR_SF12,20);  //lowest Datarate possible in 915MHz region

}


void sleepNow() {  
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here  
    sleep_enable();          // enables the sleep bit in the mcucr register  
    attachInterrupt(1,wakeUpNow, FALLING); // use interrupt 1 (pin 3) and run function  
    wakeup_by_pin = 0;       // set a global variable to 0 before sleep
    wakeup_by_wdt = 0;       // set a global variable to 0 before sleep
    sleep_mode();            // here the device is actually put to sleep!!  
    // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP  
    sleep_disable();         // first thing after waking from sleep: disable sleep...
    wdt_reset();
    
    if(wakeup_by_pin == 1){  
        Serial.println(F("Woke up by pin ..."));  
        delay(100);  
        detachInterrupt(1);      // disables interrupt 1 on pin 3 so the wakeUpNow code will not be executed during normal running time. 
        LMIC_setStandby();
        LoraInitialization();
    }
    else if(wakeup_by_wdt == 1){
      digitalWrite(blueLED,HIGH);                 // Turn ON Blue LED 
      Serial.println(F("Woke up by wdt ..."));  
      delay(100);    
      do_watchdog_routine();
    }    
}  

void do_watchdog_routine()  {
   
   if(counter_to_seventyfive == 75){     
      counter_to_seventyfive = 0;
      Serial.println(F("Reaching around 10 minutes, send heartbeat now..."));
      delay(100);
      LMIC_setStandby();
      LoraInitialization();
   }else{
      counter_to_seventyfive++;
      Serial.print(F("Increment counter by one and sleep : "));
      Serial.println(counter_to_seventyfive);
      delay(100);
      LMIC_setSleep();
      digitalWrite(blueLED,LOW);                 // Turn ON Blue LED 
      sleepNow();
   }
}

ISR(WDT_vect){
  wakeup_by_wdt = 1;
}

void wakeUpNow() {   
  wakeup_by_pin = 1;
}  
