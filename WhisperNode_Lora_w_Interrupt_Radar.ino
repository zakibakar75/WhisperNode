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
                        
T2Flash myFlash;  //added for Whisper Node 

int wakePin = 3;
int radarSensing = 0;
int wakeup_by_pin = 0;
volatile float averageVcc = 0.0;

/********************************* ABP Section : Need to fill these *************************************************************************/
/* LoRaWAN NwkSKey, network session key
   This is the default Semtech key, which is used by the prototype TTN
   network initially. */
static const PROGMEM u1_t NWKSKEY[16] = { 0xA1, 0xE9, 0x00, 0x2F, 0xBA, 0xA5, 0x67, 0xBC, 0xA6, 0xA6, 0xA0, 0x99, 0x6A, 0x1C, 0xC8, 0x90 };
/*Eg : 9FE76F0216C06C4A3449DE5C9AF56162 as got from TTN dashboard*/

/* LoRaWAN AppSKey, application session key
   This is the default Semtech key, which is used by the prototype TTN
   network initially. */
static const u1_t PROGMEM APPSKEY[16] = { 0xAA, 0x3D, 0x16, 0xCA, 0xE7, 0xFE, 0xEC, 0x82, 0xAF, 0xBB, 0x86, 0x09, 0xC6, 0x8E, 0xFE, 0x11 };
/*Eg : 6CA021BC3EF5A903D0CFD65557291CA0 as got from TTN dashboard */

/* LoRaWAN end-device address (DevAddr)
   See http://thethingsnetwork.org/wiki/AddressSpace */
static const u4_t DEVADDR = 0x260115BA ; // <-- Change this address for every node!
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
const unsigned TX_INTERVAL = 300;  //every 300secs

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
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            delay(100);
            Serial.println(F("Sleep Now"));
            delay(100);

            //radarSensing = digitalRead(wakePin);
            
            //if(radarSensing == LOW)
            //{
               LMIC_setSleep();
               sleepNow();     // sleep function called here
            //}            
            
            /* Schedule next transmission */
            //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
         
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    
    static uint8_t message[3];
    
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

        /************** For RadarSensing ************/
        Serial.print(F("Radar Sensing :"));
        Serial.println(radarSensing);
        message[2] = radarSensing;
        /****************************************************/
        
        LMIC_setTxData2(1, message, sizeof(message), 0);        
        Serial.println(F("Packet queued"));
        /*Print Freq being used*/
        Serial.print("Transmit on Channel : ");Serial.println(LMIC.txChnl);
        /*Print mV being supplied*/

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
        
    pinMode(wakePin, INPUT);  
    attachInterrupt(1, wakeUpNow, RISING);

    LoraInitialization();  // Do all Lora Init Stuff

    /* Start job */
    do_send(&sendjob);
}

void loop() {  
    os_runloop_once();   
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
    LMIC_setDrTxpow(DR_SF12,20);  //lowest Datarate possible in 923MHz region

}


void sleepNow() {  
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here  
    sleep_enable();          // enables the sleep bit in the mcucr register  
    attachInterrupt(1,wakeUpNow, RISING); // use interrupt 1 (pin 3) and run function  
    wakeup_by_pin = 0;       // set a global variable to 0 before sleep
    sleep_mode();            // here the device is actually put to sleep!!  
    // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP  
    sleep_disable();         // first thing after waking from sleep: disable sleep...
    
    if(wakeup_by_pin == 1){  
        Serial.println(F("Woke up by pin ..."));  
        delay(100);  
        detachInterrupt(1);      // disables interrupt 1 on pin 3 so the wakeUpNow code will not be executed during normal running time. 
        radarSensing = 1;
        LMIC_setStandby();
        LoraInitialization();
        do_send(&sendjob);
    }
  
}  

void wakeUpNow() {   
  wakeup_by_pin = 1;
}  
