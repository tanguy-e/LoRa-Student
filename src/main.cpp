
/*

This demonstrates how to save the join information in to permanent memory
so that if the power fails, batteries run out or are changed, the rejoin
is more efficient & happens sooner due to the way that LoRaWAN secures
the join process - see the wiki for more details.

This is typically useful for devices that need more power than a battery
driven sensor - something like a air quality monitor or GPS based device that
is likely to use up it's power source resulting in loss of the session.

The relevant code is flagged with a ##### comment

Saving the entire session is possible but not demonstrated here - it has
implications for flash wearing and complications with which parts of the 
session may have changed after an uplink. So it is assumed that the device
is going in to deep-sleep, as below, between normal uplinks.

Once you understand what happens, feel free to delete the comments and 
Serial.prints - we promise the final result isn't that many lines.

*/

#if !defined(ESP32)
  #pragma error ("This is not the example your device is looking for - ESP32 only")
#endif

// ##### load the ESP32 preferences facilites
#include <Preferences.h>
Preferences store;

// LoRaWAN config, credentials & pinmap
#include "config.h" 
#include <Arduino.h>
#include <RadioLib.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <CayenneLPP.h>

Adafruit_BMP280 bmp;
CayenneLPP lpp(51);


// utilities & vars to support ESP32 deep-sleep. The RTC_DATA_ATTR attribute
// puts these in to the RTC memory which is preserved during deep-sleep
RTC_DATA_ATTR uint16_t bootCount = 0;
RTC_DATA_ATTR uint16_t bootCountSinceUnsuccessfulJoin = 0;
RTC_DATA_ATTR uint8_t LWsession[RADIOLIB_LORAWAN_SESSION_BUF_SIZE];

// abbreviated version from the Arduino-ESP32 package, see
// https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/api/deepsleep.html
// for the complete set of options
void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
    Serial.println(F("Wake from sleep"));
  } else {
    Serial.print(F("Wake not caused by deep sleep: "));
    Serial.println(wakeup_reason);
  }

  Serial.print(F("Boot count: "));
  Serial.println(++bootCount);      // increment before printing
}

// put device in to lowest power deep-sleep mode
void gotoSleep(uint32_t seconds) {
  esp_sleep_enable_timer_wakeup(seconds * 1000UL * 1000UL); // function uses uS
  Serial.println(F("Sleeping\n"));
  Serial.flush();

  esp_deep_sleep_start();

  // if this appears in the serial debug, we didn't go to sleep!
  // so take defensive action so we don't continually uplink
  Serial.println(F("\n\n### Sleep failed, delay of 5 minutes & then restart ###\n"));
  delay(5UL * 60UL * 1000UL);
  ESP.restart();
}

int16_t lwActivate() {
  int16_t state = RADIOLIB_ERR_UNKNOWN;

  // setup the OTAA session information
  node.beginOTAA(joinEUI, devEUI, nwkKey, appKey);

  Serial.println(F("Recalling LoRaWAN nonces & session"));
  // ##### setup the flash storage
  store.begin("radiolib");
  // ##### if we have previously saved nonces, restore them and try to restore session as well
  if (store.isKey("nonces")) {
    uint8_t buffer[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];										// create somewhere to store nonces
    store.getBytes("nonces", buffer, RADIOLIB_LORAWAN_NONCES_BUF_SIZE);	// get them from the store
    state = node.setBufferNonces(buffer); 															// send them to LoRaWAN
    debug(state != RADIOLIB_ERR_NONE, F("Restoring nonces buffer failed"), state, false);

    // recall session from RTC deep-sleep preserved variable
    state = node.setBufferSession(LWsession); // send them to LoRaWAN stack

    // if we have booted more than once we should have a session to restore, so report any failure
    // otherwise no point saying there's been a failure when it was bound to fail with an empty LWsession var.
    debug((state != RADIOLIB_ERR_NONE) && (bootCount > 1), F("Restoring session buffer failed"), state, false);

    // if Nonces and Session restored successfully, activation is just a formality
    // moreover, Nonces didn't change so no need to re-save them
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(F("Succesfully restored session - now activating"));
      state = node.activateOTAA();
      debug((state != RADIOLIB_LORAWAN_SESSION_RESTORED), F("Failed to activate restored session"), state, true);

      // ##### close the store before returning
      store.end();
      return(state);
    }

  } else {  // store has no key "nonces"
    Serial.println(F("No Nonces saved - starting fresh."));
  }

  // if we got here, there was no session to restore, so start trying to join
  state = RADIOLIB_ERR_NETWORK_NOT_JOINED;
  while (state != RADIOLIB_LORAWAN_NEW_SESSION) {
    Serial.println(F("Join ('login') to the LoRaWAN Network"));
    state = node.activateOTAA();

    // ##### save the join counters (nonces) to permanent store
    Serial.println(F("Saving nonces to flash"));
    uint8_t buffer[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];           // create somewhere to store nonces
    uint8_t *persist = node.getBufferNonces();                  // get pointer to nonces
    memcpy(buffer, persist, RADIOLIB_LORAWAN_NONCES_BUF_SIZE);  // copy in to buffer
    store.putBytes("nonces", buffer, RADIOLIB_LORAWAN_NONCES_BUF_SIZE); // send them to the store
    
    // we'll save the session after an uplink

    if (state != RADIOLIB_LORAWAN_NEW_SESSION) {
      Serial.print(F("Join failed: "));
      Serial.println(state);

      // how long to wait before join attempts. This is an interim solution pending 
      // implementation of TS001 LoRaWAN Specification section #7 - this doc applies to v1.0.4 & v1.1
      // it sleeps for longer & longer durations to give time for any gateway issues to resolve
      // or whatever is interfering with the device <-> gateway airwaves.
      uint32_t sleepForSeconds = min((bootCountSinceUnsuccessfulJoin++ + 1UL) * 60UL, 3UL * 60UL);
      Serial.print(F("Boots since unsuccessful join: "));
      Serial.println(bootCountSinceUnsuccessfulJoin);
      Serial.print(F("Retrying join in "));
      Serial.print(sleepForSeconds);
      Serial.println(F(" seconds"));

      gotoSleep(sleepForSeconds);

    } // if activateOTAA state
  } // while join

  Serial.println(F("Joined"));

  // reset the failed join count
  bootCountSinceUnsuccessfulJoin = 0;

  delay(1000);  // hold off off hitting the airwaves again too soon - an issue in the US
  
  // ##### close the store
  store.end();  
  return(state);
}

// setup & execute all device functions ...
void setup() {
  Serial.begin(115200);
  while (!Serial);  							// wait for serial to be initalised
  delay(2000);  									// give time to switch to the serial monitor
  Serial.println(F("\nSetup"));
  print_wakeup_reason();
  
  unsigned status;
  status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                     "try a different address!"));
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
				  
				  
  int16_t state = 0;  						// return value for calls to RadioLib

  // setup the radio based on the pinmap (connections) in config.h
  uint8_t downlinkPayload[10];  // Make sure this fits your plans!
  size_t  downlinkSize;         // To hold the actual payload size received
  // you can also retrieve additional information about an uplink or 
  // downlink by passing a reference to LoRaWANEvent_t structure
  LoRaWANEvent_t uplinkDetails;
  LoRaWANEvent_t downlinkDetails;
  
  Serial.println(F("Initalise the radio"));
  state = radio.begin();
  debug(state != RADIOLIB_ERR_NONE, F("Initalise radio failed"), state, true);

  // activate node by restoring session or otherwise joining the network
  state = lwActivate();
  // state is one of RADIOLIB_LORAWAN_NEW_SESSION or RADIOLIB_LORAWAN_SESSION_RESTORED

  char str[30];
  float temperature = bmp.readTemperature();
  float pressure = (bmp.readPressure()/100.0);

  sprintf(str,"Température : %.1f °C",temperature);
  Serial.println();
  Serial.println(str);
  sprintf(str,"Pression : %.1f hPa",pressure);
  Serial.println(str);
  
  lpp.reset();

  lpp.addTemperature(1, temperature);
  lpp.addBarometricPressure(2, pressure);

  byte packet[lpp.getSize()];
  lpp.copy(packet);

  Serial.print("Octets envoyés : ");
  Serial.println(lpp.getSize());

  Serial.print("Trame envoyée : ");
  uint8_t *payload = lpp.getBuffer();

  for (unsigned char i = 0; i < lpp.getSize(); i++)
  {
    Serial.print("0x");
    Serial.print(payload[i], HEX);
    Serial.print(" ");
  }
    Serial.print(F("FcntUp: "));
  uint32_t fCntUp = node.getFCntUp();
  Serial.println(fCntUp);
  
  Serial.println(F("[LoRaWAN] En train d'envoyer la trame ... "));
  if(fCntUp == 1) {
    Serial.println(F("and requesting LinkCheck and DeviceTime"));
    node.sendMacCommandReq(RADIOLIB_LORAWAN_MAC_LINK_CHECK);
    node.sendMacCommandReq(RADIOLIB_LORAWAN_MAC_DEVICE_TIME);
    state = node.sendReceive(lpp.getBuffer(), lpp.getSize(), 10, downlinkPayload, &downlinkSize, true, &uplinkDetails, &downlinkDetails); 
  } else {
    state = node.sendReceive(lpp.getBuffer(), lpp.getSize(), 10, downlinkPayload, &downlinkSize, false, &uplinkDetails, &downlinkDetails);    
  }  


  debug(state < RADIOLIB_ERR_NONE, F("Error in sendReceive"), state, false);

  // Check if a downlink was received 
  // (state 0 = no downlink, state 1/2 = downlink in window Rx1/Rx2)
  if(state > 0) {
    Serial.println(F("Received a downlink"));
    // Did we get a downlink with data for us
    if(downlinkSize > 0) {
      Serial.println(F("Downlink data: "));
      arrayDump(downlinkPayload, downlinkSize);
      String str = (char*)downlinkPayload;
	  Serial.println(str);
    } else {
      Serial.println(F("<MAC commands only>"));
    }

    // print RSSI (Received Signal Strength Indicator)
    Serial.print(F("[LoRaWAN] RSSI:\t\t"));
    Serial.print(radio.getRSSI());
    Serial.println(F(" dBm"));

    // print SNR (Signal-to-Noise Ratio)
    Serial.print(F("[LoRaWAN] SNR:\t\t"));
    Serial.print(radio.getSNR());
    Serial.println(F(" dB"));

    // print extra information about the event
    Serial.println(F("[LoRaWAN] Event information:"));
    Serial.print(F("[LoRaWAN] Confirmed:\t"));
    Serial.println(downlinkDetails.confirmed);
    Serial.print(F("[LoRaWAN] Confirming:\t"));
    Serial.println(downlinkDetails.confirming);
    Serial.print(F("[LoRaWAN] Datarate:\t"));
    Serial.println(downlinkDetails.datarate);
    Serial.print(F("[LoRaWAN] Frequency:\t"));
    Serial.print(downlinkDetails.freq, 3);
    Serial.println(F(" MHz"));
    Serial.print(F("[LoRaWAN] Frame count:\t"));
    Serial.println(downlinkDetails.fCnt);
    Serial.print(F("[LoRaWAN] Port:\t\t"));
    Serial.println(downlinkDetails.fPort);
    Serial.print(F("[LoRaWAN] Time-on-air: \t"));
    Serial.print(node.getLastToA());
    Serial.println(F(" ms"));
    Serial.print(F("[LoRaWAN] Rx window: \t"));
    Serial.println(state);

    uint8_t margin = 0;
    uint8_t gwCnt = 0;
    if(node.getMacLinkCheckAns(&margin, &gwCnt) == RADIOLIB_ERR_NONE) {
      Serial.print(F("[LoRaWAN] LinkCheck margin:\t"));
      Serial.println(margin);
      Serial.print(F("[LoRaWAN] LinkCheck count:\t"));
      Serial.println(gwCnt);
    }

    uint32_t networkTime = 0;
    uint8_t fracSecond = 0;
    if(node.getMacDeviceTimeAns(&networkTime, &fracSecond, true) == RADIOLIB_ERR_NONE) {
      Serial.print(F("[LoRaWAN] DeviceTime Unix:\t"));
      Serial.println(networkTime);
      Serial.print(F("[LoRaWAN] DeviceTime second:\t1/"));
      Serial.println(fracSecond);
    }
  
  } else {
    Serial.println(F("[LoRaWAN] No downlink received"));
  }



  // now save session to RTC memory
  uint8_t *persist = node.getBufferSession();
  memcpy(LWsession, persist, RADIOLIB_LORAWAN_SESSION_BUF_SIZE);
  
  // wait until next uplink - observing legal & TTN FUP constraints
  gotoSleep(uplinkIntervalSeconds);

}


// The ESP32 wakes from deep-sleep and starts from the very beginning.
// It then goes back to sleep, so loop() is never called and which is
// why it is empty.

void loop() {}
