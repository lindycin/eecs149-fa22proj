/*
 * motor_test.ino
 *
 * Sends a varying (bounded) input PWM
 * signal to pin 2_0 of the Launchpad.
 * The motor will begin slow, speed up,
 * and slow down again repeatedly.
 *
 * EE16B Fall 2016
 * John Maidens, Emily Naviasky & Nathaniel Mailoa
 *
 * EE16B Fall 2017
 * Andrew Blatner
 *
 * EE16B Spring 2019
 * Mia Mirkovic
 *
 */

#include <bluefruit.h>

#define MOTOR                  PIN_A0

#define RXLED                       17 // The RX LED has a defined Arduino pin
#define TXLED                       30 // The TX LED has a defined Arduino pin

float pwm = 0;
int dir = 1;

#define MANUFACTURER_ID   0x0059

// "nRF Connect" app can be used to detect beacon
uint8_t beaconUuid[16] =
{
  0x01, 0x12, 0x23, 0x34, 0x45, 0x56, 0x67, 0x78,
  0x89, 0x9a, 0xab, 0xbc, 0xcd, 0xde, 0xef, 0xf0
};

// A valid Beacon packet consists of the following information:
// UUID, Major, Minor, RSSI @ 1M
BLEBeacon beacon(beaconUuid, 0x0102, 0x0304, -54);

#define SCAN_INTERVAL                   0x00A0                                      /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                     0x0050                                      /**< Determines scan window in units of 0.625 millisecond. */

#define SCAN_DURATION                   0x0000                                      /**< Duration of the scanning in units of 10 milliseconds. If set to 0x0000, scanning will continue until it is explicitly disabled. */




void setup(void) {
  Serial.begin(38400);

  // beacon
  Serial.println("Bluefruit52 Beacon Example");
  Serial.println("--------------------------\n");

  Bluefruit.begin();

  // off Blue LED for lowest power consumption
  Bluefruit.autoConnLed(false);
  Bluefruit.setTxPower(0);    // Check bluefruit.h for supported values

  // Manufacturer ID is required for Manufacturer Specific Data
  beacon.setManufacturer(MANUFACTURER_ID);

  // Setup the advertising packet
  // startAdv();

  // start scanning
  ble_gap_scan_params_t* scanparam;
  // scanparam->active = 1;
  static ble_gap_scan_params_t const m_scan_params =
{
    .extended      = 1,
    .active        = 1,
    .interval      = SCAN_INTERVAL,
    .window        = SCAN_WINDOW,
    .timeout       = SCAN_DURATION,
    .scan_phys     = BLE_GAP_PHY_1MBPS,
    .filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL,
};
  const ble_data_t* buffer[16];
  Serial.println(sd_ble_gap_scan_start(scanparam, *buffer));
  Serial.println((int)buffer);

  Serial.println("Broadcasting beacon, open your beacon app to test");

  // Suspend Loop() to save power, since we didn't have any code there
  // suspendLoop();

  // speaker
  pinMode(MOTOR, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RXLED, OUTPUT);
  pinMode(TXLED, OUTPUT);
  
  Serial.print("Setup done\n");
  reset_blinker();

  write_pwm(0); // Turn off motor
    
}

void loop(void) {

  float duty_cycle = pwm/255*100;
  write_pwm(pwm);
  Serial.print("Duty cycle: ");
  Serial.print(duty_cycle,DEC);
  Serial.println("%");
  pwm = pwm + dir*5;
  if (pwm >= 10) {
    dir = -1;
  }
  if (pwm <= 0) {
    pwm = 10;
  }
  delay(100);
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void write_pwm(int pwm) {
  int val = (int) min(max(0, pwm), 255);
  Serial.print("pwm write: ");
  Serial.print(val,DEC);
  if (val == 10) {
    analogWrite(MOTOR, val + 2);
  } else {
    analogWrite(MOTOR, 1);
  }
  // analogWrite(MOTOR, val - 4);
}

void reset_blinker(void) {
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
}

void startAdv(void)
{  
  // Advertising packet
  // Set the beacon payload using the BLEBeacon class populated
  // earlier in this example
  Bluefruit.Advertising.setBeacon(beacon);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * Apple Beacon specs
   * - Type: Non connectable, undirected
   * - Fixed interval: 100 ms -> fast = slow = 100 ms
   */
  //Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_ADV_NONCONN_IND);
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(160, 160);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}