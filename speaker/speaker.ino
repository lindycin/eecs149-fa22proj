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
#define VERBOSE_OUTPUT (0)

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

typedef struct node_record_s
{
  uint8_t  addr[6];    // Six byte device address
  int8_t   rssi;       // RSSI value
  uint32_t timestamp;  // Timestamp for invalidation purposes
  int8_t   reserved;   // Padding for word alignment
} node_record_t;
const uint8_t CUSTOM_UUID[] =
{
    0x83, 0x2C, 0x0A, 0xC4, 0xB3, 0x2D, 0x86, 0xBA,
    0x67, 0x4C, 0xA7, 0xB4, 0x4D, 0x3F, 0xE4, 0x85
};

BLEUuid uuid = BLEUuid(CUSTOM_UUID);

// counters
int count_yellow = 0;
int tot_dist_yellow = 0;
int count_purple = 0;
int tot_dist_purple = 0;

int vary_vol = -8;

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

  // start scanning (new)
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values

  /* Set the LED interval for blinky pattern on BLUE LED */
  Bluefruit.setConnLedInterval(250);

  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Filter out packet with a min rssi
   * - Interval = 100 ms, window = 50 ms
   * - Use active scan (used to retrieve the optional scan response adv packet)
   * - Start(0) = will scan forever since no timeout is given
   */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  // Bluefruit.Scanner.filterRssi(-80);            // Only invoke callback for devices with RSSI >= -80 dBm
  // Bluefruit.Scanner.filterUuid(uuid);           // Only invoke callback if the target UUID was found
  //Bluefruit.Scanner.filterMSD(0xFFFF);          // Only invoke callback when MSD is present with the specified Company ID
  Bluefruit.Scanner.setInterval(160, 80);       // in units of 0.625 ms
  Bluefruit.Scanner.useActiveScan(true);        // Request scan response data
  Bluefruit.Scanner.start(0);                   // 0 = Don't stop scanning after n seconds
  Serial.println("Scanning ...");

  // Setup the advertising packet
  // startAdv();

  // start scanning (old)
  // ble_gap_scan_params_t* scanparam;
  // scanparam->active = 1;
  // scanparam->extended = 1;
  // scanparam->interval = SCAN_INTERVAL;
  // scanparam->window = SCAN_WINDOW;
  // scanparam->timeout = SCAN_DURATION;
  // scanparam->scan_phys = BLE_GAP_PHY_1MBPS;
  // scanparam->filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL;

  // static uint8_t m_scan_buffer_data[16]; /**< buffer where advertising reports will be stored by the SoftDevice. */

  // // /**@brief Pointer to the buffer where advertising reports will be stored by the SoftDevice. */
  // static ble_data_t m_scan_buffer =
  // {
  //   m_scan_buffer_data,
  //   16
  // };

  // uint32_t err_code;
  // err_code = sd_ble_gap_scan_start(scanparam, &m_scan_buffer);
  // if (err_code != 8) {
  //   Serial.println("invalid: bad error");
  //   while (1);
  // }
  // Serial.println(*(m_scan_buffer.p_data));
  // Serial.println(m_scan_buffer_data[0]);
  // Serial.println(m_scan_buffer_data[1]);
  // Serial.println(m_scan_buffer_data[2]);
  // Serial.println(m_scan_buffer_data[3]);
  // Serial.println(m_scan_buffer_data[4]);
  // Serial.println(m_scan_buffer_data[5]);
  // Serial.println(m_scan_buffer_data[6]);
  // Serial.println(m_scan_buffer_data[7]);
  // Serial.println(m_scan_buffer_data[8]);

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
  // Serial.print("Duty cycle: ");
  // Serial.print(duty_cycle,DEC);
  // Serial.println("%");
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
  
  // Serial.print("pwm write: ");
  // Serial.print(val,DEC);
  if (val == 10) {
    analogWrite(MOTOR, val +vary_vol);
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

void scan_callback(ble_gap_evt_adv_report_t* report)
{
  node_record_t record;
  
  /* Prepare the record to try to insert it into the existing record list */
  memcpy(record.addr, report->peer_addr.addr, 6); /* Copy the 6-byte device ADDR */
  record.rssi = report->rssi;                     /* Copy the RSSI value */
  record.timestamp = millis();                    /* Set the timestamp (approximate) */

  if (record.addr[5] == 0xc0 && record.addr[4] == 0x98) {
    Serial.println("FOUND");
    // Serial.println(record.addr[4]);
    Serial.println(record.rssi);
     // rssi conversions
     tot_dist_purple += report->rssi;
    
    if (count_purple == 9) { // averaging 10 distances
        count_purple = 0;
        int rssi = tot_dist_purple/10;
        float power = (-67.0 - rssi)/(10.0*3.0);
        printf("power: %f", power);
        float dist = powf(10,power);
        if (dist <= 1.5) {
          vary_vol = -9;
        } else if (dist <= 2.1) {
          vary_vol = -8;
        } else {
          vary_vol = -4;
        }
        printf("dist: %f, ", dist);
        printf("\n");
        tot_dist_purple = 0;
        // advertise volume levels for speakers
    } else {
        count_purple += 1;
    }
  }
/* Fully parse and display the advertising packet to the Serial Monitor
 * if verbose/debug output is requested */
#if VERBOSE_OUTPUT
  uint8_t len = 0;
  uint8_t buffer[32];
  memset(buffer, 0, sizeof(buffer));

  /* Display the timestamp and device address */
  if (report->type.scan_response)
  {
    Serial.printf("[SR%10d] Packet received from ", millis());
  }
  else
  {
    Serial.printf("[ADV%9d] Packet received from ", millis());
  }
  // MAC is in little endian --> print reverse
  Serial.printBufferReverse(report->peer_addr.addr, 6, ':');
  Serial.println("");
  
  /* Raw buffer contents */
  Serial.printf("%14s %d bytes\n", "PAYLOAD", report->data.len);
  if (report->data.len)
  {
    Serial.printf("%15s", " ");
    Serial.printBuffer(report->data.p_data, report->data.len, '-');
    Serial.println();
  }

  /* RSSI value */
  Serial.printf("%14s %d dBm\n", "RSSI", report->rssi);

  /* Adv Type */
  Serial.printf("%14s ", "ADV TYPE");
  if ( report->type.connectable )
  {
    Serial.print("Connectable ");
  }else
  {
    Serial.print("Non-connectable ");
  }

  if ( report->type.directed )
  {
    Serial.println("directed");
  }else
  {
    Serial.println("undirected");
  }

  /* Shortened Local Name */
  if(Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME, buffer, sizeof(buffer)))
  {
    Serial.printf("%14s %s\n", "SHORT NAME", buffer);
    memset(buffer, 0, sizeof(buffer));
  }

  /* Complete Local Name */
  if(Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, buffer, sizeof(buffer)))
  {
    Serial.printf("%14s %s\n", "COMPLETE NAME", buffer);
    memset(buffer, 0, sizeof(buffer));
  }

  /* TX Power Level */
  if (Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_TX_POWER_LEVEL, buffer, sizeof(buffer)))
  {
    Serial.printf("%14s %i\n", "TX PWR LEVEL", buffer[0]);
    memset(buffer, 0, sizeof(buffer));
  }

  /* Check for BLE UART UUID */
  if ( Bluefruit.Scanner.checkReportForUuid(report, BLEUART_UUID_SERVICE) )
  {
    Serial.printf("%14s %s\n", "BLE UART", "UUID Found!");
  }

  /* Check for DIS UUID */
  if ( Bluefruit.Scanner.checkReportForUuid(report, UUID16_SVC_DEVICE_INFORMATION) )
  {
    Serial.printf("%14s %s\n", "DIS", "UUID Found!");
  }

  /* Check for Manufacturer Specific Data */
  len = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, buffer, sizeof(buffer));
  if (len)
  {
    Serial.printf("%14s ", "MAN SPEC DATA");
    Serial.printBuffer(buffer, len, '-');
    Serial.println();
    memset(buffer, 0, sizeof(buffer));
  }

  Serial.println();
#endif

  


 

  // For Softdevice v6: after received a report, scanner will be paused
  // We need to call Scanner resume() to continue scanning
  Bluefruit.Scanner.resume();
}