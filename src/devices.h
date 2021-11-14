
typedef struct _DD {
        const int esp_chip_id;
        const String id;
} device_details ;

// current
const device_details devices[] = {
  // ---------------------- PROXIMITY --------------------------
  {0x0065B6C7, "001"},  // proximity TOF1 - VL53L1X - 4m
  {0x00AAA13E, "002"},  // proximity TOF1 - VL52L1X - 4m pololu
  {0x0065AFD6, "003"},  // proximity TOF0 - VL52L0X - 2m - jumps
  {0x0066455C, "004"},  // proximity TOF0 - VL52L0X - 2m
  {0x0066425C, "005"},  // gesture/proximity/colour - RGB 16bit raw values
  {0x0065C28B, "006"},  // proximity ultrasonic SRF01
  {0x0066465D, "007"},  // proximity,ultrasonic SFR02
  {0x00A97FCC, "008"},  // proximity,ultrasonic HC-SR04
  {0x0065EAC7, "009"},  // air quality - CCS811 gas sensor
  // ---------------------- OTHERS -----------------------------
  {0x00666608, "010"},  // HR
  {0x0066460B, "011"},  // Thermal camera - 8x8
  {0x00A8F31B, "012"},  // RGB - light - TCS34725
  {0x00664624, "013"},  // Mic
  {0x006648F0, "014"},  // Humidity DHT22
  {0x00A91667, "015"},  // Humidity DHT22
  {0x0065AFF1, "016"},  // Light different then 012 [LIGHT - ISL29125] - ADC 16 bits - 65535
  {0x006648F3, "017"},  // Humidity DHT22
  {0x00664552, "018"},  // Humidity DHT22
  {0x00662689, "019"},  // weight single cell 1kg?
  {0x00664871, "020"},  // weight 4x50kg
  {0x0022AD43, "021"},  // Mobile Gyro on battery (Wemos D1)
  {0x00D52499, "022"},  // Dust sensor
  {0x00AAAAAA, "023"},  // Thermal camera Hi-Res 32x24 (ESP32) separate code
  {0x006643D4, "024"},  // dummy - spare box
  {0x0042970A, "031"},  // Sand valve - prototype
  {0x00D5B7A1, "032"},  // Sand valve 2
  {0x00D5A22A, "033"},  // Water valve 1
  {0x001B3B07, "034"},  // Water valve 2
  {0x00D5246C, "035"},  // Water valve 3
  {0x00D5B8A1, "036"},  // Water valve 4
  {0x00, "099"} // blank or dummy default if not defined above
};
