
typedef struct _DD {
        const int esp_chip_id;
        const String id;
} device_details ;

// current
const device_details devices[] = {
  // ---------------------- PROXIMITY --------------------------
  {0x0065B6C7, "001"},  // proximity TOF1 - VL53L1X - 4m
  {0x00AAA13E, "002"},  // proximity TOF1 - VL52L1X - 4m pololu
  {0x0065AFD6, "003"},  // proximity TOF0 - VL52L0X - 2m
  {0x0066455C, "004"},  // proximity TOF0 - VL52L0X - 2m
  {0x0066425C, "005"},  // gesture/proximity/colour
  {0x0065C28B, "006"},
  {0x0066465D, "007"},
  {0x00A97FCC, "008"},
  {0x0065EAC7, "009"},
  // ---------------------- OTHERS -----------------------------
  {0x00666608, "010"},  // Temperature & Humidity
  {0x0066460B, "011"},
  {0x00A8F31B, "012"},
  {0x00664624, "013"},
  {0x006648F0, "014"},
  {0x00A91667, "015"},
  {0x0065AFF1, "016"},
  {0x006648F3, "017"},
  {0x00664552, "018"},
  {0x006643D4, "019"},
  {0x00664871, "020"},
  {0x00, "099"} // default if not defined above
};
