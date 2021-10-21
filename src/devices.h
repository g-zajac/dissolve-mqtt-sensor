
typedef struct _DD {
        const int esp_chip_id;
        const String id;
} device_details ;

// current
const device_details devices[] = {
  {0x0065B6C7, "001"},
  {0x00AAA13E, "002"},
  {0x0065AFD6, "003"},
  {0x0066455C, "004"},
  {0x0066425C, "005"},
  {0x0065C28B, "006"},
  {0x0066465D, "007"},
  {0x00A97FCC, "008"},
  {0x0065EAC7, "009"},
  {0x00666608, "010"},
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
