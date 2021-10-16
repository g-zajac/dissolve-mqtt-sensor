
typedef struct _DD {
        const int esp_chip_id;
        const String id;
} device_details ;

// current
const device_details devices[] = {
  {0x0065B6C7, "01"},
  {0x00AAA13E, "02"},
  {0x0065AFD6, "03"},
  {0x0066455C, "04"},
  {0x0066425C, "05"},
  {0x0065C28B, "06"},
  {0x0066465D, "07"},
  {0x00A97FCC, "08"},
  {0x0065EAC7, "09"},
  {0x00666608, "10"},
  {0x0066460B, "11"},
  {0x00A8F31B, "12"},
  {0x00664624, "13"},
  {0x006648F0, "14"},
  {0x00A91667, "15"},
  {0x0065AFF1, "16"},
  {0x006648F3, "17"},
  {0x00664552, "18"},
  {0x006643D4, "19"},
  {0x00664871, "20"},
  {0x00, "99"} // default if not defined above
};
