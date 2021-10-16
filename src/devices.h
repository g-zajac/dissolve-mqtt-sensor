
typedef struct _DD {
        const int esp_chip_id;
        const String id;
} device_details ;

// current
const device_details devices[] = {
  {0x0065B6C7, "01"},
  
  {0x0029DCAB, "02"},
  {0x00D2F1AE, "03"},
  {0x00000000, "99"} // default if not defined above
};
