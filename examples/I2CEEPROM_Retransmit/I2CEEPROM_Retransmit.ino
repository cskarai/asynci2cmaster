#include <TWIMaster.h>

#define I2C_ADDRESS 0x57

void dumpErrorStatus(uint8_t status)
{
  switch(status)
  {
    case TWI_STATUS_DEVICE_NOT_PRESENT:
      Serial.println(F("Device not present!"));
      break;
    case TWI_STATUS_TRANSMIT_ERROR:
      Serial.println(F("Bus error"));
      break;
    case TWI_STATUS_NEGATIVE_ACKNOWLEDGE:
      Serial.println(F("Negative acknowledge"));
      break;
  }
}

void sendCallback(uint8_t status)
{
  if( status == TWI_STATUS_OK )
  {
    Serial.println(F("Send ok"));
  }
  else
    dumpErrorStatus(status);
}

void setup() {

  Serial.begin(57600);
  Serial.println(F("Starting"));

  TWIMaster::begin();
  TWIMaster::setRetryCount(15);
  
  char arg[18];
  memset(arg+2, 255, 16);
  arg[0] = 0x08; // 0x800 address
  arg[1] = 0x00;
  
  TWIMaster::send(I2C_ADDRESS, arg, sizeof(arg), sendCallback);
  TWIMaster::send(I2C_ADDRESS, arg, sizeof(arg), sendCallback);
}

void loop() {
  TWIMaster::loop();
}

