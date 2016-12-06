#include <AsyncI2CMaster.h>

#define I2C_ADDRESS 0x57

AsyncI2CMaster i2cMaster;

uint8_t state = 0;

void requestCallback(uint8_t status, void *arg, uint8_t * data, uint8_t datalen);

void dumpErrorStatus(uint8_t status)
{
  switch(status)
  {
    case I2C_STATUS_DEVICE_NOT_PRESENT:
      Serial.println(F("Device not present!"));
      break;
    case I2C_STATUS_TRANSMIT_ERROR:
      Serial.println(F("Bus error"));
      break;
    case I2C_STATUS_NEGATIVE_ACKNOWLEDGE:
      Serial.println(F("Negative acknowledge"));
      break;
  }
}

void initCallback(uint8_t status, void *arg)
{
  if( status == I2C_STATUS_OK )
  {
    Serial.println(F("Init ok"));
    state++;
  }
  else
    dumpErrorStatus(status);
}

void sendCallback(uint8_t status, void *arg)
{
  if( status == I2C_STATUS_OK )
  {
    Serial.println(F("Send ok"));
    i2cMaster.read(I2C_ADDRESS, 3, requestCallback);
  }
  else
    dumpErrorStatus(status);
}

void requestCallback(uint8_t status, void *arg, uint8_t * data, uint8_t datalen)
{
  if( status == I2C_STATUS_OK )
  {
    Serial.print(F("Data received: ["));
    for( uint8_t i=0; i < datalen; i++ )
    {
      char bf[10];
      sprintf_P(bf, PSTR("%02x "), data[i]);
      Serial.print(bf);
    }
    Serial.println(F("]"));

    if( ++state == 3 )
    {
      uint8_t arg[2];
      arg[0] = 0;
      arg[1] = 6;
      i2cMaster.send(I2C_ADDRESS, arg, sizeof(arg), sendCallback);
    } 
  }
  else
    dumpErrorStatus(status);
}

void setup() {

  Serial.begin(115200);
  Serial.println(F("Starting"));


  i2cMaster.begin();
  i2cMaster.setRetryCount(15);

  uint8_t arginit[12];
  arginit[0] = 0;
  arginit[1] = 0;
  for(int i=2; i < sizeof(arginit); i++)
    arginit[i] = i-1;
    
  i2cMaster.send(I2C_ADDRESS, arginit, sizeof(arginit), initCallback);
  
  uint8_t arg[2];
  arg[0] = 0;
  arg[1] = 0;
  i2cMaster.request(I2C_ADDRESS, arg, sizeof(arg), 3, requestCallback);

  arg[1] = 3;
  i2cMaster.request(I2C_ADDRESS, arg, sizeof(arg), 3, requestCallback);
}

void loop() {
  i2cMaster.loop();
}

