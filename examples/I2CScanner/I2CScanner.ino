#include <AsyncI2CMaster.h>

AsyncI2CMaster i2cMaster;

uint8_t address = 1;

void scanCallback(uint8_t status, void *arg)
{
  if( status == I2C_STATUS_OK )
  {
    Serial.print(F("I2C found:"));
    if( address < 16 )
      Serial.print(F("0"));
    Serial.println(address, HEX);
  }

  if( ++address < 128 )
    i2cMaster.send(address, NULL, 0, scanCallback);
  else
    Serial.println("I2C scan done");
}

void setup()
{
  Serial.begin(115200);
  Serial.println(F("Starting"));


  i2cMaster.begin();
  i2cMaster.setRetryCount(3);

  i2cMaster.send(address, NULL, 0, scanCallback);
}

void loop()
{
  i2cMaster.loop();
}

