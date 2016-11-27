#include "AsyncI2CMaster.h"
#include "Arduino.h"

#if defined(ESP8266)

#include "interrupts.h"

#define DISABLED_IRQ_SECTION_START {                           \
                                     InterruptLock irqLock;

#define DISABLED_IRQ_SECTION_LEAVE }

#else

#include <avr/io.h>
#include <avr/interrupt.h>

#define DISABLED_IRQ_SECTION_START {                           \
                                     uint8_t sreg = SREG;      \
                                     cli();

#define DISABLED_IRQ_SECTION_LEAVE   SREG = sreg;              \
                                   }

#endif

typedef enum
{
  OPERATION_READ,
  OPERATION_REQUEST,
  OPERATION_SEND,
  OPERATION_REQUEST_READ,
  OPERATION_PENDING,
} OperationEnum;

typedef enum
{
  I2C_STATUS_PENDING=253,
  I2C_STATUS_CLOSING=254,
  I2C_STATUS_FREE=255,
} InternalOperationEnum;

struct I2CTransaction
{
  uint8_t operation;
  uint8_t readLen;
  uint8_t writeLen;
  uint8_t i2cAddress;
  union {
    I2CSendCallback    sendCb;
    I2CRequestCallback requestCb;
    I2CReadCallback    readCb;
  } cb;
  uint8_t data[0];
};

void AsyncI2CMaster::begin()
{
  bufferPtr = 0;
  status = I2C_STATUS_FREE;

  dataPtr = 0;
  tries = 0;
  retryCount = 0;

  I2C_init();
}

struct I2CTransaction * AsyncI2CMaster::allocateTransaction(uint8_t i2cAddress, uint8_t wlen, uint8_t rlen)
{
  uint8_t len = ((wlen > rlen) ? wlen : rlen) + sizeof(struct I2CTransaction);
  if( len & 3 )
  {
    // padding to 4 bytes
    len |= 3;
    len++;
  }
  
  if( len + bufferPtr > MASTER_BUFFER_SIZE )
    return 0;
  
  struct I2CTransaction * t = (struct I2CTransaction *)(buffer + bufferPtr);
  t->operation = OPERATION_PENDING;
  t->readLen = rlen;
  t->writeLen = wlen;
  t->i2cAddress = i2cAddress;
  
  bufferPtr += len;
  return t;
}

void AsyncI2CMaster::startTransaction()
{
  if( (bufferPtr != 0) && (status == I2C_STATUS_FREE) )
  {
    status = I2C_STATUS_PENDING;
    tries = 0;
    I2C_startTransaction( (struct I2CTransaction *)buffer );
  }
}

uint8_t AsyncI2CMaster::send(uint8_t i2cAddress, uint8_t * data, uint8_t dataLen, I2CSendCallback callback)
{
  uint8_t ret; 
  DISABLED_IRQ_SECTION_START;
  
  struct I2CTransaction * t = allocateTransaction(i2cAddress, dataLen, 0);
  if( t != 0 )
  {
    t->cb.sendCb = callback;
    memcpy(t->data, data, dataLen);
    t->operation = OPERATION_SEND;
  
    startTransaction();
    ret = I2C_STATUS_OK;
  }
  else
    ret = I2C_STATUS_OUT_OF_MEMORY;
  
  DISABLED_IRQ_SECTION_LEAVE;
  return ret;
}

uint8_t AsyncI2CMaster::broadcast(uint8_t * data, uint8_t dataLen, I2CSendCallback callback)
{
  return send(0, data, dataLen, callback);
}

uint8_t AsyncI2CMaster::request(uint8_t i2cAddress, uint8_t * data, uint8_t dataLen, uint8_t receiveLen, I2CRequestCallback callback)
{
  uint8_t ret; 
  DISABLED_IRQ_SECTION_START;
  
  struct I2CTransaction * t = allocateTransaction(i2cAddress, dataLen, receiveLen );
  if( t == 0 )
    ret = I2C_STATUS_OUT_OF_MEMORY;
  else
  {
    t->cb.requestCb = callback;
    memcpy(t->data, data, dataLen);
    t->operation = OPERATION_REQUEST;

    startTransaction();
    ret = I2C_STATUS_OK;
  }

  DISABLED_IRQ_SECTION_LEAVE;
  return ret;
}

void AsyncI2CMaster::loop()
{
  if( bufferPtr == 0 ) // no outstanding transaction?
    return;
  
  struct I2CTransaction * t = (struct I2CTransaction *)buffer;
  I2C_handle(t);
  
  switch(status)
  {
    case I2C_STATUS_OK:
      {
        if( t->cb.sendCb != 0 )
        {
          switch(t->operation)
          {
            case OPERATION_SEND:
              t->cb.sendCb(status);
              break;
            case OPERATION_READ:
              t->cb.readCb(status, t->data, t->readLen);
              break;
            case OPERATION_REQUEST:
            case OPERATION_REQUEST_READ:
              t->cb.readCb(status, t->data, t->readLen);
              break;
          }
        }
        status = I2C_STATUS_CLOSING;
      }
      break;
    case I2C_STATUS_DEVICE_NOT_PRESENT:
      if( ++tries <= retryCount )
      {
        status = I2C_STATUS_PENDING;

        // repeated start
        I2C_startTransaction(t);
        break;
      }
    case I2C_STATUS_TRANSMIT_ERROR:
    case I2C_STATUS_NEGATIVE_ACKNOWLEDGE:
      {
        if( t->cb.sendCb != 0 )
        {
          switch(t->operation)
          {
            case OPERATION_SEND:
              t->cb.sendCb(status);
              break;
            case OPERATION_READ:
              t->cb.readCb(status, 0, 0);
              break;
            case OPERATION_REQUEST:
            case OPERATION_REQUEST_READ:
              t->cb.requestCb(status, 0, 0);
              break;
          }
        }
        status = I2C_STATUS_CLOSING;
      }
      break;
    case I2C_STATUS_CLOSING:
      {
        if( I2C_isStopped(t) )
        {
          status = I2C_STATUS_FREE;

          I2C_close(t);
      
          uint8_t len = ((t->writeLen > t->readLen) ? t->writeLen : t->readLen) + sizeof(struct I2CTransaction);
          if( len & 3 ) // padding
          {
            len |= 3;
            len++;
          }
      
          DISABLED_IRQ_SECTION_START;

          if( bufferPtr > len ) {
            memcpy(buffer, buffer+len, bufferPtr - len);
            bufferPtr -= len;
          } else
            bufferPtr = 0;
  
          DISABLED_IRQ_SECTION_LEAVE;
          startTransaction();
        }
      }
      break;
  }
}

uint8_t AsyncI2CMaster::read(uint8_t i2cAddress, uint8_t receiveLen, I2CReadCallback callback)
{
  uint8_t ret;
  DISABLED_IRQ_SECTION_START;
  
  struct I2CTransaction * t = allocateTransaction(i2cAddress, 0, receiveLen);
  if( t == 0 )
    ret = I2C_STATUS_OUT_OF_MEMORY;
  else
  {
    t->cb.readCb = callback;
    t->operation = OPERATION_READ;
  
    startTransaction();
    ret = I2C_STATUS_OK;
  }

  DISABLED_IRQ_SECTION_LEAVE;
  return ret;
}

#if defined(ESP8266)

#define SDA_LOW()   (GPES = (1 << pinSDA)) //Enable SDA (becomes output and since GPO is 0 for the pin, it will pull the line low)
#define SDA_HIGH()  (GPEC = (1 << pinSDA)) //Disable SDA (becomes input and since it has pullup it will go high)
#define SDA_READ()  ((GPI & (1 << pinSDA)) != 0)
#define SCL_LOW()   (GPES = (1 << pinSCL))
#define SCL_HIGH()  (GPEC = (1 << pinSCL))
#define SCL_READ()  ((GPI & (1 << pinSCL)) != 0)


typedef enum
{
  DONE,
  DATA_LOW,               // set data to LOW
  DATA_HIGH,              // set data to HIGH
  DATA_SEND,              // set data to required bit
  DATA_READ,              // read data
  CLOCK_LOW,              // set clock to LOW
  CLOCK_HIGH,             // set click to HIGH
  WAIT_25PC_PULSE,        // wait 0.25 pulse
  WAIT_75PC_PULSE,        // wait 0.75 pulse
  WAIT_1_PULSE,           // wait 1 pulse
  WAIT_DATA_HIGH,
  WAIT_CLOCK_HIGH,
} I2CBitBangState;

I2CBitBangState startBitBang[] = {
  DATA_HIGH, WAIT_DATA_HIGH, WAIT_1_PULSE, CLOCK_HIGH, WAIT_CLOCK_HIGH, WAIT_1_PULSE, DATA_LOW, WAIT_1_PULSE,
  CLOCK_LOW, WAIT_1_PULSE,
  DONE
};

I2CBitBangState sendByteBitBang[] = {
  DATA_SEND, WAIT_75PC_PULSE, CLOCK_HIGH, WAIT_CLOCK_HIGH, WAIT_1_PULSE, CLOCK_LOW, WAIT_25PC_PULSE, // bit 7
  DATA_SEND, WAIT_75PC_PULSE, CLOCK_HIGH, WAIT_CLOCK_HIGH, WAIT_1_PULSE, CLOCK_LOW, WAIT_25PC_PULSE, // bit 6
  DATA_SEND, WAIT_75PC_PULSE, CLOCK_HIGH, WAIT_CLOCK_HIGH, WAIT_1_PULSE, CLOCK_LOW, WAIT_25PC_PULSE, // bit 5
  DATA_SEND, WAIT_75PC_PULSE, CLOCK_HIGH, WAIT_CLOCK_HIGH, WAIT_1_PULSE, CLOCK_LOW, WAIT_25PC_PULSE, // bit 4
  DATA_SEND, WAIT_75PC_PULSE, CLOCK_HIGH, WAIT_CLOCK_HIGH, WAIT_1_PULSE, CLOCK_LOW, WAIT_25PC_PULSE, // bit 3
  DATA_SEND, WAIT_75PC_PULSE, CLOCK_HIGH, WAIT_CLOCK_HIGH, WAIT_1_PULSE, CLOCK_LOW, WAIT_25PC_PULSE, // bit 2
  DATA_SEND, WAIT_75PC_PULSE, CLOCK_HIGH, WAIT_CLOCK_HIGH, WAIT_1_PULSE, CLOCK_LOW, WAIT_25PC_PULSE, // bit 1
  DATA_SEND, WAIT_75PC_PULSE, CLOCK_HIGH, WAIT_CLOCK_HIGH, WAIT_1_PULSE, CLOCK_LOW, WAIT_25PC_PULSE, // bit 0
  DATA_HIGH, WAIT_75PC_PULSE, CLOCK_HIGH, WAIT_CLOCK_HIGH, DATA_READ, WAIT_1_PULSE, CLOCK_LOW, WAIT_25PC_PULSE, // bit ack
  DONE
};

I2CBitBangState readByteBitBang[] = {
  DATA_HIGH, WAIT_75PC_PULSE, CLOCK_HIGH, WAIT_CLOCK_HIGH, DATA_READ, WAIT_1_PULSE, CLOCK_LOW, WAIT_25PC_PULSE, // bit 7
  DATA_HIGH, WAIT_75PC_PULSE, CLOCK_HIGH, WAIT_CLOCK_HIGH, DATA_READ, WAIT_1_PULSE, CLOCK_LOW, WAIT_25PC_PULSE, // bit 5
  DATA_HIGH, WAIT_75PC_PULSE, CLOCK_HIGH, WAIT_CLOCK_HIGH, DATA_READ, WAIT_1_PULSE, CLOCK_LOW, WAIT_25PC_PULSE, // bit 6
  DATA_HIGH, WAIT_75PC_PULSE, CLOCK_HIGH, WAIT_CLOCK_HIGH, DATA_READ, WAIT_1_PULSE, CLOCK_LOW, WAIT_25PC_PULSE, // bit 4
  DATA_HIGH, WAIT_75PC_PULSE, CLOCK_HIGH, WAIT_CLOCK_HIGH, DATA_READ, WAIT_1_PULSE, CLOCK_LOW, WAIT_25PC_PULSE, // bit 3
  DATA_HIGH, WAIT_75PC_PULSE, CLOCK_HIGH, WAIT_CLOCK_HIGH, DATA_READ, WAIT_1_PULSE, CLOCK_LOW, WAIT_25PC_PULSE, // bit 2
  DATA_HIGH, WAIT_75PC_PULSE, CLOCK_HIGH, WAIT_CLOCK_HIGH, DATA_READ, WAIT_1_PULSE, CLOCK_LOW, WAIT_25PC_PULSE, // bit 1
  DATA_HIGH, WAIT_75PC_PULSE, CLOCK_HIGH, WAIT_CLOCK_HIGH, DATA_READ, WAIT_1_PULSE, CLOCK_LOW, WAIT_25PC_PULSE, // bit 0
  DATA_SEND, WAIT_75PC_PULSE, CLOCK_HIGH, WAIT_CLOCK_HIGH, WAIT_1_PULSE, CLOCK_LOW, WAIT_25PC_PULSE, // bit last
  DONE
};

I2CBitBangState stopBitBang[] = {
  DATA_LOW, WAIT_25PC_PULSE, CLOCK_HIGH, WAIT_CLOCK_HIGH, WAIT_1_PULSE, DATA_HIGH, WAIT_DATA_HIGH, WAIT_1_PULSE,
  DONE
};

typedef enum
{
  SEND_START,
  SEND_ADDRESS_READ,
  SEND_ADDRESS_WRITE,
  SEND_DATA_BYTES,
  RECEIVE_DATA_BYTES,
  SEND_REPEATED_START,
  SEND_STOP,
  SCENARIO_DONE,
} I2CScenario;

I2CScenario cancelScenario[] {
  SEND_STOP, SCENARIO_DONE
};

I2CScenario sendScenario[] = {
  SEND_START, SEND_ADDRESS_WRITE, SEND_DATA_BYTES, SEND_STOP, SCENARIO_DONE
};

I2CScenario requestScenario[] = {
  SEND_START, SEND_ADDRESS_WRITE, SEND_DATA_BYTES, SEND_REPEATED_START, SEND_ADDRESS_READ, RECEIVE_DATA_BYTES, SEND_STOP, SCENARIO_DONE
};

I2CScenario readScenario[] = {
  SEND_START, SEND_ADDRESS_READ, RECEIVE_DATA_BYTES, SEND_STOP, SCENARIO_DONE
};

void AsyncI2CMaster::I2C_init()
{
  bitBangPtr = scenarioPtr = 0;
  cycleToWait = 0;
  
  pinMode(pinSDA, INPUT_PULLUP);
  pinMode(pinSCL, INPUT_PULLUP);
}

void AsyncI2CMaster::I2C_handle(struct I2CTransaction *)
{
  if( cycleToWait )
  {
    int elapsed = ESP.getCycleCount() - savedCycle; 
    if( elapsed < cycleToWait )
      return;
    cycleToWait = 0;
    savedCycle = 0;
  }
  
  if( bitBangPtr )
  {
    I2CBitBangState * bitBang = (I2CBitBangState *)bitBangPtr;
    
    switch( *bitBang )
    {
      case DATA_LOW:
        SDA_LOW();
        break;
      case DATA_HIGH:
        SDA_HIGH();
        break;
      case DATA_SEND:
        if( dataWrite & 128 )
          SDA_HIGH();
        else
          SDA_LOW();

        dataWrite <<= 1;
        break;
      case DATA_READ:
        dataRead <<= 1;
        if( SDA_READ() )
          dataRead |= 1;
        break;
      case CLOCK_LOW:
        SCL_LOW();
        break;
      case CLOCK_HIGH:
        SCL_HIGH();
        break;
      case WAIT_25PC_PULSE:
        savedCycle = ESP.getCycleCount();
        cycleToWait = (F_CPU / I2C_FREQ)/4;
        break;
      case WAIT_75PC_PULSE:
        savedCycle = ESP.getCycleCount();
        cycleToWait = 3*(F_CPU / I2C_FREQ)/4;
        break;
      case WAIT_1_PULSE:
        savedCycle = ESP.getCycleCount();
        cycleToWait = (F_CPU / I2C_FREQ);
        break;
      case WAIT_DATA_HIGH:
        if( ! SDA_READ() )
          return;
        break;
      case WAIT_CLOCK_HIGH:
        if( ! SCL_READ() )
          return;
        break;
      case DONE:
        bitBangPtr = 0;
        I2C_bitBangCompleted();
        return;
    }

    bitBang++;
    bitBangPtr = bitBang;
  }
}

void AsyncI2CMaster::I2C_bitBangCompleted()
{
  I2CScenario * scenario = (I2CScenario *)scenarioPtr;
  
  switch(*scenario)
  {
    case SEND_START:
    case SEND_STOP:
    case SEND_REPEATED_START:
      I2C_playScenario(++scenario);
      break;
    case SEND_ADDRESS_READ:
    case SEND_ADDRESS_WRITE:
      if( dataRead ) // NACK ?
      {
        expectedStatus = I2C_STATUS_DEVICE_NOT_PRESENT;
        I2C_playScenario(cancelScenario);
      }
      else
        I2C_playScenario(++scenario);
      break;
    case SEND_DATA_BYTES:
      if( dataRead ) // NACK ?
      {
        expectedStatus = I2C_STATUS_NEGATIVE_ACKNOWLEDGE;
        I2C_playScenario(cancelScenario);
      }
      else
        I2C_playScenario(scenario);
      break;
    case RECEIVE_DATA_BYTES:
      {
        struct I2CTransaction * t = (struct I2CTransaction *)buffer;

        t->data[dataPtr++] = dataRead;
        if( dataPtr < t->readLen )
          I2C_playScenario(scenario);
        else
          I2C_playScenario(++scenario);
      }
      break;
    case SCENARIO_DONE:
      // can't happen
      break;
  }
}

void AsyncI2CMaster::I2C_playScenario(void *scenarioIn)
{
  scenarioPtr = scenarioIn;
  I2CScenario * scenario = (I2CScenario *)scenarioIn;

  struct I2CTransaction * t = (struct I2CTransaction *)buffer;
  
  switch(*scenario)
  {
    case SEND_START:
    case SEND_REPEATED_START:
      dataPtr = 0;
      I2C_sendBitBang(startBitBang);
      break;
    case SEND_ADDRESS_READ:
      if( t->operation == OPERATION_REQUEST )
        t->operation = OPERATION_REQUEST_READ;
      
      I2C_sendBitBang(sendByteBitBang);
      dataWrite = (t->i2cAddress << 1) + 1;
      break;
    case SEND_ADDRESS_WRITE:
      I2C_sendBitBang(sendByteBitBang);
      dataWrite = (t->i2cAddress << 1);
      break;
    case SEND_DATA_BYTES:
      if( dataPtr < t->writeLen )
      {
        I2C_sendBitBang(sendByteBitBang);
        dataWrite = t->data[dataPtr++];
      }
      else
        I2C_playScenario(++scenario);

      break;
      break;
    case RECEIVE_DATA_BYTES:
      I2C_sendBitBang(readByteBitBang);
      dataWrite = ( dataPtr + 1 >= t->readLen ) ? 128 : 0;
      break;
    case SEND_STOP:
      I2C_sendBitBang(stopBitBang);
      break;
    case SCENARIO_DONE:
      scenarioPtr = 0;
      if( expectedStatus != I2C_STATUS_PENDING )
        status = expectedStatus;
      else
        status = I2C_STATUS_OK;
      break;
  }
}

void AsyncI2CMaster::I2C_sendBitBang(void *bitBang)
{
  dataRead = dataWrite = 0;
  bitBangPtr = bitBang;
}

void AsyncI2CMaster::I2C_startTransaction(struct I2CTransaction *transaction)
{
  expectedStatus = I2C_STATUS_PENDING;
  
  switch( transaction->operation )
  {
    case OPERATION_SEND:
      I2C_playScenario(sendScenario);
      break;
    case OPERATION_REQUEST:
      I2C_playScenario(requestScenario);
      break;
    case OPERATION_REQUEST_READ:
    case OPERATION_READ:
      I2C_playScenario(readScenario);
      break;
  }
}

void AsyncI2CMaster::I2C_close(struct I2CTransaction *)
{
  // no need for special closing logic
}

bool AsyncI2CMaster::I2C_isStopped(struct I2CTransaction *)
{
  return (scenarioPtr == 0) && (bitBangPtr == 0);
}

#else

/****************************************************************************
  TWI State codes
****************************************************************************/
// General TWI Master staus codes                      
#define TWI_START                  0x08  // START has been transmitted  
#define TWI_REP_START              0x10  // Repeated START has been transmitted
#define TWI_ARB_LOST               0x38  // Arbitration lost

// TWI Master Transmitter staus codes                      
#define TWI_MTX_ADR_ACK            0x18  // SLA+W has been tramsmitted and ACK received
#define TWI_MTX_ADR_NACK           0x20  // SLA+W has been tramsmitted and NACK received 
#define TWI_MTX_DATA_ACK           0x28  // Data byte has been tramsmitted and ACK received
#define TWI_MTX_DATA_NACK          0x30  // Data byte has been tramsmitted and NACK received 

// TWI Master Receiver staus codes  
#define TWI_MRX_ADR_ACK            0x40  // SLA+R has been tramsmitted and ACK received
#define TWI_MRX_ADR_NACK           0x48  // SLA+R has been tramsmitted and NACK received
#define TWI_MRX_DATA_ACK           0x50  // Data byte has been received and ACK tramsmitted
#define TWI_MRX_DATA_NACK          0x58  // Data byte has been received and NACK tramsmitted

// TWI Slave Transmitter staus codes
#define TWI_STX_ADR_ACK            0xA8  // Own SLA+R has been received; ACK has been returned
#define TWI_STX_ADR_ACK_M_ARB_LOST 0xB0  // Arbitration lost in SLA+R/W as Master; own SLA+R has been received; ACK has been returned
#define TWI_STX_DATA_ACK           0xB8  // Data byte in TWDR has been transmitted; ACK has been received
#define TWI_STX_DATA_NACK          0xC0  // Data byte in TWDR has been transmitted; NOT ACK has been received
#define TWI_STX_DATA_ACK_LAST_BYTE 0xC8  // Last data byte in TWDR has been transmitted (TWEA = 0); ACK has been received

// TWI Slave Receiver staus codes
#define TWI_SRX_ADR_ACK            0x60  // Own SLA+W has been received ACK has been returned
#define TWI_SRX_ADR_ACK_M_ARB_LOST 0x68  // Arbitration lost in SLA+R/W as Master; own SLA+W has been received; ACK has been returned
#define TWI_SRX_GEN_ACK            0x70  // General call address has been received; ACK has been returned
#define TWI_SRX_GEN_ACK_M_ARB_LOST 0x78  // Arbitration lost in SLA+R/W as Master; General call address has been received; ACK has been returned
#define TWI_SRX_ADR_DATA_ACK       0x80  // Previously addressed with own SLA+W; data has been received; ACK has been returned
#define TWI_SRX_ADR_DATA_NACK      0x88  // Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
#define TWI_SRX_GEN_DATA_ACK       0x90  // Previously addressed with general call; data has been received; ACK has been returned
#define TWI_SRX_GEN_DATA_NACK      0x98  // Previously addressed with general call; data has been received; NOT ACK has been returned
#define TWI_SRX_STOP_RESTART       0xA0  // A STOP condition or repeated START condition has been received while still addressed as Slave

// TWI Miscellaneous status codes
#define TWI_NO_STATE               0xF8  // No relevant state information available
#define TWI_BUS_ERROR              0x00  // Bus error due to an illegal START or STOP condition

#define TWI_TWBR  ((F_CPU / I2C_FREQ) - 16) / 2 // TWI Bit rate Register setting.

void AsyncI2CMaster::I2C_init()
{
  TWSR = 0;
  TWBR = TWI_TWBR;                                  // Set bit rate register (Baudrate). Defined in header file.
  TWDR = 0xFF;                                      // Default content = SDA released.
  TWCR = (0<<TWEN)|                                 // Disable TWI-interface and release TWI pins.
         (0<<TWIE)|(0<<TWINT)|                      // Disable Interupt.
         (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // No Signal requests.
         (0<<TWWC);                                 //
 
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
}

void AsyncI2CMaster::I2C_startTransaction(struct I2CTransaction *)
{
  TWCR = (1<<TWEN)|                             // TWI Interface enabled.
         (0<<TWIE)|(1<<TWINT)|                  // Enable TWI Interupt and clear the flag.
         (0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|       // Initiate a START condition.
         (0<<TWWC);                             //
}

bool AsyncI2CMaster::I2C_isStopped(struct I2CTransaction *)
{
  return !( TWCR & _BV(TWSTO) );
}

void AsyncI2CMaster::I2C_close(struct I2CTransaction *)
{
  // disable TWI
  TWCR = (0<<TWEN)|
         (0<<TWIE)|(0<<TWINT)|
         (0<<TWEA)|(0<<TWSTA)|
         (0<<TWSTO)|(0<<TWWC);
}

void AsyncI2CMaster::I2C_handle(struct I2CTransaction * t)
{
  if( ! ( TWCR & (1<<TWINT) ) )
    return;
  
  if( (status == I2C_STATUS_FREE) || (status == I2C_STATUS_CLOSING ) )
  {
    TWCR |= (1<<TWINT);
    return;
  }
  
  uint8_t twsr = TWSR & 0xF8;

  switch (twsr)
  {
    case TWI_START:             // START has been transmitted  
    case TWI_REP_START:         // Repeated START has been transmitted
      dataPtr = 0;              // Set buffer pointer to 0
      
      switch( t->operation )
      {
        case OPERATION_READ:
        case OPERATION_REQUEST_READ:
          TWDR = (t->i2cAddress << 1) | 1;
          break;
        case OPERATION_REQUEST:
        case OPERATION_SEND:
          TWDR = (t->i2cAddress << 1) | 0;
          break;
      }
      TWCR = (1<<TWEN)|
             (0<<TWIE)|(1<<TWINT)|
             (0<<TWEA)|(0<<TWSTA)|
             (0<<TWSTO)|(0<<TWWC);
      break;
    case TWI_MTX_ADR_ACK:       // SLA+W has been tramsmitted and ACK received
    case TWI_MTX_DATA_ACK:      // Data byte has been tramsmitted and ACK received
      if (dataPtr < t->writeLen)
      {
        TWDR = t->data[dataPtr++];
        TWCR = (1<<TWEN)|
               (0<<TWIE)|(1<<TWINT)|
               (0<<TWEA)|(0<<TWSTA)|
               (0<<TWSTO)|(0<<TWWC);
      }else
      {
        TWDR = 0xFF;
        if( t->operation == OPERATION_SEND )
        {
           // Initiate a STOP condition.
           TWCR = (1<<TWEN)|
           (0<<TWIE)|(1<<TWINT)|
           (0<<TWEA)|(0<<TWSTA)|
           (1<<TWSTO)|(0<<TWWC);

           status = I2C_STATUS_OK;
        }
        else // request data (repeated start)
        {
           t->operation = OPERATION_REQUEST_READ;
           // repeated start
           TWCR = (1<<TWEN)|
                  (0<<TWIE)|(1<<TWINT)|
                  (0<<TWEA)|(1<<TWSTA)|
                  (0<<TWSTO)|(0<<TWWC);                             //
        }
      }
      break;
    case TWI_MRX_DATA_ACK:      // Data byte has been received and ACK transmitted
      t->data[dataPtr++] = TWDR;
    case TWI_MRX_ADR_ACK:       // SLA+R has been tramsmitted and ACK received

      if (dataPtr+1 < t->readLen )              // Detect the last byte to NACK it.
      {
        // send ack
        TWCR = (1<<TWEN)|
               (0<<TWIE)|(1<<TWINT)|
               (1<<TWEA)|(0<<TWSTA)|
               (0<<TWSTO)|(0<<TWWC);
      }else                    // Send NACK after next reception
      {
        // send nack
        TWCR = (1<<TWEN)|
               (0<<TWIE)|(1<<TWINT)|
               (0<<TWEA)|(0<<TWSTA)|
               (0<<TWSTO)|(0<<TWWC);                                 // 
      }    
      break;
    case TWI_MRX_DATA_NACK:     // Data byte has been received and NACK transmitted
      t->data[dataPtr] = TWDR;
      
      // send stop
      TWCR = (1<<TWEN)|
             (0<<TWIE)|(1<<TWINT)|
             (0<<TWEA)|(0<<TWSTA)|
             (1<<TWSTO)|(0<<TWWC);

      status = I2C_STATUS_OK;
      break;
    case TWI_ARB_LOST:          // Arbitration lost
      TWCR = (1<<TWEN)|                                 // TWI Interface enabled
             (0<<TWIE)|(1<<TWINT)|                      // Enable TWI Interupt and clear the flag
             (0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|           // Initiate a (RE)START condition.
             (0<<TWWC);                                 //
      break;
    case TWI_MTX_ADR_NACK:      // SLA+W has been tramsmitted and NACK received
    case TWI_MRX_ADR_NACK:      // SLA+R has been tramsmitted and NACK received    
      TWCR = (1<<TWEN)|
             (0<<TWIE)|(1<<TWINT)|
             (0<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|
             (0<<TWWC);
      status = I2C_STATUS_DEVICE_NOT_PRESENT;
      break;
    case TWI_MTX_DATA_NACK:     // Data byte has been tramsmitted and NACK received
      TWCR = (1<<TWEN)|
             (0<<TWIE)|(1<<TWINT)|
             (0<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|
             (0<<TWWC);
      status = I2C_STATUS_NEGATIVE_ACKNOWLEDGE;
      break;
    case TWI_BUS_ERROR:         // Bus error due to an illegal START or STOP condition
    default:     
      TWCR = (1<<TWEN)|
             (0<<TWIE)|(1<<TWINT)|
             (0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|
             (0<<TWWC);
      status = I2C_STATUS_TRANSMIT_ERROR;
  }
}

#endif
