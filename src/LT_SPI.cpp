//#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>
#include "Linduino.h"
#include "LT_SPI.h"

#define CS_PIN 5
static const int spiClk = 1000000; // 1 MHz

//uninitalised pointers to SPI objects
SPIClass * vspi = NULL;


// Reads and sends a byte
// Return 0 if successful, 1 if failed
void spi_transfer_byte(uint8_t cs_pin, uint8_t tx, uint8_t *rx)
{
  output_low(CS_PIN);                 //! 1) Pull CS low

  *rx = SPI.transfer(tx);             //! 2) Read byte and send byte

  output_high(CS_PIN);                //! 3) Pull CS high
}

// Reads and sends a word
// Return 0 if successful, 1 if failed
void spi_transfer_word(uint8_t cs_pin, uint16_t tx, uint16_t *rx)
{
  union
  {
    uint8_t b[2];
    uint16_t w;
  } data_tx;

  union
  {
    uint8_t b[2];
    uint16_t w;
  } data_rx;

  data_tx.w = tx;

  output_low(CS_PIN);                         //! 1) Pull CS low

  data_rx.b[1] = SPI.transfer(data_tx.b[1]);  //! 2) Read MSB and send MSB
  data_rx.b[0] = SPI.transfer(data_tx.b[0]);  //! 3) Read LSB and send LSB

  *rx = data_rx.w;

  output_high(CS_PIN);                        //! 4) Pull CS high
}

// Reads and sends a byte array
void spi_transfer_block(uint8_t cs_pin, uint8_t *tx, uint8_t *rx, uint8_t length)
{
  int8_t i;

  output_low(CS_PIN);                 //! 1) Pull CS low

  for (i=(length-1);  i >= 0; i--)
    rx[i] = SPI.transfer(tx[i]);    //! 2) Read and send byte array

  output_high(CS_PIN);                //! 3) Pull CS high
}


void ESP_SPI_init(void)  // Initializes SPI
{
  vspi = new SPIClass(VSPI);
  vspi->begin();
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  pinMode(5, OUTPUT); //VSPI SS
}

// Disable the SPI hardware port
void spi_disable()
{
  SPI.end();
}

// Write a data byte using the SPI hardware
void spi_write(int8_t  data)  // Byte to be written to SPI port
{
#if defined(ARDUINO_ARCH_AVR)
  SPDR = data;                  //! 1) Start the SPI transfer
  while (!(SPSR & _BV(SPIF)));  //! 2) Wait until transfer complete
#else
  SPI.transfer(data);
#endif

}

// Read and write a data byte using the SPI hardware
// Returns the data byte read
int8_t spi_read(int8_t  data) //!The data byte to be written
{
#if defined(ARDUINO_ARCH_AVR)
  SPDR = data;                  //! 1) Start the SPI transfer
  while (!(SPSR & _BV(SPIF)));  //! 2) Wait until transfer complete
  return SPDR;                  //! 3) Return the data read
#else
  return SPI.transfer(data);
#endif


}