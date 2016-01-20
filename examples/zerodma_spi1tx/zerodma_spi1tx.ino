// DMA on second SPI peripheral 'SPI1' (pins 11/12/13)

#include <SPI.h>
#include <Adafruit_ASFcore.h>
#include "status_codes.h"
#include <Adafruit_ZeroDMA.h>
#include "utility/dmac.h"
#include "utility/dma.h"

// Declare second SPI peripheral 'SPI1':
SPIClass SPI1(      // 11/12/13 classic UNO-style SPI
  &sercom1,         // -> Sercom peripheral
  34,               // MISO pin (also digital pin 12)
  37,               // SCK pin  (also digital pin 13)
  35,               // MOSI pin (also digital pin 11)
  SPI_PAD_0_SCK_1,  // TX pad (MOSI, SCK pads)
  SERCOM_RX_PAD_3); // RX pad (MISO pad)

Adafruit_ZeroDMA myDMA;
status_code stat; // For DMA status codes

// Memory we'll pipe out to SPI1
#define DATA_LENGTH (2048)
uint8_t source_memory[DATA_LENGTH];

volatile bool transfer_is_done = false; // Done yet?

// Optional callback for end-of-DMA-transfer
void dma_callback(struct dma_resource* const resource) {
  transfer_is_done = true;
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("DMA test: SPI1 data out");

  // Prefill the source with incrementing bytes
  for(uint32_t i=0; i<DATA_LENGTH; i++) source_memory[i] = i;

  SPI1.begin();

  // Configure DMA for SERCOM1 (our 'SPI1' port on 11/12/13)
  Serial.println("Configuring");
  myDMA.configure_peripheraltrigger(SERCOM1_DMAC_ID_TX);
  myDMA.configure_triggeraction(DMA_TRIGGER_ACTON_BEAT);

  Serial.print("Allocating...");
  stat = myDMA.allocate();
  printStatus(stat);

  Serial.println("Setting up transfer");
  myDMA.setup_transfer_descriptor(
    source_memory,                    // move data from here
    (void *)(&SERCOM1->SPI.DATA.reg), // to here
    DATA_LENGTH,                      // this many...
    DMA_BEAT_SIZE_BYTE,               // bytes/hword/words
    true,                             // increment source addr?
    false);                           // increment dest addr?

  Serial.print("Adding descriptor...");
  stat = myDMA.add_descriptor();
  printStatus(stat);

  Serial.println("Registering & enabling callback");
  myDMA.register_callback(dma_callback); // by default, called when xfer done
  myDMA.enable_callback(); // by default, for xfer done registers
}

void loop() {
  Serial.println("Starting transfer job");
  // Set up 1 MHz SPI xfer
  SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  stat = myDMA.start_transfer_job();
  printStatus(stat);

  while(!transfer_is_done); // chill

  SPI1.endTransaction();
  Serial.println("Done!");
  delay(100);
}

void printStatus(status_code stat) {
  Serial.print("Status ");
  switch (stat) {
    case STATUS_OK:
      Serial.println("OK"); break;
    case STATUS_BUSY:
      Serial.println("BUSY"); break;
    case STATUS_ERR_INVALID_ARG:
      Serial.println("Invalid Arg."); break;
    default:
      Serial.print("Unknown 0x"); Serial.println(stat); break;
  }
}

