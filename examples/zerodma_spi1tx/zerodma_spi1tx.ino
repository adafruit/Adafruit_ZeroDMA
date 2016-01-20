// Double-buffered DMA on second SPI peripheral 'SPI1' (pins 11/12/13).
// Interleaves between two data buffers...one is filled with new data
// as the other is being transmitted; an explicit form of multitasking.

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
status_code      stat; // For DMA status

// Data we'll pipe out to SPI1.  There are TWO buffers; one being
// filled with new data while the other's being transmitted in
// the background.
#define DATA_LENGTH (512)
uint8_t source_memory[2][DATA_LENGTH],
        buffer_being_filled = 0, // Index of 'filling' buffer
        buffer_value        = 0; // Value of fill

volatile bool transfer_is_done = true; // Done yet?

// Callback for end-of-DMA-transfer
void dma_callback(struct dma_resource* const resource) {
  transfer_is_done = true;
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("DMA test: SPI1 data out");

  SPI1.begin();

  // Configure DMA for SERCOM1 (our 'SPI1' port on 11/12/13)
  Serial.println("Configuring");
  myDMA.configure_peripheraltrigger(SERCOM1_DMAC_ID_TX);
  myDMA.configure_triggeraction(DMA_TRIGGER_ACTON_BEAT);

  Serial.print("Allocating...");
  stat = myDMA.allocate();
  printStatus(stat);

  Serial.print("Adding descriptor...");
  stat = myDMA.add_descriptor();
  printStatus(stat);

  Serial.println("Registering & enabling callback");
  myDMA.register_callback(dma_callback); // by default, called when xfer done
  myDMA.enable_callback(); // by default, for xfer done registers
}

void loop() {
  // Fill buffer with new data.  The alternate buffer might still be
  // transmitting in the background via DMA.
  memset(source_memory[buffer_being_filled], buffer_value, DATA_LENGTH);

  // Wait for prior transfer to complete before starting new one...
  Serial.print("Waiting on prior transfer...");
  while(!transfer_is_done) Serial.write('.');
  SPI1.endTransaction();
  Serial.println("Done!");

  // Set up DMA transfer using the newly-filled buffer as source...
  myDMA.setup_transfer_descriptor(
    source_memory[buffer_being_filled], // move data from here
    (void *)(&SERCOM1->SPI.DATA.reg),   // to here
    DATA_LENGTH,                        // this many...
    DMA_BEAT_SIZE_BYTE,                 // bytes/hword/words
    true,                               // increment source addr?
    false);                             // increment dest addr?

  // Begin new transfer...
  Serial.println("Starting new transfer job");
  SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  transfer_is_done = false;                      // Reset 'done' flag
  stat             = myDMA.start_transfer_job(); // Go!
  printStatus(stat);

  // Switch buffer indices so the alternate buffer is filled/xfer'd
  // on the next pass.
  buffer_being_filled = 1 - buffer_being_filled;
  buffer_value++;
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

