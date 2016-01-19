#include <SPI.h>
#include <Adafruit_ASFcore.h>
#include "status_codes.h"
#include <Adafruit_ZeroDMA.h>
#include "utility/dmac.h"
#include "utility/dma.h"

Adafruit_ZeroDMA myDMA;
status_code stat;  // we'll use this to read and print out the DMA status codes

// The memory we'll be piping out to SPI
#define DATA_LENGTH (2048)
uint8_t source_memory[DATA_LENGTH];

// are we done yet?
volatile bool transfer_is_done = false;

// If you like, a callback can be used
void dma_callback(struct dma_resource* const resource) {
    transfer_is_done = true;
}

void setup() {
  Serial.begin(115200);
  Serial.println("DMA test: SPI data out");

  pinMode(13, OUTPUT);
  SPI.begin();
  
  Serial.println("Configuring");
  myDMA.configure_peripheraltrigger(SERCOM4_DMAC_ID_TX);  // SERMCOM4 == SPI native SERCOM
  myDMA.configure_triggeraction(DMA_TRIGGER_ACTON_BEAT);

  Serial.print("Allocating...");
  stat = myDMA.allocate();
  printStatus(stat);
  
  Serial.println("Setting up transfer");
  myDMA.setup_transfer_descriptor(source_memory,  // move data from here
                                  (void *)(&SERCOM4->SPI.DATA.reg), // to here
                                  DATA_LENGTH,                           // this many...
                                  DMA_BEAT_SIZE_BYTE,                    // bytes/hword/words
                                  true,                                  // increment source addr?
                                  false);                                // increment dest addr?

  Serial.print("Adding descriptor...");
  stat = myDMA.add_descriptor();
  printStatus(stat);
  
  Serial.println("Registering & enabling callback");
  myDMA.register_callback(dma_callback); // by default, called when xfer done
  myDMA.enable_callback(); // by default, for xfer done registers
  
  // Prefill the source with incrementing bytes, dest with 0's
  for (uint32_t i=0; i<DATA_LENGTH; i++) { 
      source_memory[i] = i;
  }
  
  Serial.println("Starting transfer job");
  
  // set up SPI
  SPI.beginTransaction(SPISettings(12000000, MSBFIRST, SPI_MODE0));
  
  // 'raw' pin control, #13 is a.k.a PA17, we're using this for timing
  PORT->Group[0].OUTSET.reg = (1 << 17);  // set PORTA.17 high  "digitalWrite(13, HIGH)"

  // once started, we dont need to trigger it because it will autorun
  stat = myDMA.start_transfer_job();
  printStatus(stat);
  
  while (! transfer_is_done);             // chill

  PORT->Group[0].OUTCLR.reg = (1 << 17);  // clear PORTA.17 high "digitalWrite(13, LOW)"

  SPI.endTransaction();  
  Serial.println("Done!");
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

void loop() {
}
