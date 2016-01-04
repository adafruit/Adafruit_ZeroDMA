#include <Adafruit_ASFcore.h>
#include <Adafruit_ZeroDMA.h>
#include "utility/dmac.h"
#include "utility/dma.h"

Adafruit_ZeroDMA myDMA;
status_code stat;  // we'll use this to read and print out the DMA status codes

// The memory we'll be moving!
#define DATA_LENGTH (1024)        // 1024 bytes -> 136us
uint8_t source_memory[DATA_LENGTH];
uint8_t destination_memory[DATA_LENGTH];

// are we done yet?
volatile bool transfer_is_done = false;

// If you like, a callback can be used
void dma_callback(struct dma_resource* const resource) {
    transfer_is_done = true;
}

void setup() {
  Serial.begin(115200);
  Serial.println("DMA test: simple memory moving");

  pinMode(13, OUTPUT);
  
  myDMA.allocate();
  Serial.println("Setting up transfer");
  myDMA.setup_transfer_descriptor(source_memory, destination_memory, DATA_LENGTH);

  Serial.println("Adding descriptor");
  stat = myDMA.add_descriptor();
  printStatus(stat);
  
  Serial.println("Registering & enabling callback");
  myDMA.register_callback(dma_callback); // by default, called when xfer done
  myDMA.enable_callback(); // by default, for xfer done registers
  
  // Prefill the source with incrementing bytes, dest with 0's
  for (uint32_t i=0; i<DATA_LENGTH; i++) { 
      source_memory[i] = i;
      destination_memory[i] = 0;

      Serial.print(destination_memory[i], HEX); Serial.print(' ');
      if ((i % 16) == 15) Serial.println();
  }
  
  Serial.println("Starting transfer job");
  stat = myDMA.start_transfer_job();
  printStatus(stat);
  
  Serial.println("Triggering DMA");
  myDMA.trigger_transfer();

  // 'raw' pin control, #13 is a.k.a PA17
  PORT->Group[0].OUTSET.reg = (1 << 17);  // set PORTA.17 high  "digitalWrite(13, HIGH)"
  
  while (! transfer_is_done);             // chill

  PORT->Group[0].OUTCLR.reg = (1 << 17);  // clear PORTA.17 high "digitalWrite(13, LOW)"
  
  Serial.println("Done!");

  
  for (uint32_t i=0; i<DATA_LENGTH; i++) { 
      Serial.print(destination_memory[i], HEX); Serial.print(' ');
      if ((i % 16) == 15) Serial.println();
  }
  
  // now do the same but 'manually'
  PORT->Group[0].OUTSET.reg = (1 << 17);  // set PORTA.17 high  "digitalWrite(13, HIGH)"
  
  memcpy(destination_memory, source_memory,  DATA_LENGTH); // 1024 bytes = 193us
 
  PORT->Group[0].OUTCLR.reg = (1 << 17);  // clear PORTA.17 high "digitalWrite(13, LOW)"

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
