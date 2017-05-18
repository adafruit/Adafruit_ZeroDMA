#ifndef _ADAFRUIT_ZERODMA_H_
#define _ADAFRUIT_ZERODMA_H_

#include "Arduino.h"
#include "utility/dmac.h"
#include "utility/dma.h"

class Adafruit_ZeroDMA {
 public:
  Adafruit_ZeroDMA(void);

  status_code allocate(void);
  void configure_peripheraltrigger(uint32_t periphtrigger);

  void configure_triggeraction(dma_transfer_trigger_action action);

  void setup_transfer_descriptor(void *source_memory, void *dest_memory, uint32_t xfercount, dma_beat_size beatsize = DMA_BEAT_SIZE_BYTE, bool srcinc = true, bool destinc = true);
  status_code add_descriptor(DmacDescriptor* descriptor = NULL);
  status_code start_transfer_job(void);
  status_code free(void);
  void trigger_transfer(void);
  void register_callback(dma_callback_t callback, dma_callback_type type = DMA_CALLBACK_TRANSFER_DONE);
  void enable_callback(dma_callback_type type = DMA_CALLBACK_TRANSFER_DONE);
  void loop(boolean on);
  void suspend(),
       resume(),
       abort(),
       modify(void *src, void *dst, uint32_t len);

 private:  
  struct dma_resource_config _config;
  struct dma_descriptor_config descriptor_config;
  COMPILER_ALIGNED(16) DmacDescriptor _descriptor;
  struct dma_resource _resource;
};

#endif // _ADAFRUIT_ZERODMA_H_
