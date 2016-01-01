#include "Arduino.h"
#include "utility/dmac.h"
#include "utility/dma.h"

class Adafruit_ZeroDMA {
 public:
  Zero_DMA(void);

  status_code allocate(void);
  void configure_peripheraltrigger(uint32_t periphtrigger);
  void configure_triggeraction(dma_transfer_trigger_action action);

  void setup_transfer_descriptor(void *source_memory, void *dest_memory, uint32_t xfercount, dma_beat_size beatsize = DMA_BEAT_SIZE_BYTE, bool srcinc = true, bool destinc = true);
  status_code add_descriptor(void);
  status_code start_transfer_job(void);
  void trigger_transfer(void);
  void register_callback(dma_callback_t callback, dma_callback_type type = DMA_CALLBACK_TRANSFER_DONE);
  void enable_callback(dma_callback_type type = DMA_CALLBACK_TRANSFER_DONE);
 private:  
  struct dma_resource_config _config;
  COMPILER_ALIGNED(16) DmacDescriptor _descriptor;
  struct dma_resource _resource;
};

static void configure_dma_resource(struct dma_resource *resource);

