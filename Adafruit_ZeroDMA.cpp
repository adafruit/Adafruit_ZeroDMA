#include <Adafruit_ZeroDMA.h>
#include "utility/dmac.h"
#include "utility/dma.h"

// mostly from asfdoc_sam0_dma_basic_use_case.html

Adafruit_ZeroDMA::Adafruit_ZeroDMA(void) {
  dma_get_config_defaults(&_config);
}

void Adafruit_ZeroDMA::configure_peripheraltrigger(uint32_t periphtrigger) {
  _config.peripheral_trigger = periphtrigger;
  //Serial.print("periph trigger: 0x"); Serial.println(periphtrigger,HEX);
}

void Adafruit_ZeroDMA::configure_triggeraction(dma_transfer_trigger_action action) {
  _config.trigger_action = action;
}

status_code Adafruit_ZeroDMA::allocate(void) {
  return dma_allocate(&_resource, &_config);
}

void Adafruit_ZeroDMA::setup_transfer_descriptor(void *source_memory, void *destination_memory, uint32_t xfercount, dma_beat_size beatsize, bool srcinc, bool destinc)
{
  uint8_t countsize;
  if (beatsize == DMA_BEAT_SIZE_BYTE) countsize = 1;
  if (beatsize == DMA_BEAT_SIZE_HWORD) countsize = 2; // in bytes
  if (beatsize == DMA_BEAT_SIZE_WORD) countsize = 4;  // in bytes
    
    dma_descriptor_get_config_defaults(&descriptor_config);

    descriptor_config.beat_size = beatsize;
    descriptor_config.dst_increment_enable = destinc;
    descriptor_config.src_increment_enable = srcinc;

    descriptor_config.block_transfer_count = xfercount;

    descriptor_config.source_address = (uint32_t)source_memory;
    if (srcinc) 
      descriptor_config.source_address += countsize*xfercount; // the *end* of the transfer

    descriptor_config.destination_address = (uint32_t)destination_memory;
    if (destinc) 
       descriptor_config.destination_address += countsize*xfercount; // the *end* of the transfer

    dma_descriptor_create(&_descriptor, &descriptor_config);
}

status_code Adafruit_ZeroDMA::add_descriptor(void) {
   return dma_add_descriptor(&_resource, &_descriptor);
}

status_code Adafruit_ZeroDMA::free(void) {
   return dma_free(&_resource);
}

void Adafruit_ZeroDMA::trigger_transfer(void) {
  return dma_trigger_transfer(&_resource);
}

status_code Adafruit_ZeroDMA::start_transfer_job(void) {
  return dma_start_transfer_job(&_resource);
}

void Adafruit_ZeroDMA::register_callback(dma_callback_t callback, dma_callback_type type) {
  return dma_register_callback(&_resource, callback, type);
}

void Adafruit_ZeroDMA::enable_callback(dma_callback_type type) {
  return dma_enable_callback(&_resource, type);
}

void Adafruit_ZeroDMA::loop(boolean on) {
  if (on) {
    descriptor_config.next_descriptor_address = (uint32_t)&_descriptor;
  } else {
     descriptor_config.next_descriptor_address = 0;
  }
  _descriptor.DESCADDR.reg = descriptor_config.next_descriptor_address;
}
