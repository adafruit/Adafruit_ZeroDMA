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

void Adafruit_ZeroDMA::configure_triggeraction(
  dma_transfer_trigger_action action) {
  _config.trigger_action = action;
}

status_code Adafruit_ZeroDMA::allocate(void) {
  return dma_allocate(&_resource, &_config);
}

void Adafruit_ZeroDMA::setup_transfer_descriptor(
  void *source_memory, void *destination_memory,
  uint32_t xfercount, dma_beat_size beatsize,
  bool srcinc, bool destinc)
{
  uint8_t countsize; // Beat transfer size IN BYTES
  switch(beatsize) {
    default:                  countsize = 1; break;
    case DMA_BEAT_SIZE_HWORD: countsize = 2; break;
    case DMA_BEAT_SIZE_WORD:  countsize = 4; break;
  }

  dma_descriptor_get_config_defaults(&descriptor_config);

  descriptor_config.beat_size              = beatsize;
  descriptor_config.src_increment_enable   = srcinc;
  descriptor_config.dst_increment_enable   = destinc;
  descriptor_config.block_transfer_count   = xfercount;
  descriptor_config.source_address         = (uint32_t)source_memory;
  descriptor_config.destination_address    = (uint32_t)destination_memory;
  if(srcinc) // If increment enable, advance address to *end* of transfer
    descriptor_config.source_address      += countsize * xfercount;
  if(destinc) // If increment enable, advance address to *end* of transfer
    descriptor_config.destination_address += countsize * xfercount;

  dma_descriptor_create(&_descriptor, &descriptor_config);
}

status_code Adafruit_ZeroDMA::add_descriptor(DmacDescriptor* descriptor) {
  if(!descriptor) descriptor = &_descriptor;
  return dma_add_descriptor(&_resource, descriptor);
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

void Adafruit_ZeroDMA::register_callback(
  dma_callback_t callback, dma_callback_type type) {
  return dma_register_callback(&_resource, callback, type);
}

void Adafruit_ZeroDMA::enable_callback(dma_callback_type type) {
  return dma_enable_callback(&_resource, type);
}

void Adafruit_ZeroDMA::loop(boolean on) {
  if(on) {
    descriptor_config.next_descriptor_address = (uint32_t)&_descriptor;
  } else {
     descriptor_config.next_descriptor_address = 0;
  }
  _descriptor.DESCADDR.reg = descriptor_config.next_descriptor_address;
}

// Suspend/resume don't quite do what I thought -- avoid using for now.
void Adafruit_ZeroDMA::suspend(void) {
  dma_suspend_job(&_resource);
}

void Adafruit_ZeroDMA::resume(void) {
  dma_resume_job(&_resource);
}

// Abort is OK though.
void Adafruit_ZeroDMA::abort(void) {
  dma_abort_job(&_resource);
}

// Modify DMA descriptor with a new source address, destination address &
// block transfer count.  All other attributes (including increment enables,
// etc.) are unchanged.  Mostly for changing the data being pushed to a
// peripheral (DAC, SPI, whatev.)
void Adafruit_ZeroDMA::modify(void *src, void *dst, uint32_t len) {

  uint8_t countsize; // Beat transfer size IN BYTES
  switch(descriptor_config.beat_size) {
    default:                  countsize = 1; break;
    case DMA_BEAT_SIZE_HWORD: countsize = 2; break;
    case DMA_BEAT_SIZE_WORD:  countsize = 4; break;
  }

  if(src) {
    _descriptor.SRCADDR.reg = (uint32_t)src;
    if(descriptor_config.src_increment_enable)
      _descriptor.SRCADDR.reg += len * countsize;
  }

  if(dst) {
    _descriptor.DSTADDR.reg = (uint32_t)dst;
    if(descriptor_config.dst_increment_enable)
      _descriptor.DSTADDR.reg += len * countsize;
  }

  _descriptor.BTCNT.reg = len;

  cpu_irq_enter_critical();
  _resource.job_status = STATUS_BUSY;
  memcpy(&descriptor_section[_resource.channel_id], _resource.descriptor,
    sizeof(DmacDescriptor));
  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
  cpu_irq_leave_critical();
}

