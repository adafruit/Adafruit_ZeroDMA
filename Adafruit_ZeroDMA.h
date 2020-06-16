/*!
 * @file Adafruit_ZeroDMA.h
 *
 * This is part of Adafruit's DMA library for SAMD microcontrollers on
 * the Arduino platform. SAMD21 and SAMD51 lines are supported.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Phil "PaintYourDragon" Burgess for Adafruit Industries,
 * based partly on DMA insights from Atmel ASFCORE 3.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef _ADAFRUIT_ZERODMA_H_
#define _ADAFRUIT_ZERODMA_H_

#include "Arduino.h"
#ifdef DMAC_RESERVED_CHANNELS // SAMD core > 1.2.1
#include <dma.h>
#else
#include "utility/dma.h"
#endif

/** Status codes returned by some DMA functions and/or held in
    a channel's jobStatus variable. */
enum ZeroDMAstatus {
  DMA_STATUS_OK = 0,
  DMA_STATUS_ERR_NOT_FOUND,
  DMA_STATUS_ERR_NOT_INITIALIZED,
  DMA_STATUS_ERR_INVALID_ARG,
  DMA_STATUS_ERR_IO,
  DMA_STATUS_ERR_TIMEOUT,
  DMA_STATUS_BUSY,
  DMA_STATUS_SUSPEND,
  DMA_STATUS_ABORTED,
  DMA_STATUS_JOBSTATUS = -1 // For printStatus() function
};

/*!
    @brief  Class encapsulating DMA jobs and descriptors.
*/
class Adafruit_ZeroDMA {
public:
  Adafruit_ZeroDMA(void);

  // DMA channel functions

  /*!
    @brief  Allocate channel for ZeroDMA object. This allocates a CHANNEL,
            not a DESCRIPTOR.
    @return ZeroDMAstatus type:
            DMA_STATUS_OK on success.
            DMA_STATUS_ERR_NOT_FOUND if no DMA channels are free.

  */
  ZeroDMAstatus allocate(void);

  /*!
    @brief  Start a previously allocated-and-configured DMA job.
    @return ZeroDMAstatus type:
            DMA_STATUS_OK on success.
            DMA_STATUS_BUSY if resource is busy.
            DMA_STATUS_ERR_NOT_INITIALIZED if attempting to start job
            on a channel that failed to allocate.
            DMA_STATUS_ERR_INVALID_ARG if a bad transfer size was specified.
  */
  ZeroDMAstatus startJob(void);

  /*!
    @brief  Deallocates a previously-allocated DMA channel. This deallocates
            the CHANNEL, not any associated DESCRIPTORS.

    @return ZeroDMAstatus type:
            DMA_STATUS_OK on success.
            DMA_STATUS_BUSY if channel is busy (can't deallocate while in use).
            DMA_STATUS_ERR_NOT_INITIALIZED if channel isn't in use.
  */
  ZeroDMAstatus free(void);

  /*!
    @brief  Activate a previously allocated-and-configured DMA channel's
            software trigger.
  */
  void trigger(void);

  /*!
    @brief  Set DMA peripheral trigger. This can be done before or after
            channel is allocated.
    @param  trigger  A device-specific DMA peripheral trigger ID, typically
                     defined in a header file associated with that chip.
                     Example triffer IDs might include SERCOM2_DMAC_ID_TX
                     (SERCOM transfer complete) ADC_DMAC_ID_RESRDY (ADC
                     results ready).
  */
  void setTrigger(uint8_t trigger);

  /*!
    @brief  Set DMA trigger action. This can be done before or after
            channel is allocated.
    @param  action  One of DMA_TRIGGER_ACTON_BLOCK, DMA_TRIGGER_ACTON_BEAT or
                    DMA_TRIGGER_ACTON_TRANSACTION for desired behavior.
  */
  void setAction(dma_transfer_trigger_action action);

  /*!
    @brief  Set and enable callback function for ZeroDMA object. This can be
            called before or after channel and/or descriptors are allocated,
            but needs to be called before job is started.
    @param  callback  Pointer to callback function which accepts a pointer
                      to a ZeroDMA object (or NULL to disable callback).
    @param  type      Which DMA operation to attach this function to (a
                      channel can have multiple callbacks assigned if needed,
                      one for each of these situations):
                      DMA_CALLBACK_DONE on successful completion of a DMA
                      transfer.
                      DMA_CALLBACK_TRANSFER_ERR if a bus error is detected
                      during an AHB access or when the DMAC fetches an
                      invalid descriptor.
                      DMA_CALLBACK_CHANNEL_SUSPEND when a channel is suspended.
  */
  void setCallback(void (*callback)(Adafruit_ZeroDMA *) = NULL,
                   dma_callback_type type = DMA_CALLBACK_TRANSFER_DONE);

  /*!
    @brief  Select whether a channel's descriptor list should repeat or not.
            This can be called before or after channel and any descriptors
            are allocated.
    @param  flag  'true' if DMA descriptor list should repeat indefinitely,
                  'false' if DMA transfer stops at end of descriptor list.
  */
  void loop(boolean flag);

  /*!
    @brief  Suspend a DMA channel. AVOID USING FOR NOW.
  */
  void suspend(void);

  /*!
    @brief  Resume a previously-suspended DMA channel. AVOID USING FOR NOW.
  */
  void resume(void);

  /*!
    @brief  Cancel a DMA transfer operation.
  */
  void abort(void);

  /*!
    @brief  Set DMA channel level priority.
    @param  pri  DMA_PRIORITY_0 (lowest priority) through DMA_PRIORITY_3
                 (highest priority).
  */
  void setPriority(dma_priority pri);

  /*!
    @brief  Print (to Serial console) a string corresponding to a DMA
            job status value.
    @param  s  Job status as might be returned by allocate(), startJob(),
                etc., e.g. DMA_STATUS_OK, DMA_STATUS_ERR_NOT_FOUND, ...
  */
  void printStatus(ZeroDMAstatus s = DMA_STATUS_JOBSTATUS);

  /*!
    @brief   Get the DMA channel index associated with a ZeroDMA object.
    @return  uint8_t  Channel index (0 to DMAC_CH_NUM-1, or 0xFF).
  */
  uint8_t getChannel(void);

  // DMA descriptor functions

  /*!
    @brief   Allocate and append a DMA descriptor to a channel's descriptor
             list. Channel must be allocated first.
    @param   src       Source address.
    @param   dst       Destination address.
    @param   count     Transfer count.
    @param   size      Per-count transfer size (DMA_BEAT_SIZE_BYTE,
                       DMA_BEAT_SIZE_HWORD or DMA_BEAT_SIZE_WORD for 8, 16,
                       32 bits respectively).
    @param   srcInc    If true, increment the source address following each
                       count.
    @param   dstInc    If true, increment the destination address following
                       each count.
    @param   stepSize  If source/dest address increment in use, this indicates
                       the 'step size' (allowing it to skip over elements).
                       DMA_ADDRESS_INCREMENT_STEP_SIZE_1 for a contiguous
                       transfer, "_SIZE_2 for alternate items, "_SIZE_4
                       8, 16, 32, 64 or 128 for other skip ranges.
    @param   stepSel   DMA_STEPSEL_SRC or DMA_STEPSEL_DST depending which
                       pointer the step size should apply to (can't be used
                       on both simultaneously).
    @return  DmacDescriptor*  Pointer to DmacDescriptor structure, or NULL
                              on various errors. Calling code should keep the
                              pointer for later if it needs to change or free
                              the descriptor.
  */
  DmacDescriptor *
  addDescriptor(void *src, void *dst, uint32_t count = 0,
                dma_beat_size size = DMA_BEAT_SIZE_BYTE, bool srcInc = true,
                bool dstInc = true,
                uint32_t stepSize = DMA_ADDRESS_INCREMENT_STEP_SIZE_1,
                bool stepSel = DMA_STEPSEL_DST);

  /*!
    @brief  Change a previously-allocated DMA descriptor. Only the most
            common settings (source, dest, count) are available here. For
            anything more esoteric, you'll need to modify the descriptor
            structure yourself.
    @param  d      Pointer to descriptor structure (as returned by
                   addDescriptor()).
    @param  src    New source address.
    @param  dst    New destination address.
    @param  count  New transfer count.
  */
  void changeDescriptor(DmacDescriptor *d, void *src = NULL, void *dst = NULL,
                        uint32_t count = 0);

  /*!
    @brief  Interrupt handler function, used internally by the library,
            DO NOT TOUCH!
    @param  flags  Channel number, actually.
  */
  void _IRQhandler(uint8_t flags);

  /*!
    @brief   Test if DMA transfer is in-progress. Might be better to use
             callback and flag, unsure.
    @return  'true' if DMA channel is busy, 'false' otherwise.
  */
  bool isActive();

protected:
  uint8_t channel; ///< DMA channel index (0 to DMAC_CH_NUM-1, or 0xFF)
  volatile enum ZeroDMAstatus jobStatus; ///< Last known DMA job status
  bool hasDescriptors;       ///< 'true' if one or more descriptors assigned
  bool loopFlag;             ///< 'true' if descriptor chain loops back to start
  uint8_t peripheralTrigger; ///< Value set by setTrigger()
  dma_transfer_trigger_action triggerAction; ///< Value set by setAction()
  void (*callback[DMA_CALLBACK_N])(Adafruit_ZeroDMA *); ///< Callback func *s
};

#endif // _ADAFRUIT_ZERODMA_H_
