/*!
 * @file Adafruit_ZeroDMA.cpp
 *
 * @mainpage Adafruit DMA Arduino library for SAMD microcontrollers.
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's DMA library for SAMD
 * microcontrollers on the Arduino platform. SAMD21 and SAMD51 lines
 * are supported.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section dependencies Dependencies
 *
 * @section author Author
 *
 * Written by Phil "PaintYourDragon" Burgess for Adafruit Industries,
 * based partly on DMA insights from Atmel ASFCORE 3.
 *
 * @section license License
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#include <Adafruit_ZeroDMA.h>
#include <malloc.h> // memalign() function
#include <stdlib.h>

#ifdef USE_TINYUSB
// For Serial when selecting TinyUSB
#include <Adafruit_TinyUSB.h>
#endif

#ifdef DMAC_RESERVED_CHANNELS // SAMD core > 1.2.1
#include <dma.h> // _descriptor[] and _writeback[] are extern'd here
static volatile uint32_t _channelMask = DMAC_RESERVED_CHANNELS;
#else
#include "utility/dma.h"
static volatile uint32_t _channelMask = 0; // Bitmask of allocated channels

// DMA descriptor list entry point (and writeback buffer) per channel
__attribute__((__aligned__(16))) static DmacDescriptor ///< 128 bit alignment
    _descriptor[DMAC_CH_NUM] SECTION_DMAC_DESCRIPTOR,  ///< Descriptor table
    _writeback[DMAC_CH_NUM] SECTION_DMAC_DESCRIPTOR;   ///< Writeback table
#endif

// Pointer to ZeroDMA object for each channel is needed for the
// ISR (in C, outside of class context) to access callbacks.
static Adafruit_ZeroDMA *_dmaPtr[DMAC_CH_NUM] = {0}; // Init to NULL

/// @cond INTERNAL
#if defined(__SAMD51__) || defined(__SAME51__)
#define ADAFRUIT_ZERODMA_HAS_DMAC_CHANNELS
#define ADAFRUIT_ZERODMA_DMAC_CHANNEL(channel) DMAC->Channel[channel]
#define ADAFRUIT_ZERODMA_DMAC_INTPEND_ID() DMAC->INTPEND.bit.ID
#define ADAFRUIT_ZERODMA_CHINT_TERR DMAC_CHINTENCLR_TERR
#define ADAFRUIT_ZERODMA_CHINT_TCMPL DMAC_CHINTENCLR_TCMPL
#define ADAFRUIT_ZERODMA_CHINT_SUSP DMAC_CHINTENCLR_SUSP
#define ADAFRUIT_ZERODMA_CHINT_MASK                                            \
  (ADAFRUIT_ZERODMA_CHINT_TERR | ADAFRUIT_ZERODMA_CHINT_TCMPL |                \
   ADAFRUIT_ZERODMA_CHINT_SUSP)
#elif defined(__SAME53__) || defined(__SAME54__)
#define ADAFRUIT_ZERODMA_HAS_DMAC_CHANNELS
#define ADAFRUIT_ZERODMA_HAS_DMAC_REGS
#define ADAFRUIT_ZERODMA_DMAC_CHANNEL(channel) DMAC_REGS->CHANNEL[channel]
#define ADAFRUIT_ZERODMA_DMAC_INTPEND_ID()                                     \
  ((DMAC_REGS->DMAC_INTPEND & DMAC_INTPEND_ID_Msk) >> DMAC_INTPEND_ID_Pos)
#define ADAFRUIT_ZERODMA_CHINT_TERR DMAC_CHINTFLAG_TERR_Msk
#define ADAFRUIT_ZERODMA_CHINT_TCMPL DMAC_CHINTFLAG_TCMPL_Msk
#define ADAFRUIT_ZERODMA_CHINT_SUSP DMAC_CHINTFLAG_SUSP_Msk
#define ADAFRUIT_ZERODMA_CHINT_MASK                                            \
  (ADAFRUIT_ZERODMA_CHINT_TERR | ADAFRUIT_ZERODMA_CHINT_TCMPL |                \
   ADAFRUIT_ZERODMA_CHINT_SUSP)
#endif

#ifndef ADAFRUIT_ZERODMA_CHINT_TERR
#define ADAFRUIT_ZERODMA_CHINT_TERR DMAC_CHINTENCLR_TERR
#define ADAFRUIT_ZERODMA_CHINT_TCMPL DMAC_CHINTENCLR_TCMPL
#define ADAFRUIT_ZERODMA_CHINT_SUSP DMAC_CHINTENCLR_SUSP
#define ADAFRUIT_ZERODMA_CHINT_MASK                                            \
  (ADAFRUIT_ZERODMA_CHINT_TERR | ADAFRUIT_ZERODMA_CHINT_TCMPL |                \
   ADAFRUIT_ZERODMA_CHINT_SUSP)
#endif

#ifdef ADAFRUIT_ZERODMA_HAS_DMAC_REGS
#define ADAFRUIT_ZERODMA_DMAC_BUSYCH() DMAC_REGS->DMAC_BUSYCH
#define ADAFRUIT_ZERODMA_DMAC_CHANNEL_BUSY(channel)                            \
  (ADAFRUIT_ZERODMA_DMAC_BUSYCH() & (1 << (channel)))
#elif defined(ADAFRUIT_ZERODMA_HAS_DMAC_CHANNELS)
#define ADAFRUIT_ZERODMA_DMAC_BUSYCH() DMAC->BUSYCH.reg
#define ADAFRUIT_ZERODMA_DMAC_CHANNEL_BUSY(channel)                            \
  (ADAFRUIT_ZERODMA_DMAC_CHANNEL(channel).CHSTATUS.reg & DMAC_CHSTATUS_BUSY)
#else
#define ADAFRUIT_ZERODMA_DMAC_BUSYCH() DMAC->BUSYCH.reg
#define ADAFRUIT_ZERODMA_DMAC_CHANNEL_BUSY(channel)                            \
  (DMAC->CHSTATUS.reg & DMAC_CHSTATUS_BUSY)
#endif

#if defined(__SAME53__) || defined(__SAME54__)
#define ADAFRUIT_ZERODMA_DESCRIPTOR_FIELD(desc, field) ((desc)->DMAC_##field)
#define ADAFRUIT_ZERODMA_BTCTRL_VALID DMAC_BTCTRL_VALID_Msk
#define ADAFRUIT_ZERODMA_BTCTRL_SRCINC DMAC_BTCTRL_SRCINC_Msk
#define ADAFRUIT_ZERODMA_BTCTRL_DSTINC DMAC_BTCTRL_DSTINC_Msk
#define ADAFRUIT_ZERODMA_BTCTRL_STEPSEL DMAC_BTCTRL_STEPSEL_Msk
#else
#define ADAFRUIT_ZERODMA_DESCRIPTOR_FIELD(desc, field) ((desc)->field.reg)
#define ADAFRUIT_ZERODMA_BTCTRL_VALID DMAC_BTCTRL_VALID
#define ADAFRUIT_ZERODMA_BTCTRL_SRCINC DMAC_BTCTRL_SRCINC
#define ADAFRUIT_ZERODMA_BTCTRL_DSTINC DMAC_BTCTRL_DSTINC
#define ADAFRUIT_ZERODMA_BTCTRL_STEPSEL DMAC_BTCTRL_STEPSEL
#endif // __SAME53__ / __SAME54__

static inline uint16_t descriptorBtctrl(const DmacDescriptor *desc) {
  return ADAFRUIT_ZERODMA_DESCRIPTOR_FIELD(desc, BTCTRL);
}

static inline void descriptorSetBtctrl(DmacDescriptor *desc, uint16_t value) {
  ADAFRUIT_ZERODMA_DESCRIPTOR_FIELD(desc, BTCTRL) = value;
}

static inline uint16_t descriptorBtcnt(const DmacDescriptor *desc) {
  return ADAFRUIT_ZERODMA_DESCRIPTOR_FIELD(desc, BTCNT);
}

static inline void descriptorSetBtcnt(DmacDescriptor *desc, uint16_t value) {
  ADAFRUIT_ZERODMA_DESCRIPTOR_FIELD(desc, BTCNT) = value;
}

static inline uint32_t descriptorSrcaddr(const DmacDescriptor *desc) {
  return ADAFRUIT_ZERODMA_DESCRIPTOR_FIELD(desc, SRCADDR);
}

static inline void descriptorSetSrcaddr(DmacDescriptor *desc, uint32_t value) {
  ADAFRUIT_ZERODMA_DESCRIPTOR_FIELD(desc, SRCADDR) = value;
}

static inline uint32_t descriptorDstaddr(const DmacDescriptor *desc) {
  return ADAFRUIT_ZERODMA_DESCRIPTOR_FIELD(desc, DSTADDR);
}

static inline void descriptorSetDstaddr(DmacDescriptor *desc, uint32_t value) {
  ADAFRUIT_ZERODMA_DESCRIPTOR_FIELD(desc, DSTADDR) = value;
}

static inline uint32_t descriptorDescaddr(const DmacDescriptor *desc) {
  return ADAFRUIT_ZERODMA_DESCRIPTOR_FIELD(desc, DESCADDR);
}

static inline void descriptorSetDescaddr(DmacDescriptor *desc, uint32_t value) {
  ADAFRUIT_ZERODMA_DESCRIPTOR_FIELD(desc, DESCADDR) = value;
}

static inline uint16_t descriptorBuildBtctrl(dma_beat_size size, bool srcInc,
                                             bool dstInc, bool stepSel,
                                             uint32_t stepSize) {
  return ADAFRUIT_ZERODMA_BTCTRL_VALID |
         DMAC_BTCTRL_EVOSEL(DMA_EVENT_OUTPUT_DISABLE) |
         DMAC_BTCTRL_BLOCKACT(DMA_BLOCK_ACTION_NOACT) |
         DMAC_BTCTRL_BEATSIZE(size) |
         (srcInc ? ADAFRUIT_ZERODMA_BTCTRL_SRCINC : 0) |
         (dstInc ? ADAFRUIT_ZERODMA_BTCTRL_DSTINC : 0) |
         (stepSel ? ADAFRUIT_ZERODMA_BTCTRL_STEPSEL : 0) |
         DMAC_BTCTRL_STEPSIZE(stepSize);
}

static inline dma_beat_size descriptorBeatSize(const DmacDescriptor *desc) {
  return static_cast<dma_beat_size>(
      (descriptorBtctrl(desc) & DMAC_BTCTRL_BEATSIZE_Msk) >>
      DMAC_BTCTRL_BEATSIZE_Pos);
}

static inline bool descriptorSrcInc(const DmacDescriptor *desc) {
  return (descriptorBtctrl(desc) & ADAFRUIT_ZERODMA_BTCTRL_SRCINC) != 0;
}

static inline bool descriptorDstInc(const DmacDescriptor *desc) {
  return (descriptorBtctrl(desc) & ADAFRUIT_ZERODMA_BTCTRL_DSTINC) != 0;
}

static inline bool descriptorStepSel(const DmacDescriptor *desc) {
  return (descriptorBtctrl(desc) & ADAFRUIT_ZERODMA_BTCTRL_STEPSEL) != 0;
}

static inline uint8_t descriptorStepSize(const DmacDescriptor *desc) {
  return (descriptorBtctrl(desc) & DMAC_BTCTRL_STEPSIZE_Msk) >>
         DMAC_BTCTRL_STEPSIZE_Pos;
}

static inline bool descriptorValid(const DmacDescriptor *desc) {
  return (descriptorBtctrl(desc) & ADAFRUIT_ZERODMA_BTCTRL_VALID) != 0;
}

#undef ADAFRUIT_ZERODMA_DESCRIPTOR_FIELD
#undef ADAFRUIT_ZERODMA_BTCTRL_VALID
#undef ADAFRUIT_ZERODMA_BTCTRL_SRCINC
#undef ADAFRUIT_ZERODMA_BTCTRL_DSTINC
#undef ADAFRUIT_ZERODMA_BTCTRL_STEPSEL
/// @endcond

// Adapted from ASF3 interrupt_sam_nvic.c:

static volatile unsigned long cpu_irq_critical_section_counter = 0;
static volatile unsigned char cpu_irq_prev_interrupt_state = 0;

static void cpu_irq_enter_critical(void) {
  if (!cpu_irq_critical_section_counter) {
    if (__get_PRIMASK() == 0) { // IRQ enabled?
      __disable_irq();          // Disable it
      __DMB();
      cpu_irq_prev_interrupt_state = 1;
    } else {
      // Make sure the to save the prev state as false
      cpu_irq_prev_interrupt_state = 0;
    }
  }

  cpu_irq_critical_section_counter++;
}

static void cpu_irq_leave_critical(void) {
  // Check if the user is trying to leave a critical section
  // when not in a critical section
  if (cpu_irq_critical_section_counter > 0) {
    cpu_irq_critical_section_counter--;

    // Only enable global interrupts when the counter
    // reaches 0 and the state of the global interrupt flag
    // was enabled when entering critical state */
    if ((!cpu_irq_critical_section_counter) && cpu_irq_prev_interrupt_state) {
      __DMB();
      __enable_irq();
    }
  }
}

// CONSTRUCTOR -------------------------------------------------------------

// Constructor initializes Adafruit_ZeroDMA basics but does NOT allocate a
// DMA channel (that's done in allocate()) or start a job (that's done in
// startJob()).  This is because constructors in a global context are called
// before a sketch's setup() function, which may have some other hardware
// initialization of its own, don't want it clobbering us.
Adafruit_ZeroDMA::Adafruit_ZeroDMA(void) {
  channel = 0xFF; // Channel not yet allocated
  jobStatus = DMA_STATUS_OK;
  hasDescriptors = false; // No descriptors allocated yet
  loopFlag = false;
  peripheralTrigger = 0; // Software trigger only by default
  triggerAction = DMA_TRIGGER_ACTON_TRANSACTION;
  memset(callback, 0, sizeof(callback));
}

// TODO: add destructor? Should stop job, delete descriptors, free channel.

// INTERRUPT SERVICE ROUTINE -----------------------------------------------

/*!
    @brief  This is a C function that exists outside the Adafruit_ZeroDMA
            context. DMA channel number is determined from the INTPEND
            register, from this we get a ZeroDMA object pointer through the
            _dmaPtr[] array. (It's done this way because jobStatus and
            callback[] are protected elements in the ZeroDMA object -- we
            can't touch them in C, but the next function after this, being
            part of the ZeroDMA class, can.)
*/
extern "C" {
#ifdef ADAFRUIT_ZERODMA_HAS_DMAC_CHANNELS
void DMAC_0_Handler(void) {
#else
void DMAC_Handler(void) {
#endif
  cpu_irq_enter_critical();

#ifdef ADAFRUIT_ZERODMA_HAS_DMAC_CHANNELS
  uint8_t channel =
      ADAFRUIT_ZERODMA_DMAC_INTPEND_ID(); // Channel # causing interrupt
#else
  uint8_t channel = DMAC->INTPEND.bit.ID; // Channel # causing interrupt
#endif
  if (channel < DMAC_CH_NUM) {
    Adafruit_ZeroDMA *dma;
    if ((dma = _dmaPtr[channel])) { // -> Channel's ZeroDMA object
#ifdef ADAFRUIT_ZERODMA_HAS_DMAC_CHANNELS
      // Call IRQ handler with channel #
      dma->_IRQhandler(channel);
#else
      DMAC->CHID.bit.ID = channel;
      // Call IRQ handler with interrupt flag(s)
      dma->_IRQhandler(DMAC->CHINTFLAG.reg);
#endif
    }
  }

  cpu_irq_leave_critical();
}

#ifdef ADAFRUIT_ZERODMA_HAS_DMAC_CHANNELS
// These must be strong definitions. The SAME5x Arduino core also supplies
// weak Dummy_Handler aliases; weak aliases here make vector resolution depend
// on archive link order and can send active DMA IRQs to Dummy_Handler.
void DMAC_1_Handler(void) { DMAC_0_Handler(); }
void DMAC_2_Handler(void) { DMAC_0_Handler(); }
void DMAC_3_Handler(void) { DMAC_0_Handler(); }
#if defined(__SAME53__) || defined(__SAME54__)
void DMAC_OTHER_Handler(void) { DMAC_0_Handler(); }
#else
void DMAC_4_Handler(void) { DMAC_0_Handler(); }
#endif
#endif
}

void Adafruit_ZeroDMA::_IRQhandler(uint8_t flags) {
#ifdef ADAFRUIT_ZERODMA_HAS_DMAC_CHANNELS
  // 'flags' is initially passed in as channel number,
  // from which we look up the actual interrupt flags...
#ifdef ADAFRUIT_ZERODMA_HAS_DMAC_REGS
  flags = ADAFRUIT_ZERODMA_DMAC_CHANNEL(flags).DMAC_CHINTFLAG;
#else
  flags = ADAFRUIT_ZERODMA_DMAC_CHANNEL(flags).CHINTFLAG.reg;
#endif
#endif
  if (flags & ADAFRUIT_ZERODMA_CHINT_TERR) {
    // Clear error flag
#ifdef ADAFRUIT_ZERODMA_HAS_DMAC_CHANNELS
#ifdef ADAFRUIT_ZERODMA_HAS_DMAC_REGS
    ADAFRUIT_ZERODMA_DMAC_CHANNEL(channel).DMAC_CHINTFLAG =
        ADAFRUIT_ZERODMA_CHINT_TERR;
#else
    ADAFRUIT_ZERODMA_DMAC_CHANNEL(channel).CHINTFLAG.reg =
        ADAFRUIT_ZERODMA_CHINT_TERR;
#endif
#else
    DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TERR;
#endif
    jobStatus = DMA_STATUS_ERR_IO;
    if (callback[DMA_CALLBACK_TRANSFER_ERROR])
      callback[DMA_CALLBACK_TRANSFER_ERROR](this);
  } else if (flags & ADAFRUIT_ZERODMA_CHINT_TCMPL) {
    // Clear transfer complete flag
#ifdef ADAFRUIT_ZERODMA_HAS_DMAC_CHANNELS
#ifdef ADAFRUIT_ZERODMA_HAS_DMAC_REGS
    ADAFRUIT_ZERODMA_DMAC_CHANNEL(channel).DMAC_CHINTFLAG =
        ADAFRUIT_ZERODMA_CHINT_TCMPL;
#else
    ADAFRUIT_ZERODMA_DMAC_CHANNEL(channel).CHINTFLAG.reg =
        ADAFRUIT_ZERODMA_CHINT_TCMPL;
#endif
#else
    DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TCMPL;
#endif
    jobStatus = DMA_STATUS_OK;
    if (callback[DMA_CALLBACK_TRANSFER_DONE])
      callback[DMA_CALLBACK_TRANSFER_DONE](this);
  } else if (flags & ADAFRUIT_ZERODMA_CHINT_SUSP) {
    // Clear channel suspend flag
#ifdef ADAFRUIT_ZERODMA_HAS_DMAC_CHANNELS
#ifdef ADAFRUIT_ZERODMA_HAS_DMAC_REGS
    ADAFRUIT_ZERODMA_DMAC_CHANNEL(channel).DMAC_CHINTFLAG =
        ADAFRUIT_ZERODMA_CHINT_SUSP;
#else
    ADAFRUIT_ZERODMA_DMAC_CHANNEL(channel).CHINTFLAG.reg =
        ADAFRUIT_ZERODMA_CHINT_SUSP;
#endif
#else
    DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_SUSP;
#endif
    jobStatus = DMA_STATUS_SUSPEND;
    if (callback[DMA_CALLBACK_CHANNEL_SUSPEND])
      callback[DMA_CALLBACK_CHANNEL_SUSPEND](this);
  }
}

// DMA CHANNEL FUNCTIONS ---------------------------------------------------

// Allocates channel for ZeroDMA object
ZeroDMAstatus Adafruit_ZeroDMA::allocate(void) {

  if (channel < DMAC_CH_NUM)
    return DMA_STATUS_OK; // Already alloc'd!

  // Find index of first free DMA channel.  As currently written,
  // this "does not play well with others" as it assumes _channelMask
  // is the final arbiter of channels in use (this is true only within
  // this library -- but other DMA-driven code may have allocated its
  // own channel(s) elsewhere, sometimes with an equally broken
  // approach).  A possible alternate approach, I haven't tested this
  // yet, might be to loop through each channel, set DMAC->CHID.bit.ID
  // and then test whether CHCTRLA.bit.ENABLE is set?  But for now...
  for (channel = 0; (channel < DMAC_CH_NUM) && (_channelMask & (1 << channel));
       channel++)
    ;
  // Doesn't help that code later does a software reset of the DMA
  // controller, which would blow out other DMA-using libraries
  // anyway (or they're just as likely to blow out this one).
  // I think it's just an all-or-nothing affair...use one library
  // for DMA everything, never mix and match.

  if (channel >= DMAC_CH_NUM) // No free channel!
    return DMA_STATUS_ERR_NOT_FOUND;

  cpu_irq_enter_critical();

  if (!_channelMask) { // No channels allocated yet; initialize DMA!
#if !defined(DMAC_RESERVED_CHANNELS)
#if (SAML21) || (SAML22) || (SAMC20) || (SAMC21)
    PM->AHBMASK.bit.DMAC_ = 1;
#elif defined(__SAMD51__) || defined(__SAME51__)
    MCLK->AHBMASK.bit.DMAC_ = 1; // Initialize DMA clocks
#elif defined(ADAFRUIT_ZERODMA_HAS_DMAC_REGS)
    MCLK_REGS->MCLK_AHBMASK |= MCLK_AHBMASK_DMAC_Msk; // Initialize DMA clocks
#else
    PM->AHBMASK.bit.DMAC_ = 1; // Initialize DMA clocks
    PM->APBBMASK.bit.DMAC_ = 1;
#endif
#ifdef ADAFRUIT_ZERODMA_HAS_DMAC_REGS
    DMAC_REGS->DMAC_CTRL &= ~DMAC_CTRL_DMAENABLE_Msk; // Disable DMA controller
    DMAC_REGS->DMAC_CTRL |= DMAC_CTRL_SWRST_Msk;      // Perform software reset

    // Initialize descriptor list addresses
    DMAC_REGS->DMAC_BASEADDR = (uint32_t)_descriptor;
    DMAC_REGS->DMAC_WRBADDR = (uint32_t)_writeback;
#else
    DMAC->CTRL.bit.DMAENABLE = 0; // Disable DMA controller
    DMAC->CTRL.bit.SWRST = 1;     // Perform software reset

    // Initialize descriptor list addresses
    DMAC->BASEADDR.bit.BASEADDR = (uint32_t)_descriptor;
    DMAC->WRBADDR.bit.WRBADDR = (uint32_t)_writeback;
#endif
    memset(_descriptor, 0, sizeof(_descriptor));
    memset(_writeback, 0, sizeof(_writeback));

    // Re-enable DMA controller with all priority levels
#ifdef ADAFRUIT_ZERODMA_HAS_DMAC_REGS
    DMAC_REGS->DMAC_CTRL = DMAC_CTRL_DMAENABLE_Msk | DMAC_CTRL_LVLEN(0xF);
#else
    DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xF);
#endif
#endif

    // Enable DMA interrupt at lowest priority
#ifdef ADAFRUIT_ZERODMA_HAS_DMAC_CHANNELS
    IRQn_Type irqs[] = {DMAC_0_IRQn, DMAC_1_IRQn, DMAC_2_IRQn, DMAC_3_IRQn,
#ifdef ADAFRUIT_ZERODMA_HAS_DMAC_REGS
                        DMAC_OTHER_IRQn
#else
                        DMAC_4_IRQn
#endif
    };
    for (uint8_t i = 0; i < (sizeof irqs / sizeof irqs[0]); i++) {
      NVIC_EnableIRQ(irqs[i]);
      NVIC_SetPriority(irqs[i], (1 << __NVIC_PRIO_BITS) - 1);
    }
#else
    NVIC_EnableIRQ(DMAC_IRQn);
    NVIC_SetPriority(DMAC_IRQn, (1 << __NVIC_PRIO_BITS) - 1);
#endif
  }

  _channelMask |= 1 << channel; // Mark channel as allocated
  _dmaPtr[channel] = this;      // Channel-index-to-object pointer

  // Reset the allocated channel
#ifdef ADAFRUIT_ZERODMA_HAS_DMAC_REGS
  ADAFRUIT_ZERODMA_DMAC_CHANNEL(channel).DMAC_CHCTRLA &=
      ~DMAC_CHCTRLA_ENABLE_Msk;
  ADAFRUIT_ZERODMA_DMAC_CHANNEL(channel).DMAC_CHCTRLA |= DMAC_CHCTRLA_SWRST_Msk;
#elif defined(ADAFRUIT_ZERODMA_HAS_DMAC_CHANNELS)
  DMAC->Channel[channel].CHCTRLA.bit.ENABLE = 0;
  DMAC->Channel[channel].CHCTRLA.bit.SWRST = 1;
#else
  DMAC->CHID.bit.ID = channel;
  DMAC->CHCTRLA.bit.ENABLE = 0;
  DMAC->CHCTRLA.bit.SWRST = 1;
#endif

  // Clear software trigger
#ifdef ADAFRUIT_ZERODMA_HAS_DMAC_REGS
  DMAC_REGS->DMAC_SWTRIGCTRL &= ~(1 << channel);
#else
  DMAC->SWTRIGCTRL.reg &= ~(1 << channel);
#endif

  // Configure default behaviors
#ifdef ADAFRUIT_ZERODMA_HAS_DMAC_REGS
  ADAFRUIT_ZERODMA_DMAC_CHANNEL(channel).DMAC_CHPRILVL =
      DMAC_CHPRILVL_PRILVL(0);
  ADAFRUIT_ZERODMA_DMAC_CHANNEL(channel).DMAC_CHCTRLA =
      (ADAFRUIT_ZERODMA_DMAC_CHANNEL(channel).DMAC_CHCTRLA &
       ~(DMAC_CHCTRLA_TRIGSRC_Msk | DMAC_CHCTRLA_TRIGACT_Msk |
         DMAC_CHCTRLA_BURSTLEN_Msk)) |
      DMAC_CHCTRLA_TRIGSRC(peripheralTrigger) |
      DMAC_CHCTRLA_TRIGACT(triggerAction) |
      DMAC_CHCTRLA_BURSTLEN(DMAC_CHCTRLA_BURSTLEN_SINGLE_Val);
#elif defined(ADAFRUIT_ZERODMA_HAS_DMAC_CHANNELS)
  DMAC->Channel[channel].CHPRILVL.bit.PRILVL = 0;
  DMAC->Channel[channel].CHCTRLA.bit.TRIGSRC = peripheralTrigger;
  DMAC->Channel[channel].CHCTRLA.bit.TRIGACT = triggerAction;
  DMAC->Channel[channel].CHCTRLA.bit.BURSTLEN =
      DMAC_CHCTRLA_BURSTLEN_SINGLE_Val; // Single-beat burst length
#else
  DMAC->CHCTRLB.bit.LVL = 0;
  DMAC->CHCTRLB.bit.TRIGSRC = peripheralTrigger;
  DMAC->CHCTRLB.bit.TRIGACT = triggerAction;
#endif

  cpu_irq_leave_critical();

  return DMA_STATUS_OK;
}

void Adafruit_ZeroDMA::setPriority(dma_priority pri) {
#ifdef ADAFRUIT_ZERODMA_HAS_DMAC_REGS
  ADAFRUIT_ZERODMA_DMAC_CHANNEL(channel).DMAC_CHPRILVL =
      (ADAFRUIT_ZERODMA_DMAC_CHANNEL(channel).DMAC_CHPRILVL &
       ~DMAC_CHPRILVL_PRILVL_Msk) |
      DMAC_CHPRILVL_PRILVL(pri);
#elif defined(ADAFRUIT_ZERODMA_HAS_DMAC_CHANNELS)
  DMAC->Channel[channel].CHPRILVL.bit.PRILVL = pri;
#else
  DMAC->CHCTRLB.bit.LVL = pri;
#endif
}

// Deallocate DMA channel
// TODO: should this delete/deallocate the descriptor list?
ZeroDMAstatus Adafruit_ZeroDMA::free(void) {

  ZeroDMAstatus status = DMA_STATUS_OK;

  cpu_irq_enter_critical(); // jobStatus is volatile

  if (ADAFRUIT_ZERODMA_DMAC_CHANNEL_BUSY(channel)) {
    status = DMA_STATUS_BUSY; // Can't leave when busy
  } else if ((channel < DMAC_CH_NUM) && (_channelMask & (1 << channel))) {
    // Valid in-use channel; release it
    _channelMask &= ~(1 << channel); // Clear bit
    if (!_channelMask) {             // No more channels in use?
#ifdef ADAFRUIT_ZERODMA_HAS_DMAC_CHANNELS
      NVIC_DisableIRQ(DMAC_0_IRQn); // Disable DMA interrupt
      NVIC_DisableIRQ(DMAC_1_IRQn);
      NVIC_DisableIRQ(DMAC_2_IRQn);
      NVIC_DisableIRQ(DMAC_3_IRQn);
#ifdef ADAFRUIT_ZERODMA_HAS_DMAC_REGS
      NVIC_DisableIRQ(DMAC_OTHER_IRQn);
#else
      NVIC_DisableIRQ(DMAC_4_IRQn);
#endif
#ifdef ADAFRUIT_ZERODMA_HAS_DMAC_REGS
      DMAC_REGS->DMAC_CTRL &= ~DMAC_CTRL_DMAENABLE_Msk;  // Disable DMA
      MCLK_REGS->MCLK_AHBMASK &= ~MCLK_AHBMASK_DMAC_Msk; // Disable DMA clock
#else
      DMAC->CTRL.bit.DMAENABLE = 0; // Disable DMA
      MCLK->AHBMASK.bit.DMAC_ = 0;  // Disable DMA clock
#endif
#else
      NVIC_DisableIRQ(DMAC_IRQn);   // Disable DMA interrupt
      DMAC->CTRL.bit.DMAENABLE = 0; // Disable DMA
      PM->APBBMASK.bit.DMAC_ = 0;   // Disable DMA clocks
      PM->AHBMASK.bit.DMAC_ = 0;
#endif
    }
    _dmaPtr[channel] = NULL;
    channel = 0xFF;
  } else {
    status = DMA_STATUS_ERR_NOT_INITIALIZED; // Channel not in use
  }

  cpu_irq_leave_critical();

  return status;
}

// Start DMA transfer job.  Channel and descriptors should be allocated
// before calling this.
ZeroDMAstatus Adafruit_ZeroDMA::startJob(void) {
  ZeroDMAstatus status = DMA_STATUS_OK;

  cpu_irq_enter_critical(); // Job status is volatile

  if (ADAFRUIT_ZERODMA_DMAC_CHANNEL_BUSY(channel)) {
    status = DMA_STATUS_BUSY; // Resource is busy
  } else if (channel >= DMAC_CH_NUM) {
    status = DMA_STATUS_ERR_NOT_INITIALIZED; // Channel not in use
  } else if (!hasDescriptors || (descriptorBtcnt(&_descriptor[channel]) <= 0)) {
    status = DMA_STATUS_ERR_INVALID_ARG; // Bad transfer size
  } else {
    uint8_t i, interruptMask = 0;
    for (i = 0; i < DMA_CALLBACK_N; i++)
      if (callback[i])
        interruptMask |= (1 << i);
    jobStatus = DMA_STATUS_BUSY;
#ifdef ADAFRUIT_ZERODMA_HAS_DMAC_REGS
    ADAFRUIT_ZERODMA_DMAC_CHANNEL(channel).DMAC_CHINTENSET =
        ADAFRUIT_ZERODMA_CHINT_MASK & interruptMask;
    ADAFRUIT_ZERODMA_DMAC_CHANNEL(channel).DMAC_CHINTENCLR =
        ADAFRUIT_ZERODMA_CHINT_MASK & ~interruptMask;
    ADAFRUIT_ZERODMA_DMAC_CHANNEL(channel).DMAC_CHCTRLA |=
        DMAC_CHCTRLA_ENABLE_Msk;
#elif defined(ADAFRUIT_ZERODMA_HAS_DMAC_CHANNELS)
    DMAC->Channel[channel].CHINTENSET.reg =
        DMAC_CHINTENSET_MASK & interruptMask;
    DMAC->Channel[channel].CHINTENCLR.reg =
        DMAC_CHINTENCLR_MASK & ~interruptMask;
    DMAC->Channel[channel].CHCTRLA.bit.ENABLE = 1;
#else
    DMAC->CHID.bit.ID = channel;
    DMAC->CHINTENSET.reg = DMAC_CHINTENSET_MASK & interruptMask;
    DMAC->CHINTENCLR.reg = DMAC_CHINTENCLR_MASK & ~interruptMask;
    DMAC->CHCTRLA.bit.ENABLE = 1; // Enable the transfer channel
#endif
  }

  cpu_irq_leave_critical();

  return status;
}

// Set and enable callback function for ZeroDMA object. This can be called
// before or after channel and/or descriptors are allocated, but needs
// to be called before job is started.
void Adafruit_ZeroDMA::setCallback(void (*cb)(Adafruit_ZeroDMA *),
                                   dma_callback_type type) {
  callback[type] = cb;
}

// Suspend/resume don't quite do what I thought -- avoid using for now.
void Adafruit_ZeroDMA::suspend(void) {
  cpu_irq_enter_critical();
#ifdef ADAFRUIT_ZERODMA_HAS_DMAC_REGS
  ADAFRUIT_ZERODMA_DMAC_CHANNEL(channel).DMAC_CHCTRLB |=
      DMAC_CHCTRLB_CMD_SUSPEND;
#elif defined(ADAFRUIT_ZERODMA_HAS_DMAC_CHANNELS)
  DMAC->Channel[channel].CHCTRLB.reg |= DMAC_CHCTRLB_CMD_SUSPEND;
#else
  DMAC->CHID.bit.ID = channel;
  DMAC->CHCTRLB.reg |= DMAC_CHCTRLB_CMD_SUSPEND;
#endif
  cpu_irq_leave_critical();
}

#define MAX_JOB_RESUME_COUNT 10000 ///< Loop iteration threshold for timeout
void Adafruit_ZeroDMA::resume(void) {
  cpu_irq_enter_critical(); // jobStatus is volatile
  if (jobStatus == DMA_STATUS_SUSPEND) {
    int count;
    uint32_t bitMask = 1 << channel;
#ifdef ADAFRUIT_ZERODMA_HAS_DMAC_REGS
    ADAFRUIT_ZERODMA_DMAC_CHANNEL(channel).DMAC_CHCTRLB |=
        DMAC_CHCTRLB_CMD_RESUME;
#elif defined(ADAFRUIT_ZERODMA_HAS_DMAC_CHANNELS)
    DMAC->Channel[channel].CHCTRLB.reg |= DMAC_CHCTRLB_CMD_RESUME;
#else
    DMAC->CHID.bit.ID = channel;
    DMAC->CHCTRLB.reg |= DMAC_CHCTRLB_CMD_RESUME;
#endif

    for (count = 0; (count < MAX_JOB_RESUME_COUNT) &&
                    !(ADAFRUIT_ZERODMA_DMAC_BUSYCH() & bitMask);
         count++)
      ;

    jobStatus = (count < MAX_JOB_RESUME_COUNT) ? DMA_STATUS_BUSY
                                               : DMA_STATUS_ERR_TIMEOUT;
  }
  cpu_irq_leave_critical();
}

// Abort is OK though.
void Adafruit_ZeroDMA::abort(void) {
  if (channel <= DMAC_CH_NUM) {
    cpu_irq_enter_critical();
#ifdef ADAFRUIT_ZERODMA_HAS_DMAC_REGS
    ADAFRUIT_ZERODMA_DMAC_CHANNEL(channel).DMAC_CHCTRLA &=
        ~DMAC_CHCTRLA_ENABLE_Msk; // Disable channel
    ADAFRUIT_ZERODMA_DMAC_CHANNEL(channel).DMAC_CHCTRLA |=
        DMAC_CHCTRLA_SWRST_Msk;
    while (ADAFRUIT_ZERODMA_DMAC_CHANNEL(channel).DMAC_CHCTRLA &
           DMAC_CHCTRLA_SWRST_Msk)
      ;
    ADAFRUIT_ZERODMA_DMAC_CHANNEL(channel).DMAC_CHCTRLA =
        DMAC_CHCTRLA_TRIGSRC(peripheralTrigger) |
        DMAC_CHCTRLA_TRIGACT(triggerAction);
#elif defined(ADAFRUIT_ZERODMA_HAS_DMAC_CHANNELS)
    DMAC->Channel[channel].CHCTRLA.bit.ENABLE = 0; // Disable channel
    DMAC->Channel[channel].CHCTRLA.bit.SWRST = 1;
    while (DMAC->Channel[channel].CHCTRLA.bit.SWRST)
      ;
    DMAC->Channel[channel].CHCTRLA.bit.TRIGSRC = peripheralTrigger;
    DMAC->Channel[channel].CHCTRLA.bit.TRIGACT = triggerAction;
#else
    DMAC->CHID.bit.ID = channel; // Select channel
    DMAC->CHCTRLA.reg = 0;       // Disable
    DMAC->CHCTRLA.bit.SWRST = 1;
    while (DMAC->CHCTRLA.bit.SWRST)
      ;
    DMAC->CHCTRLB.bit.TRIGSRC = peripheralTrigger;
    DMAC->CHCTRLB.bit.TRIGACT = triggerAction;
#endif
    // A channel aborted from SUSPEND can remain pending but never arbitrate
    // after it is merely disabled and re-enabled. SWRST clears that internal
    // state; the cached trigger configuration reconstructs the allocated
    // channel for its next job.
    jobStatus = DMA_STATUS_ABORTED;
    cpu_irq_leave_critical();
  }
}

// Set DMA peripheral trigger.
// This can be done before or after channel is allocated.
void Adafruit_ZeroDMA::setTrigger(uint8_t trigger) {
  peripheralTrigger = trigger; // Save value for allocate()

  // If channel already allocated, configure peripheral trigger
  // (old lib required configure before alloc -- either way OK now)
  if (channel < DMAC_CH_NUM) {
    cpu_irq_enter_critical();
#ifdef ADAFRUIT_ZERODMA_HAS_DMAC_REGS
    ADAFRUIT_ZERODMA_DMAC_CHANNEL(channel).DMAC_CHCTRLA =
        (ADAFRUIT_ZERODMA_DMAC_CHANNEL(channel).DMAC_CHCTRLA &
         ~DMAC_CHCTRLA_TRIGSRC_Msk) |
        DMAC_CHCTRLA_TRIGSRC(trigger);
#elif defined(ADAFRUIT_ZERODMA_HAS_DMAC_CHANNELS)
    DMAC->Channel[channel].CHCTRLA.bit.TRIGSRC = trigger;
#else
    DMAC->CHID.bit.ID = channel;
    DMAC->CHCTRLB.bit.TRIGSRC = trigger;
#endif
    cpu_irq_leave_critical();
  }
}

// Set DMA trigger action.
// This can be done before or after channel is allocated.
void Adafruit_ZeroDMA::setAction(dma_transfer_trigger_action action) {
  triggerAction = action; // Save value for allocate()

  // If channel already allocated, configure trigger action
  // (old lib required configure before alloc -- either way OK now)
  if (channel < DMAC_CH_NUM) {
    cpu_irq_enter_critical();
#ifdef ADAFRUIT_ZERODMA_HAS_DMAC_REGS
    ADAFRUIT_ZERODMA_DMAC_CHANNEL(channel).DMAC_CHCTRLA =
        (ADAFRUIT_ZERODMA_DMAC_CHANNEL(channel).DMAC_CHCTRLA &
         ~DMAC_CHCTRLA_TRIGACT_Msk) |
        DMAC_CHCTRLA_TRIGACT(action);
#elif defined(ADAFRUIT_ZERODMA_HAS_DMAC_CHANNELS)
    DMAC->Channel[channel].CHCTRLA.bit.TRIGACT = action;
#else
    DMAC->CHID.bit.ID = channel;
    DMAC->CHCTRLB.bit.TRIGACT = action;
#endif
    cpu_irq_leave_critical();
  }
}

// Issue software trigger. Channel must be allocated & descriptors added!
void Adafruit_ZeroDMA::trigger(void) {
  if ((channel <= DMAC_CH_NUM) & hasDescriptors)
#ifdef ADAFRUIT_ZERODMA_HAS_DMAC_REGS
    DMAC_REGS->DMAC_SWTRIGCTRL |= (1 << channel);
#else
    DMAC->SWTRIGCTRL.reg |= (1 << channel);
#endif
}

uint8_t Adafruit_ZeroDMA::getChannel(void) { return channel; }

// DMA DESCRIPTOR FUNCTIONS ------------------------------------------------

// Allocates a new DMA descriptor (if needed) and appends it to the
// channel's descriptor list.  Returns pointer to DmacDescriptor,
// or NULL on various errors.  You'll want to keep the pointer for
// later if you need to modify or free the descriptor.
// Channel must be allocated first!
DmacDescriptor *Adafruit_ZeroDMA::addDescriptor(void *src, void *dst,
                                                uint32_t count,
                                                dma_beat_size size, bool srcInc,
                                                bool dstInc, uint32_t stepSize,
                                                bool stepSel) {

  // Channel must be allocated first
  if (channel >= DMAC_CH_NUM)
    return NULL;

  // Can't do while job's busy
  if (ADAFRUIT_ZERODMA_DMAC_CHANNEL_BUSY(channel))
    return NULL;

  DmacDescriptor *desc;

  // Scan descriptor list to find last entry.  If an entry's
  // DESCADDR value is 0, that's the end of the list and it's
  // currently un-looped.  If the DESCADDR value is the same
  // as the first entry, that's the end of the list and it's
  // looped.  Either way, set the last entry's DESCADDR value
  // to the new descriptor, and the descriptor's own DESCADDR
  // will be set later either to 0 or the list head.
  if (hasDescriptors) {
    // DMA descriptors must be 128-bit (16 byte) aligned.
    // memalign() is considered 'obsolete' but it's replacements
    // (aligned_alloc() or posix_memalign()) are not currently
    // available in the version of ARM GCC in use, but this is,
    // so here we are.
    if (!(desc = (DmacDescriptor *)memalign(16, sizeof(DmacDescriptor))))
      return NULL;
    DmacDescriptor *prev = &_descriptor[channel];
    while (descriptorDescaddr(prev) &&
           (descriptorDescaddr(prev) != (uint32_t)&_descriptor[channel])) {
      prev = (DmacDescriptor *)descriptorDescaddr(prev);
    }
    descriptorSetDescaddr(prev, (uint32_t)desc);
  } else {
    desc = &_descriptor[channel];
  }
  hasDescriptors = true;

  uint8_t bytesPerBeat; // Beat transfer size IN BYTES
  switch (size) {
  default:
    bytesPerBeat = 1;
    break;
  case DMA_BEAT_SIZE_HWORD:
    bytesPerBeat = 2;
    break;
  case DMA_BEAT_SIZE_WORD:
    bytesPerBeat = 4;
    break;
  }

  descriptorSetBtctrl(
      desc, descriptorBuildBtctrl(size, srcInc, dstInc, stepSel, stepSize));
  descriptorSetBtcnt(desc, count);
  descriptorSetSrcaddr(desc, (uint32_t)src);

  if (srcInc) {
    if (stepSel) {
      descriptorSetSrcaddr(desc, descriptorSrcaddr(desc) +
                                     bytesPerBeat * count * (1 << stepSize));
    } else {
      descriptorSetSrcaddr(desc,
                           descriptorSrcaddr(desc) + bytesPerBeat * count);
    }
  }

  descriptorSetDstaddr(desc, (uint32_t)dst);

  if (dstInc) {
    if (!stepSel) {
      descriptorSetDstaddr(desc, descriptorDstaddr(desc) +
                                     bytesPerBeat * count * (1 << stepSize));
    } else {
      descriptorSetDstaddr(desc,
                           descriptorDstaddr(desc) + bytesPerBeat * count);
    }
  }

  descriptorSetDescaddr(desc, loopFlag ? (uint32_t)&_descriptor[channel] : 0);

  return desc;
}

// Modify DMA descriptor with a new source address, destination address &
// block transfer count.  All other attributes (including increment enables,
// etc.) are unchanged.  Mostly for changing the data being pushed to a
// peripheral (DAC, SPI, whatev.)
void Adafruit_ZeroDMA::changeDescriptor(DmacDescriptor *desc, void *src,
                                        void *dst, uint32_t count) {

  uint8_t bytesPerBeat; // Beat transfer size IN BYTES
  switch (descriptorBeatSize(desc)) {
  default:
    bytesPerBeat = 1;
    break;
  case DMA_BEAT_SIZE_HWORD:
    bytesPerBeat = 2;
    break;
  case DMA_BEAT_SIZE_WORD:
    bytesPerBeat = 4;
    break;
  }

  if (count)
    descriptorSetBtcnt(desc, count);

  if (src) {
    descriptorSetSrcaddr(desc, (uint32_t)src);
    if (descriptorSrcInc(desc)) {
      if (descriptorStepSel(desc)) {
        descriptorSetSrcaddr(desc, descriptorSrcaddr(desc) +
                                       descriptorBtcnt(desc) * bytesPerBeat *
                                           (1 << descriptorStepSize(desc)));
      } else {
        descriptorSetSrcaddr(desc, descriptorSrcaddr(desc) +
                                       descriptorBtcnt(desc) * bytesPerBeat);
      }
    }
  }

  if (dst) {
    descriptorSetDstaddr(desc, (uint32_t)dst);
    if (descriptorDstInc(desc)) {
      if (!descriptorStepSel(desc)) {
        descriptorSetDstaddr(desc, descriptorDstaddr(desc) +
                                       descriptorBtcnt(desc) * bytesPerBeat *
                                           (1 << descriptorStepSize(desc)));
      } else {
        descriptorSetDstaddr(desc, descriptorDstaddr(desc) +
                                       descriptorBtcnt(desc) * bytesPerBeat);
      }
    }
  }

// I think this code is here by accident -- disabling for now.
#if 0
	cpu_irq_enter_critical();
	jobStatus          = DMA_STATUS_OK;
#if defined(ADAFRUIT_ZERODMA_HAS_DMAC_CHANNELS)
	DMAC->Channel[channel].CHCTRLA.bit.ENABLE = 1;
#else
	DMAC->CHID.bit.ID  = channel;
	DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
#endif
	cpu_irq_leave_critical();
#endif
}

// TODO: delete descriptor, delete whole descriptor chain

// Select whether channel's descriptor list should repeat or not.
// This can be done before or after channel & any descriptors are allocated.
void Adafruit_ZeroDMA::loop(boolean flag) {
  // The loop selection is 'sticky' -- that is, you can enable or
  // disable looping before a descriptor list is built, or after
  // the fact.  This requires some extra steps in the library code
  // but avoids a must-do-in-X-order constraint on user.
  loopFlag = flag;

  if (hasDescriptors) { // Descriptor list already started?
    // Scan descriptor list to find last entry.  If an entry's
    // DESCADDR value is 0, that's the end of the list and it's
    // currently un-looped.  If the DESCADDR value is the same
    // as the first entry, that's the end of the list and it's
    // already looped.
    DmacDescriptor *desc = &_descriptor[channel];
    while (descriptorDescaddr(desc) &&
           (descriptorDescaddr(desc) != (uint32_t)&_descriptor[channel])) {
      desc = (DmacDescriptor *)descriptorDescaddr(desc);
    }
    // Loop or unloop descriptor list as appropriate
    descriptorSetDescaddr(desc, loopFlag ? (uint32_t)&_descriptor[channel] : 0);
  }
}

// MISCELLANY --------------------------------------------------------------

void Adafruit_ZeroDMA::printStatus(ZeroDMAstatus s) {
  if (s == DMA_STATUS_JOBSTATUS)
    s = jobStatus;
  Serial.print("Status: ");
  switch (s) {
  case DMA_STATUS_OK:
    Serial.println("OK");
    break;
  case DMA_STATUS_ERR_NOT_FOUND:
    Serial.println("NOT FOUND");
    break;
  case DMA_STATUS_ERR_NOT_INITIALIZED:
    Serial.println("NOT INITIALIZED");
    break;
  case DMA_STATUS_ERR_INVALID_ARG:
    Serial.println("INVALID ARGUMENT");
    break;
  case DMA_STATUS_ERR_IO:
    Serial.println("IO ERROR");
    break;
  case DMA_STATUS_ERR_TIMEOUT:
    Serial.println("TIMEOUT");
    break;
  case DMA_STATUS_BUSY:
    Serial.println("BUSY");
    break;
  case DMA_STATUS_SUSPEND:
    Serial.println("SUSPENDED");
    break;
  case DMA_STATUS_ABORTED:
    Serial.println("ABORTED");
    break;
  default:
    Serial.print("Unknown 0x");
    Serial.println((int)s);
    break;
  }
}

bool Adafruit_ZeroDMA::isActive() {
  return descriptorValid(&_writeback[channel]);
}
