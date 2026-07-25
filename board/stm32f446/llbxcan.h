#include "llbxcan_declarations.h"

// kbps multiplied by 10
const uint32_t speeds[SPEEDS_ARRAY_SIZE] = {100U, 200U, 500U, 1000U, 1250U, 2500U, 5000U, 10000U};
const uint32_t data_speeds[DATA_SPEEDS_ARRAY_SIZE] = {0U}; // No separate data speed, dummy

bool llcan_set_speed(CAN_TypeDef *CANx, uint32_t speed, bool loopback, bool silent) {
  bool ret = true;

  // wake the peripheral: bxCAN resets into SLEEP mode and will not receive
  // anything while SLAK is set.
  register_clear_bits(&(CANx->MCR), CAN_MCR_SLEEP);
  {
    uint32_t wake_to = 0U;
    while (((CANx->MSR & CAN_MSR_SLAK) == CAN_MSR_SLAK) && (wake_to < CAN_INIT_TIMEOUT_MS)) {
      delay(10000);
      wake_to++;
    }
  }

  // initialization mode
  register_set(&(CANx->MCR), CAN_MCR_TTCM | CAN_MCR_INRQ, 0x180FFU);
  uint32_t timeout_counter = 0U;
  while((CANx->MSR & CAN_MSR_INAK) != CAN_MSR_INAK){
    // Delay for about 1ms
    delay(10000);
    timeout_counter++;

    if(timeout_counter >= CAN_INIT_TIMEOUT_MS){
      print(CAN_NAME_FROM_CANIF(CANx)); print(" set_speed timed out (1)!\n");
      ret = false;
      break;
    }
  }

  if(ret){
    // set time quanta from defines
    register_set(&(CANx->BTR), ((CAN_BTR_TS1_0 * (CAN_SEQ1-1U)) |
                                   (CAN_BTR_TS2_0 * (CAN_SEQ2-1U)) |
                                   (CAN_BTR_SJW_0 * (CAN_SJW-1U)) |
                                   (can_speed_to_prescaler(speed) - 1U)), 0xC37F03FFU);

    // silent loopback mode for debugging
    if (loopback) {
      register_set_bits(&(CANx->BTR), CAN_BTR_SILM | CAN_BTR_LBKM);
    }
    if (silent) {
      register_set_bits(&(CANx->BTR), CAN_BTR_SILM);
    }

    // leave init mode; keep SLEEP clear (ABOM = auto bus-off recovery)
    register_set(&(CANx->MCR), CAN_MCR_TTCM | CAN_MCR_ABOM, 0x180FFU);

    timeout_counter = 0U;
    while(((CANx->MSR & CAN_MSR_INAK) == CAN_MSR_INAK)) {
      // Delay for about 1ms
      delay(10000);
      timeout_counter++;

      if(timeout_counter >= CAN_INIT_TIMEOUT_MS){
        print(CAN_NAME_FROM_CANIF(CANx)); print(" set_speed timed out (2)!\n");
        ret = false;
        break;
      }
    }
  }

  return ret;
}

void llcan_irq_disable(const CAN_TypeDef *CANx) {
  if (CANx == CAN1) {
    NVIC_DisableIRQ(CAN1_TX_IRQn);
    NVIC_DisableIRQ(CAN1_RX0_IRQn);
    NVIC_DisableIRQ(CAN1_SCE_IRQn);
  } else if (CANx == CAN2) {
    NVIC_DisableIRQ(CAN2_TX_IRQn);
    NVIC_DisableIRQ(CAN2_RX0_IRQn);
    NVIC_DisableIRQ(CAN2_SCE_IRQn);
  } else {
  }
}

void llcan_irq_enable(const CAN_TypeDef *CANx) {
  if (CANx == CAN1) {
    NVIC_EnableIRQ(CAN1_TX_IRQn);
    NVIC_EnableIRQ(CAN1_RX0_IRQn);
    NVIC_EnableIRQ(CAN1_SCE_IRQn);
  } else if (CANx == CAN2) {
    NVIC_EnableIRQ(CAN2_TX_IRQn);
    NVIC_EnableIRQ(CAN2_RX0_IRQn);
    NVIC_EnableIRQ(CAN2_SCE_IRQn);
  } else {
  }
}

bool llcan_init(CAN_TypeDef *CANx) {
  bool ret = true;

  // Enter init mode
  register_set_bits(&(CANx->FMR), CAN_FMR_FINIT);

  // Wait for INAK bit to be set
  uint32_t timeout_counter = 0U;
  while(((CANx->MSR & CAN_MSR_INAK) == CAN_MSR_INAK)) {
    // Delay for about 1ms
    delay(10000);
    timeout_counter++;

    if(timeout_counter >= CAN_INIT_TIMEOUT_MS){
      print(CAN_NAME_FROM_CANIF(CANx)); print(" initialization timed out!\n");
      ret = false;
      break;
    }
  }

  if(ret){
#ifdef MAZDA_FILTER
    // ---- Hardware acceptance filter: only the IDs opendbc's Mazda port uses ----
    // The CX-5 bus carries ~743 distinct IDs at ~2-3k frames/s, which saturates
    // the 1.5 Mbaud serial link and overflows can_rx_q (observed 1.29M drops).
    // Filtering in hardware cuts that to the 17 messages carstate.py reads.
    //
    // 16-bit LIST mode: each 32-bit register holds TWO 11-bit IDs at bits 15:5.
    // 5 banks x 4 IDs = 20 slots; we use 17 (pad with a repeat of the first).
    static const uint16_t mazda_ids[20] = {
      0x078U, // BRAKE
      0x082U, // STEER
      0x09AU, // BLINK_INFO
      0x09DU, // CRZ_BTNS
      0x165U, // PEDALS
      0x202U, // ENGINE_DATA
      0x215U, // WHEEL_SPEEDS
      0x21CU, // CRZ_CTRL
      0x21FU, // CRZ_EVENTS
      0x228U, // GEAR
      0x240U, // STEER_TORQUE
      0x241U, // STEER_RATE
      0x243U, // CAM_LKAS
      0x340U, // SEATBELT
      0x43EU, // DOORS
      0x440U, // CAM_LANEINFO
      0x477U, // BSM
      0x078U, 0x078U, 0x078U  // padding (harmless duplicates)
    };

    // MASTER/SLAVE FIX (F446): CAN1+CAN2 SHARE one filter block; CAN2 (slave) has
    // NO filter regs of its own -- ALL filter access must go through CAN1, and the
    // CAN2SB field in CAN1->FMR splits the banks. The old code wrote CANx->..., so
    // for CAN2 the filter never activated -> CAN2 accepted no frames (FORM/no-rx).
    // Fix: configure through CAN1. CAN2SB=14 -> banks 0..4 = CAN1, banks 14..18 = CAN2.
    // We write the SAME 17-ID Mazda list into both bank ranges so either bus filters
    // identically. Whichever CANx we're initing, the shared config is (re)applied.
    const uint8_t base = (CANx == CAN1) ? 0U : 14U;   // this bus's filter bank range

    // filter config lives in the master (CAN1). enter filter-init on the master.
    register_set_bits(&(CAN1->FMR), CAN_FMR_FINIT);
    register_set(&(CAN1->FMR), (14UL << CAN_FMR_CAN2SB_Pos) | CAN_FMR_FINIT,
                 CAN_FMR_CAN2SB | CAN_FMR_FINIT);      // CAN2 start bank = 14

    // 16-bit scale (FS1R clear) + LIST mode (FM1R set) for this bus's 5 banks
    const uint32_t bankmask = 0x1FUL << base;
    CAN1->FS1R &= ~bankmask;
    CAN1->FM1R |= bankmask;

    for (uint8_t i = 0U; i < 5U; i++) {
      const uint8_t bank = base + i;
      const uint16_t a = mazda_ids[(i * 4U) + 0U];
      const uint16_t b = mazda_ids[(i * 4U) + 1U];
      const uint16_t c = mazda_ids[(i * 4U) + 2U];
      const uint16_t d = mazda_ids[(i * 4U) + 3U];
      // STID occupies bits 15:5 of each 16-bit half
      CAN1->sFilterRegister[bank].FR1 = ((uint32_t)(b << 5) << 16) | (uint32_t)(a << 5);
      CAN1->sFilterRegister[bank].FR2 = ((uint32_t)(d << 5) << 16) | (uint32_t)(c << 5);
    }
    CAN1->FA1R |= bankmask;                            // activate this bus's banks
    register_clear_bits(&(CAN1->FMR), CAN_FMR_FINIT);  // leave filter-init on master
#else
    // no mask
    // For some weird reason some of these registers do not want to set properly on CAN2 and CAN3. Probably something to do with the single/dual mode and their different filters.
    CANx->sFilterRegister[0].FR1 = 0U;
    CANx->sFilterRegister[0].FR2 = 0U;
    CANx->sFilterRegister[14].FR1 = 0U;
    CANx->sFilterRegister[14].FR2 = 0U;
    CANx->FA1R |= 1U | (1UL << 14);
#endif

    // Exit init mode, do not wait
    register_clear_bits(&(CANx->FMR), CAN_FMR_FINIT);

    // enable certain CAN interrupts
    register_set_bits(&(CANx->IER), CAN_IER_TMEIE | CAN_IER_FMPIE0 | CAN_IER_ERRIE | CAN_IER_LECIE | CAN_IER_BOFIE | CAN_IER_EPVIE | CAN_IER_EWGIE | CAN_IER_FOVIE0 | CAN_IER_FFIE0);

    // clear overrun flag on init
    CANx->RF0R &= ~(CAN_RF0R_FOVR0);

    llcan_irq_enable(CANx);
  }
  return ret;
}

void llcan_clear_send(CAN_TypeDef *CANx) {
  CANx->TSR |= CAN_TSR_ABRQ0; // Abort message transmission on error interrupt
  CANx->MSR |= CAN_MSR_ERRI; // Clear error interrupt
}
