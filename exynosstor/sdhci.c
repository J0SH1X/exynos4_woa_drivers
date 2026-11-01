/** @file
  Standard SDHCI host backend (Exynos4 HSMMC) for microSD (copied from
  Exynos441xPkg SdCardDxe/Sdhci.c; built for Windows through efishim.h).

  Drives the SDHCI-standard host controller at EXYNOS4_PA_HSMMC2 in PIO
  mode.  Register offsets follow the SDHCI spec (drivers/mmc/host/sdhci.h);
  the Exynos4 extras (CONTROL2/3 base-clock select) follow
  arch/arm/plat-samsung/include/plat/regs-sdhci.h.

  The card-detect decision (whether to even bring this host up) is made by
  the driver entry point, not here.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include "sdcard.h"

//
// SDHCI register offsets (spec).
//
#define SDHCI_BLOCK_SIZE       0x04
#define SDHCI_BLOCK_COUNT      0x06
#define SDHCI_ARGUMENT         0x08
#define SDHCI_TRANSFER_MODE    0x0C
#define SDHCI_COMMAND          0x0E
#define SDHCI_RESPONSE         0x10
#define SDHCI_BUFFER_DATA_PORT 0x20
#define SDHCI_PRESENT_STATE    0x24
#define SDHCI_HOST_CONTROL     0x28
#define SDHCI_POWER_CONTROL    0x29
#define SDHCI_CLOCK_CONTROL    0x2C
#define SDHCI_TIMEOUT_CONTROL  0x2E
#define SDHCI_SOFTWARE_RESET   0x2F
#define SDHCI_INT_STATUS       0x30
#define SDHCI_INT_ENABLE       0x34
#define SDHCI_SIGNAL_ENABLE    0x38
#define SDHCI_HOST_CONTROL2    0x3E
#define SDHCI_CAPABILITIES     0x40
#define SDHCI_HOST_VERSION     0xFE

//
// PRESENT_STATE bits.
//
#define SDHCI_CMD_INHIBIT   (1U << 0)
#define SDHCI_DAT_INHIBIT   (1U << 1)
#define SDHCI_SPACE_AVAILABLE (1U << 10)  // buffer write enable
#define SDHCI_DATA_AVAILABLE  (1U << 11)  // buffer read enable
#define SDHCI_DAT_LVL_MASK  (0x0FU << 20)
#define SDHCI_DAT_LVL_SHIFT 20

//
// CLOCK_CONTROL bits.
//
#define SDHCI_CLK_INT_EN     0x0001
#define SDHCI_CLK_INT_STABLE 0x0002
#define SDHCI_CLK_CARD_EN    0x0004
#define SDHCI_DIV_MASK       0xFF00

//
// SOFTWARE_RESET bits.
//
#define SDHCI_RST_ALL  0x01
#define SDHCI_RST_CMD  0x02
#define SDHCI_RST_DATA 0x04

//
// INT_STATUS bits (only the ones this driver waits on).
//
#define SDHCI_INT_CMD_DONE   (1U << 0)
#define SDHCI_INT_XFER_DONE  (1U << 1)
#define SDHCI_INT_DMA        (1U << 3)
#define SDHCI_INT_BUF_WR      (1U << 4)
#define SDHCI_INT_BUF_RD      (1U << 5)
#define SDHCI_INT_ERR_MASK   0xFFFF0000U

//
// COMMAND register construction.
//
#define SDHCI_CMD_RESP_NONE   0x00
#define SDHCI_CMD_RESP_LONG   0x01
#define SDHCI_CMD_RESP_SHORT  0x02
#define SDHCI_CMD_RESP_BUSY   0x03
#define SDHCI_CMD_CRC         0x08
#define SDHCI_CMD_INDEX       0x10
#define SDHCI_CMD_DATA        0x20

//
// TRANSFER_MODE bits.
//
#define SDHCI_TRNS_BLK_CNT_EN 0x02
#define SDHCI_TRNS_READ    0x10
#define SDHCI_TRNS_MULTI   0x20

//
// HOST_CONTROL bits.
//
#define SDHCI_CTRL_4BITBUS  0x02
#define SDHCI_CTRL_8BITBUS  0x20

//
// Exynos4 CONTROL2 (base-clock select).
//
#define S3C_SDHCI_CONTROL2         0x80
#define S3C_SDHCI_CONTROL3         0x84

//
// CONTROL2 bits the kernel sets for every card (exynos4_setup_sdhci_cfg_card).
// ENCLKOUTHOLD stops SDCLK while the data buffer is full; without it a slow
// PIO reader overruns the buffer and a multi-block read stalls mid-way.
//
#define S3C_CTRL2_ENSTAASYNCCLR    (1U << 31)
#define S3C_CTRL2_ENCMDCNFMSK      (1U << 30)
#define S3C_CTRL2_ENFBCLKRX        (1U << 14)
#define S3C_CTRL2_ENCLKOUTHOLD     (1U << 8)
//
// CONTROL3 RX feedback-clock delay (FCSELn).  The kernel sets all four, but
// it runs the card at 50 MHz high speed; at default speed (12.5 MHz here)
// that delay samples DAT3 a bit late and 4-bit reads fail with data CRC
// errors (one flipped DAT3 bit per block, independent of the clock rate).
// Feedback clock on with no extra delay reads clean in 1- and 4-bit.
//
#define S3C_CTRL3_NO_FB_DELAY      0

//
// Tight polls before falling back to 1 ms stalls (Stall granularity is
// ~100 us, far longer than one block at speed).
//
#define SDHCI_SPIN_POLLS  200000
#define S3C_CTRL2_SELBASECLK_MASK  (0x3U << 4)
#define S3C_CTRL2_SELBASECLK_SHIFT 4
#define S3C_CTRL2_SELBASECLK_SCLK  2   // sclk_mmc from the CMU (clk_type DIV_EXTERNAL)

//
// Wait for a bit in PRESENT_STATE before issuing a command/transfer.
//
STATIC
EFI_STATUS
WaitInhibitClear (
  IN MMC_HOST  *Host,
  IN UINT32    Mask
  )
{
  UINTN  Retry;

  for (Retry = 0; Retry < 2000; Retry++) {
    if ((MmioRead32 (Host->Base + SDHCI_PRESENT_STATE) & Mask) == 0) {
      return EFI_SUCCESS;
    }
    gBS->Stall (1000);
  }
  return EFI_TIMEOUT;
}

//
// CMD/DAT line reset.  Required by the spec after an error, and after an
// abandoned transfer so the next command is not stuck behind DAT inhibit.
//
STATIC
VOID
ResetLines (
  IN MMC_HOST  *Host
  )
{
  UINTN  Retry;

  MmioWrite8 (Host->Base + SDHCI_SOFTWARE_RESET, SDHCI_RST_CMD | SDHCI_RST_DATA);
  for (Retry = 0; Retry < 1000; Retry++) {
    if (!(MmioRead8 (Host->Base + SDHCI_SOFTWARE_RESET) &
          (SDHCI_RST_CMD | SDHCI_RST_DATA))) {
      break;
    }
    gBS->Stall (1000);
  }
}

//
// Poll SDHCI_INT_STATUS for completion (command or transfer), clear it, and
// surface any error bits.
//
STATIC
EFI_STATUS
WaitInt (
  IN MMC_HOST  *Host,
  IN UINT32    DoneMask
  )
{
  UINTN  Retry;
  UINTN  Spin;
  UINT32 Int;

  Spin = 0;
  for (Retry = 0; Retry < 5000; Retry++) {
    Int = MmioRead32 (Host->Base + SDHCI_INT_STATUS);
    if (Int & SDHCI_INT_ERR_MASK) {
      MmioWrite32 (Host->Base + SDHCI_INT_STATUS, Int);
      DEBUG ((DEBUG_ERROR, "Sdhci: INT error 0x%x CMD=0x%x\n",
              Int, MmioRead16 (Host->Base + SDHCI_COMMAND)));

      ResetLines (Host);
      return EFI_DEVICE_ERROR;
    }
    if (Int & DoneMask) {
      MmioWrite32 (Host->Base + SDHCI_INT_STATUS, Int & DoneMask);
      return EFI_SUCCESS;
    }
    if (Spin < SDHCI_SPIN_POLLS) {
      Spin++;
      Retry--;
      continue;
    }
    gBS->Stall (1000);
  }
  DEBUG ((DEBUG_ERROR, "Sdhci: wait 0x%x timeout INT=0x%x PRES=0x%x BLKCNT=%u CMD=0x%x\n",
          DoneMask, Int, MmioRead32 (Host->Base + SDHCI_PRESENT_STATE),
          MmioRead16 (Host->Base + SDHCI_BLOCK_COUNT),
          MmioRead16 (Host->Base + SDHCI_COMMAND)));
  ResetLines (Host);
  return EFI_TIMEOUT;
}

STATIC
EFI_STATUS
EFIAPI
SdhciSendCmd (
  IN MMC_HOST  *Host,
  IN UINT32    Cmd,
  IN UINT32    Arg,
  IN UINT32    Flags,
  OUT UINT32   *Resp
  )
{
  EFI_STATUS  Status;
  UINT32      CmdReg;
  UINT32      RespCtl;

  Status = WaitInhibitClear (Host, SDHCI_CMD_INHIBIT);
  if (EFI_ERROR (Status)) {
    return Status;
  }
  if ((Flags & HOST_CMD_DATA) &&
      EFI_ERROR (WaitInhibitClear (Host, SDHCI_DAT_INHIBIT))) {
    return EFI_TIMEOUT;
  }

  //
  // Nothing is in flight (both inhibits are clear): drop stale status.  A
  // leftover XFER_DONE (e.g. from an R1b busy end) would otherwise make the
  // next write's completion wait return before the data was sent.
  //
  MmioWrite32 (Host->Base + SDHCI_INT_STATUS, 0xFFFFFFFFU);

  //
  // ARGUMENT, then COMMAND (write triggers execution).
  //
  MmioWrite32 (Host->Base + SDHCI_ARGUMENT, Arg);

  //
  // CRC check unless the response has none (R3); index check only for
  // R1/R1b/R6/R7 (kernel MMC_RSP_OPCODE): R2 and R3 carry 111111 there.
  //
  RespCtl = 0;
  if (!(Flags & HOST_CMD_NO_CRC)) {
    RespCtl |= SDHCI_CMD_CRC;
    if (Flags & (HOST_CMD_RESP_SHORT | HOST_CMD_RESP_BUSY)) {
      RespCtl |= SDHCI_CMD_INDEX;
    }
  }
  if (Flags & HOST_CMD_RESP_LONG) {
    RespCtl |= SDHCI_CMD_RESP_LONG;
  } else if (Flags & HOST_CMD_RESP_BUSY) {
    RespCtl |= SDHCI_CMD_RESP_BUSY;
  } else if (Flags & HOST_CMD_RESP_SHORT) {
    RespCtl |= SDHCI_CMD_RESP_SHORT;
  }

  CmdReg = (Cmd << 8) | RespCtl;
  if (Flags & HOST_CMD_DATA) {
    CmdReg |= SDHCI_CMD_DATA;
  }

  MmioWrite16 (Host->Base + SDHCI_COMMAND, (UINT16)CmdReg);

  Status = WaitInt (Host, SDHCI_INT_CMD_DONE);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  //
  // R1b without data (CMD7, CMD12 after a write): wait out the card's busy
  // on DAT0.  The Exynos SDHCI raises no busy-end interrupt (kernel quirk
  // SDHCI_QUIRK_NO_BUSY_IRQ), but DAT inhibit holds until DAT0 is released.
  //
  if ((Flags & HOST_CMD_RESP_BUSY) && !(Flags & HOST_CMD_DATA)) {
    Status = WaitInhibitClear (Host, SDHCI_DAT_INHIBIT);
    if (EFI_ERROR (Status)) {
      return Status;
    }
  }

  if (Resp != NULL) {
    Resp[0] = MmioRead32 (Host->Base + SDHCI_RESPONSE + 0x0);
    Resp[1] = MmioRead32 (Host->Base + SDHCI_RESPONSE + 0x4);
    Resp[2] = MmioRead32 (Host->Base + SDHCI_RESPONSE + 0x8);
    Resp[3] = MmioRead32 (Host->Base + SDHCI_RESPONSE + 0xC);

    //
    // SDHCI stores R2 without its CRC byte (bits 127:8 in RESPONSE[119:0]);
    // shift back up so Resp[3] holds bits 127:96 like the MSHCI backend.
    //
    if (Flags & HOST_CMD_RESP_LONG) {
      Resp[3] = (Resp[3] << 8) | (Resp[2] >> 24);
      Resp[2] = (Resp[2] << 8) | (Resp[1] >> 24);
      Resp[1] = (Resp[1] << 8) | (Resp[0] >> 24);
      Resp[0] = Resp[0] << 8;
    }
  }

  return EFI_SUCCESS;
}

//
// Program the internal divider + Exynos4 base-clock select, then enable the
// SD clock.  MaxClockHz is the fastest the card was asked to run; we derive
// the divider from the host's own base clock via CAPABILITIES.
//
STATIC
EFI_STATUS
EFIAPI
SdhciSetClock (
  IN MMC_HOST  *Host,
  IN UINT32    ClockHz
  )
{
  UINT32  BaseClock;
  UINT32  Div;
  UINT16  ClkReg;
  UINTN   Retry;

  //
  // Disable the SD clock before reprogramming.
  //
  MmioWrite16 (Host->Base + SDHCI_CLOCK_CONTROL, 0);

  if (ClockHz == 0) {
    return EFI_SUCCESS;
  }

  //
  // CAPABILITIES[7:0] = base clock in MHz.
  //
  //
  // CAPABILITIES[13:8] = base clock in MHz; 0 on Exynos4 (external divider),
  // so use the sclk_mmc rate the board code programmed.
  //
  BaseClock = ((MmioRead32 (Host->Base + SDHCI_CAPABILITIES) >> 8) & 0x3F) * 1000000U;
  if (BaseClock == 0) {
    BaseClock = Host->BaseClockHz;
  }

  //
  // SDHCI 2.0 divider: SDCLK = base / (2 * Div) with Div a power of two
  // (0x01..0x80), 0 = base.  Round up so we never exceed the target.
  //
  Div = 0;
  if (BaseClock > ClockHz) {
    for (Div = 1; (Div < 0x80) && ((BaseClock / (2 * Div)) > ClockHz); Div <<= 1) {
    }
  }

  ClkReg = (UINT16)((Div << 8) | SDHCI_CLK_INT_EN);
  MmioWrite16 (Host->Base + SDHCI_CLOCK_CONTROL, ClkReg);

  //
  // Wait for the internal clock to stabilise.
  //
  for (Retry = 0; Retry < 1000; Retry++) {
    ClkReg = MmioRead16 (Host->Base + SDHCI_CLOCK_CONTROL);
    if (ClkReg & SDHCI_CLK_INT_STABLE) {
      break;
    }
    gBS->Stall (1000);
  }
  if (!(ClkReg & SDHCI_CLK_INT_STABLE)) {
    return EFI_TIMEOUT;
  }

  //
  // Enable the card-side clock.
  //
  MmioWrite16 (Host->Base + SDHCI_CLOCK_CONTROL,
               (UINT16)(ClkReg | SDHCI_CLK_CARD_EN));

  return EFI_SUCCESS;
}

//
// Program block size/count and transfer mode ahead of the data command (both
// are latched when the COMMAND register is written).
//
STATIC
EFI_STATUS
EFIAPI
SdhciPrepareData (
  IN MMC_HOST  *Host,
  IN BOOLEAN   Read,
  IN UINT32    BlockCount
  )
{
  UINT16  Mode;

  //
  // Longest data timeout (TMCLK * 2^27).  The reset value 0 is TMCLK * 2^13,
  // shorter than an ordinary card's read access time.
  //
  MmioWrite8 (Host->Base + SDHCI_TIMEOUT_CONTROL, 0x0E);
  MmioWrite16 (Host->Base + SDHCI_BLOCK_SIZE, (UINT16)BLOCK_SIZE);
  MmioWrite16 (Host->Base + SDHCI_BLOCK_COUNT, (UINT16)BlockCount);

  Mode = 0;
  if (Read) {
    Mode |= SDHCI_TRNS_READ;
  }
  if (BlockCount > 1) {
    Mode |= SDHCI_TRNS_MULTI | SDHCI_TRNS_BLK_CNT_EN;
  }
  MmioWrite16 (Host->Base + SDHCI_TRANSFER_MODE, Mode);

  return EFI_SUCCESS;
}

//
// PIO one or more 512-byte blocks through the SDHCI buffer port.
//
STATIC
EFI_STATUS
EFIAPI
SdhciXferBlock (
  IN MMC_HOST  *Host,
  IN BOOLEAN   Read,
  IN UINT32    BlockCount,
  OUT VOID     *Buffer
  )
{
  UINT32  *Ptr;
  UINT32  Words;
  UINT32  Block;
  UINT32  Mask;
  UINT32  Int;
  UINTN   Retry;

  Ptr = Buffer;

  //
  // Gate each block on PRESENT_STATE buffer read/write enable, as Linux's
  // sdhci_transfer_pio() does.  The controller double-buffers; waiting for
  // one BUF_RD/BUF_WR interrupt per block loses one when two blocks land
  // back to back (seen: CMD18 stuck at BLKCNT=29 with no BUF_RD).
  //
  Mask = Read ? SDHCI_DATA_AVAILABLE : SDHCI_SPACE_AVAILABLE;
  for (Block = 0; Block < BlockCount; Block++) {
    for (Retry = 0; ; Retry++) {
      Int = MmioRead32 (Host->Base + SDHCI_INT_STATUS);
      if (Int & SDHCI_INT_ERR_MASK) {
        MmioWrite32 (Host->Base + SDHCI_INT_STATUS, Int);
        DEBUG ((DEBUG_ERROR, "Sdhci: data error 0x%x at block %u/%u\n",
                Int, Block, BlockCount));
        ResetLines (Host);
        return EFI_DEVICE_ERROR;
      }
      if (MmioRead32 (Host->Base + SDHCI_PRESENT_STATE) & Mask) {
        break;
      }
      if (Retry < SDHCI_SPIN_POLLS) {
        continue;
      }
      if (Retry >= SDHCI_SPIN_POLLS + 5000) {
        DEBUG ((DEBUG_ERROR, "Sdhci: buffer wait timeout at block %u/%u INT=0x%x PRES=0x%x BLKCNT=%u\n",
                Block, BlockCount, Int,
                MmioRead32 (Host->Base + SDHCI_PRESENT_STATE),
                MmioRead16 (Host->Base + SDHCI_BLOCK_COUNT)));
        ResetLines (Host);
        return EFI_TIMEOUT;
      }
      gBS->Stall (1000);
    }

    MmioWrite32 (Host->Base + SDHCI_INT_STATUS, SDHCI_INT_BUF_RD | SDHCI_INT_BUF_WR);
    for (Words = 0; Words < (BLOCK_SIZE / 4); Words++) {
      if (Read) {
        *Ptr++ = MmioRead32 (Host->Base + SDHCI_BUFFER_DATA_PORT);
      } else {
        MmioWrite32 (Host->Base + SDHCI_BUFFER_DATA_PORT, *Ptr++);
      }
    }
  }

  //
  // Wait for the whole transfer to complete.
  //
  return WaitInt (Host, SDHCI_INT_XFER_DONE);
}

//
// Host side of a bus-width switch; the card side (ACMD6) is done by MmcCore.
//
STATIC
EFI_STATUS
EFIAPI
SdhciSetBusWidth (
  IN MMC_HOST  *Host,
  IN UINT32    Width
  )
{
  UINT8  Ctrl;

  Ctrl = MmioRead8 (Host->Base + SDHCI_HOST_CONTROL);
  Ctrl &= (UINT8)~(SDHCI_CTRL_4BITBUS | SDHCI_CTRL_8BITBUS);
  if (Width == 4) {
    Ctrl |= SDHCI_CTRL_4BITBUS;
  }
  MmioWrite8 (Host->Base + SDHCI_HOST_CONTROL, Ctrl);
  return EFI_SUCCESS;
}

/**
  Reset and prepare an SDHCI host for card identification.

  Leaves the host running at a low identification clock; the caller raises it
  via SetClock once the card is in the transfer state.

  @param[out] Host       Initialised host (ops + Card zeroed).
  @param[in]  Base       MMIO base of the SDHCI controller.
  @param[in]  MaxClockHz Fastest bus clock to use in the transfer state.
  @param[in]  BaseClockHz sclk_mmc rate feeding the SDHCI divider.

  @retval EFI_SUCCESS  Host ready.
**/
EFI_STATUS
SdhciInit (
  IN MMC_HOST  *Host,
  IN UINT32    Base,
  IN UINT32    MaxClockHz,
  IN UINT32    BaseClockHz
  )
{
  EFI_STATUS  Status;
  UINTN       Retry;
  UINT32      Ctrl2;

  ZeroMem (Host, sizeof (*Host));
  Host->Base        = Base;
  Host->MaxClockHz  = MaxClockHz;
  Host->BaseClockHz = BaseClockHz;
  Host->SetBusWidth = SdhciSetBusWidth;
  Host->SendCmd    = SdhciSendCmd;
  Host->SetClock   = SdhciSetClock;
  Host->PrepareData = SdhciPrepareData;
  Host->XferBlock  = SdhciXferBlock;

  //
  // SOFTWARE_RESET: all.
  //
  MmioWrite8 (Host->Base + SDHCI_SOFTWARE_RESET, SDHCI_RST_ALL);
  for (Retry = 0; Retry < 1000; Retry++) {
    if (!(MmioRead8 (Host->Base + SDHCI_SOFTWARE_RESET) & SDHCI_RST_ALL)) {
      break;
    }
    gBS->Stall (1000);
  }
  if (MmioRead8 (Host->Base + SDHCI_SOFTWARE_RESET) & SDHCI_RST_ALL) {
    return EFI_TIMEOUT;
  }

  //
  // Polled: enable every status bit (INT_ENABLE gates whether INT_STATUS
  // latches at all) but signal none of them to the GIC.
  //
  MmioWrite32 (Host->Base + SDHCI_INT_ENABLE,  0xFFFFFFFFU);
  MmioWrite32 (Host->Base + SDHCI_SIGNAL_ENABLE, 0);
  MmioWrite32 (Host->Base + SDHCI_INT_STATUS, 0xFFFFFFFFU);

  //
  // Card clock from sclk_mmc (kernel S3C_SDHCI_CLK_DIV_EXTERNAL).
  //
  Ctrl2 = (S3C_CTRL2_SELBASECLK_SCLK << S3C_CTRL2_SELBASECLK_SHIFT) |
          S3C_CTRL2_ENSTAASYNCCLR | S3C_CTRL2_ENCMDCNFMSK |
          S3C_CTRL2_ENFBCLKRX | S3C_CTRL2_ENCLKOUTHOLD;
  MmioWrite32 (Host->Base + S3C_SDHCI_CONTROL2, Ctrl2);
  MmioWrite32 (Host->Base + S3C_SDHCI_CONTROL3, S3C_CTRL3_NO_FB_DELAY);

  //
  // Power on the bus (3.3V).
  //
  MmioWrite8 (Host->Base + SDHCI_POWER_CONTROL, 0x0E);

  //
  // Bus width starts at 1-bit; MmcCore widens to 4-bit for SD later.
  //
  MmioWrite8 (Host->Base + SDHCI_HOST_CONTROL, 0);

  //
  // Identification clock (~400 kHz).
  //
  Status = SdhciSetClock (Host, 400000);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  return EFI_SUCCESS;
}
