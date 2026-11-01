/** @file
  Shared SD layer interface (from Exynos441xPkg SdCardDxe.h, SD only): the
  command/init layer (mmccore.c) drives the SDHCI backend (sdhci.c) through
  a small ops table.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#ifndef SD_CARD_H_
#define SD_CARD_H_

#include "efishim.h"

#define BLOCK_SIZE  512

typedef enum {
  CardTypeNone = 0,
  CardTypeMmc,
  CardTypeSd,
} CARD_TYPE;

typedef struct {
  CARD_TYPE  Type;
  UINT16     Rca;
  BOOLEAN    HighCapacity;
  UINT32     BlockSize;
  UINT64     BlockCount;
} CARD_STATE;

typedef struct _MMC_HOST MMC_HOST;

typedef EFI_STATUS (EFIAPI *MMC_HOST_SEND_CMD)(MMC_HOST *Host, UINT32 Cmd, UINT32 Arg, UINT32 Flags, UINT32 *Resp);
typedef EFI_STATUS (EFIAPI *MMC_HOST_SET_CLOCK)(MMC_HOST *Host, UINT32 ClockHz);
typedef EFI_STATUS (EFIAPI *MMC_HOST_PREPARE_DATA)(MMC_HOST *Host, BOOLEAN Read, UINT32 BlockCount);
typedef EFI_STATUS (EFIAPI *MMC_HOST_SET_BUS_WIDTH)(MMC_HOST *Host, UINT32 Width);
typedef EFI_STATUS (EFIAPI *MMC_HOST_XFER_BLOCK)(MMC_HOST *Host, BOOLEAN Read, UINT32 BlockCount, VOID *Buffer);

#define HOST_CMD_RESP_NONE   0x0000
#define HOST_CMD_RESP_SHORT  0x0001
#define HOST_CMD_RESP_LONG   0x0002
#define HOST_CMD_RESP_BUSY   0x0004
#define HOST_CMD_DATA        0x0010
#define HOST_CMD_WRITE       0x0020
#define HOST_CMD_NO_CRC      0x0040

struct _MMC_HOST {
  UINT32                  Base;         // mapped register base (VA)
  UINT32                  MaxClockHz;
  UINT32                  BaseClockHz;  // sclk_mmc2 feeding the SDHCI divider
  CARD_STATE              Card;
  MMC_HOST_SEND_CMD       SendCmd;
  MMC_HOST_SET_CLOCK      SetClock;
  MMC_HOST_PREPARE_DATA   PrepareData;
  MMC_HOST_XFER_BLOCK     XferBlock;
  MMC_HOST_SET_BUS_WIDTH  SetBusWidth;
};

EFI_STATUS MmcInitCard (MMC_HOST *Host, BOOLEAN IsEmmc);
EFI_STATUS MmcReadBlocks (MMC_HOST *Host, EFI_LBA Lba, UINTN Blocks, VOID *Buffer);
EFI_STATUS MmcWriteBlocks (MMC_HOST *Host, EFI_LBA Lba, UINTN Blocks, VOID *Buffer);
EFI_STATUS SdhciInit (MMC_HOST *Host, UINT32 Base, UINT32 MaxClockHz, UINT32 BaseClockHz);

#endif
