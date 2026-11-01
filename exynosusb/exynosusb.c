/** @file
  USB log + debug channel for Windows RT 8.1 on the GT-N7000: the DWC2 USB
  device controller in peripheral mode, presenting the same 1209:0001
  device as UEFI's UsbLogDxe, so Tools/usblog.py and Tools/usbdbg.py work
  against running Windows.

  - Interface 0, bulk IN 0x81: the OS log ring (ExynosOsLog.h, written via
    common/oslog.h), from the oldest byte still in the ring, then live.
  - Interface 1, bulk OUT 0x02 / IN 0x82: USBDBG_PING, READ, WRITE32 (on
    physical addresses, mapped with MmMapIoSpace) and RESET (normal reboot
    through the firmware).  The UEFI file commands are not available.

  ACPI: _HID EXYN0001 (UFN0), the controller's registers and SPI 71.  UEFI
  leaves the PHY, clocks and MUIC routing set up and only soft-disconnects
  at ExitBootServices, so this re-initializes just the DWC2 core.

  Ported from Exynos441xPkg UsbFnDxe (Dwc2.c, UsbFnDxe.c) and UsbLogDxe:
  the ISR does what UsbFnDxe's interrupt handler does (FIFOs, EP0 standard
  requests, event queue); the DPC and a 10 ms timer run UsbLogDxe's poll
  loop.  Register access outside the ISR holds the interrupt lock.

  With LOAD_TEST it bugchecks 0x55534221 ("USB!") in PrepareHardware with
  P1 = memory base, P2 = interrupt (raw GSI), P3 = resource count.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include <ntddk.h>
#include <wdf.h>
#include "oslog.h"

#ifndef LOAD_TEST
#define LOAD_TEST 0
#endif
#define USB_BUGCHECK  0x55534221

//
// DWC2 registers (UsbFnDxe.h).
//
#define GAHBCFG    0x008
#define GUSBCFG    0x00C
#define GRSTCTL    0x010
#define GINTSTS    0x014
#define GINTMSK    0x018
#define GRXSTSP    0x020
#define GRXFSIZ    0x024
#define GNPTXFSIZ  0x028
#define GSNPSID    0x040
#define GHWCFG3    0x04C
#define DIEPTXF(n) (0x104 + ((n) - 1) * 4)

#define GAHBCFG_GLBLINTRMSK    (1u << 0)
#define GAHBCFG_TXFEMPLVL      (1u << 7)

#define GUSBCFG_FORCEDEVMODE   (1u << 30)
#define GUSBCFG_PHYIF16        (1u << 3)
#define GUSBCFG_TRDTIM(x)      ((x) << 10)
#define GUSBCFG_TOUTCAL(x)     (x)

#define GRSTCTL_CSFTRST        (1u << 0)
#define GRSTCTL_RXFFLSH        (1u << 4)
#define GRSTCTL_TXFFLSH        (1u << 5)
#define GRSTCTL_TXFNUM(x)      ((x) << 6)
#define GRSTCTL_TXFNUM_ALL     GRSTCTL_TXFNUM (0x10)
#define GRSTCTL_AHBIDLE        (1u << 31)

#define GINT_RXFLVL            (1u << 4)
#define GINT_USBSUSP           (1u << 11)
#define GINT_USBRST            (1u << 12)
#define GINT_ENUMDONE          (1u << 13)
#define GINT_IEPINT            (1u << 18)
#define GINT_OEPINT            (1u << 19)
#define GINT_WKUPINT           (1u << 31)
#define GINT_DEVICE_MASK       (GINT_RXFLVL | GINT_USBSUSP | GINT_USBRST | GINT_ENUMDONE | \
                                GINT_IEPINT | GINT_OEPINT | GINT_WKUPINT)

#define GRXSTS_EPNUM(x)        ((x) & 0xF)
#define GRXSTS_BCNT(x)         (((x) >> 4) & 0x7FF)
#define GRXSTS_PKTSTS(x)       (((x) >> 17) & 0xF)
#define PKTSTS_OUT_DATA        2
#define PKTSTS_SETUP_DATA      6

#define DCFG        0x800
#define DCTL        0x804
#define DIEPMSK     0x810
#define DOEPMSK     0x814
#define DAINT       0x818
#define DAINTMSK    0x81C
#define DIEPEMPMSK  0x834
#define DIEPCTL(n)  (0x900 + (n) * 0x20)
#define DIEPINT(n)  (0x908 + (n) * 0x20)
#define DIEPTSIZ(n) (0x910 + (n) * 0x20)
#define DTXFSTS(n)  (0x918 + (n) * 0x20)
#define DOEPCTL(n)  (0xB00 + (n) * 0x20)
#define DOEPINT(n)  (0xB08 + (n) * 0x20)
#define DOEPTSIZ(n) (0xB10 + (n) * 0x20)
#define PCGCCTL     0xE00
#define DFIFO(n)    (0x1000 + (n) * 0x1000)

#define DCFG_DEVADDR_MASK      (0x7Fu << 4)
#define DCFG_DEVADDR(x)        ((x) << 4)
#define DCTL_SFTDISCON         (1u << 1)
#define DCTL_CGNPINNAK         (1u << 8)

#define DEPCTL_MPS_MASK        0x7FF
#define DEPCTL_USBACTEP        (1u << 15)
#define DEPCTL_EPTYPE(x)       ((x) << 18)
#define DEPCTL_STALL           (1u << 21)
#define DEPCTL_TXFNUM(x)       ((x) << 22)
#define DEPCTL_CNAK            (1u << 26)
#define DEPCTL_SNAK            (1u << 27)
#define DEPCTL_SETD0PID        (1u << 28)
#define DEPCTL_EPENA           (1u << 31)

#define DEPINT_XFERCOMPL       (1u << 0)
#define DOEPINT_SETUP          (1u << 3)
#define DIEPINT_TXFE           (1u << 7)

#define DEPTSIZ_PKTCNT(x)      ((x) << 19)
#define DOEPTSIZ0_SUPCNT(x)    ((x) << 29)

#define RX_FIFO_WORDS  512
#define EP0_TXF_WORDS  64
#define EPN_TXF_WORDS  256

#define MAX_EPS     4
#define EP0_MPS     64
#define BULK_MPS    512
#define EP_BULK     2
#define LOG_EP      1
#define DBG_EP      2

//
// USB requests and descriptor types.
//
#define REQ_GET_STATUS     0x00
#define REQ_CLEAR_FEATURE  0x01
#define REQ_SET_FEATURE    0x03
#define REQ_SET_ADDRESS    0x05
#define REQ_GET_DESCRIPTOR 0x06
#define REQ_GET_CONFIG     0x08
#define REQ_SET_CONFIG     0x09
#define DESC_DEVICE        0x01
#define DESC_CONFIG        0x02
#define DESC_STRING        0x03

//
// Debug channel wire format (UsbLogDxe.c).
//
#define USBDBG_REQ_MAGIC   0x47424455      // 'UDBG'
#define USBDBG_RSP_MAGIC   0x52424455      // 'UDBR'
#define USBDBG_MAX_READ    4096
#define USBDBG_PING        1
#define USBDBG_READ        2
#define USBDBG_WRITE32     3
#define USBDBG_RESET       4
#define USBDBG_OK          0
#define USBDBG_EBADCMD     1
#define USBDBG_EBADARG     2

#define LOG_CHUNK          4096
#define POLL_MS            10

#include <pshpack1.h>
typedef struct {
  UCHAR   RequestType;
  UCHAR   Request;
  USHORT  Value;
  USHORT  Index;
  USHORT  Length;
} USB_SETUP;

typedef struct {
  ULONG  Magic;
  ULONG  Command;
  ULONG  Arg0;
  ULONG  Arg1;
} USBDBG_HEADER;

typedef struct {
  ULONG  Magic;
  ULONG  Status;
  ULONG  Length;
  ULONG  Reserved;
} USBDBG_REPLY;
#include <poppack.h>

static const UCHAR mDeviceDescriptor[18] = {
  18, DESC_DEVICE, 0x00, 0x02, 0xFF, 0x00, 0x00, EP0_MPS,
  0x09, 0x12, 0x01, 0x00,          // 1209:0001
  0x00, 0x01, 1, 2, 3, 1
};

static const UCHAR mConfigDescriptor[48] = {
  9, DESC_CONFIG, 48, 0, 2, 1, 0, 0x80, 250,
  9, 0x04, 0, 0, 1, 0xFF, 0x00, 0x00, 0,                 // interface 0: log
  7, 0x05, 0x80 | LOG_EP, EP_BULK, BULK_MPS & 0xFF, BULK_MPS >> 8, 0,
  9, 0x04, 1, 0, 2, 0xFF, 0x00, 0x00, 0,                 // interface 1: debug
  7, 0x05, DBG_EP, EP_BULK, BULK_MPS & 0xFF, BULK_MPS >> 8, 0,
  7, 0x05, 0x80 | DBG_EP, EP_BULK, BULK_MPS & 0xFF, BULK_MPS >> 8, 0,
};

typedef struct {
  BOOLEAN  Active;
  BOOLEAN  Internal;      // EP0 reply owned by the driver
  BOOLEAN  SendZlp;
  PUCHAR   Buffer;
  ULONG    Length;
  ULONG    Done;
  ULONG    Queued;
} XFER;

//
// What the ISR hands the DPC (UsbFnDxe's EFI_USBFN messages, reduced).
//
typedef enum {
  EvReset = 1,
  EvSetup,
  EvTxDone,
  EvRxDone,
} USB_EVENT_TYPE;

typedef struct {
  USB_EVENT_TYPE  Type;
  UCHAR       Ep;
  ULONG       Bytes;
  USB_SETUP   Setup;
} USB_EVENT;

#define EVENT_QUEUE  32

typedef struct {
  PUCHAR        Base;
  SIZE_T        Size;
  WDFINTERRUPT  Interrupt;
  WDFTIMER      Timer;
  WDFWORKITEM   ResetWork;

  UCHAR         Configuration;
  XFER          In[MAX_EPS];
  XFER          Out[MAX_EPS];
  ULONG         SetupWords[2];
  UCHAR         Ep0Buffer[256];

  USB_EVENT     Queue[EVENT_QUEUE];
  ULONG         QueueHead;
  ULONG         QueueTail;

  // Function side (UsbLogDxe).
  BOOLEAN       TxBusy;
  ULONG         ReadPos;
  ULONG         TxEnd;
  UCHAR         Tx[LOG_CHUNK];
  UCHAR         Ep0Reply[64];
  BOOLEAN       DbgRxPosted;
  BOOLEAN       DbgTxBusy;
  BOOLEAN       PendingReset;
  UCHAR         DbgRx[sizeof (USBDBG_HEADER) + 64];
  UCHAR         DbgTx[sizeof (USBDBG_REPLY) + USBDBG_MAX_READ];
} DEVICE_CONTEXT, *PDEVICE_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME (DEVICE_CONTEXT, GetDeviceContext)

//
// HAL export without a public prototype: reboot through the firmware's
// ResetSystem.
//
typedef enum {
  HalHaltRoutine,
  HalPowerDownRoutine,
  HalRestartRoutine,
  HalRebootRoutine,
} FIRMWARE_REENTRY;

NTHALAPI VOID HalReturnToFirmware (FIRMWARE_REENTRY Routine);

DRIVER_INITIALIZE                DriverEntry;
EVT_WDF_DRIVER_DEVICE_ADD        EvtDeviceAdd;
EVT_WDF_DEVICE_PREPARE_HARDWARE  EvtPrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE  EvtReleaseHardware;
EVT_WDF_DEVICE_D0_ENTRY          EvtD0Entry;
EVT_WDF_DEVICE_D0_EXIT           EvtD0Exit;
EVT_WDF_INTERRUPT_ISR            EvtIsr;
EVT_WDF_INTERRUPT_DPC            EvtDpc;
EVT_WDF_TIMER                    EvtTimer;
EVT_WDF_WORKITEM                 EvtResetWork;

static ULONG Rd (PDEVICE_CONTEXT D, ULONG R)          { return READ_REGISTER_ULONG ((PULONG)(D->Base + R)); }
static VOID  Wr (PDEVICE_CONTEXT D, ULONG R, ULONG V) { WRITE_REGISTER_ULONG ((PULONG)(D->Base + R), V); }
static VOID  Or (PDEVICE_CONTEXT D, ULONG R, ULONG V) { Wr (D, R, Rd (D, R) | V); }
static VOID  And (PDEVICE_CONTEXT D, ULONG R, ULONG V){ Wr (D, R, Rd (D, R) & V); }

//
// ---------------------------------------------------------------------------
// DWC2 core (UsbFnDxe Dwc2.c).
// ---------------------------------------------------------------------------
//

static BOOLEAN
WaitClear (PDEVICE_CONTEXT D, ULONG Reg, ULONG Mask)
{
  ULONG  Retry;

  for (Retry = 0; Retry < 10000; Retry++) {
    if ((Rd (D, Reg) & Mask) == 0) {
      return TRUE;
    }
    KeStallExecutionProcessor (10);
  }
  return FALSE;
}

static VOID
FlushFifos (PDEVICE_CONTEXT D)
{
  Wr (D, GRSTCTL, GRSTCTL_TXFFLSH | GRSTCTL_TXFNUM_ALL);
  WaitClear (D, GRSTCTL, GRSTCTL_TXFFLSH);
  Wr (D, GRSTCTL, GRSTCTL_RXFFLSH);
  WaitClear (D, GRSTCTL, GRSTCTL_RXFFLSH);
}

static NTSTATUS
CoreInit (PDEVICE_CONTEXT D)
{
  ULONG  Retry;
  ULONG  Start;
  ULONG  Ep;

  for (Retry = 0; (Retry < 10000) && ((Rd (D, GRSTCTL) & GRSTCTL_AHBIDLE) == 0); Retry++) {
    KeStallExecutionProcessor (10);
  }
  Wr (D, GRSTCTL, GRSTCTL_CSFTRST);
  if (!WaitClear (D, GRSTCTL, GRSTCTL_CSFTRST)) {
    OsLog ("exynosusb: core soft reset stuck");
    return STATUS_DEVICE_NOT_READY;
  }

  // Exynos4210: 16-bit UTMI+, turnaround 5; forced device mode settles in 25 ms.
  Wr (D, GUSBCFG, GUSBCFG_FORCEDEVMODE | GUSBCFG_TRDTIM (5) | GUSBCFG_PHYIF16 | GUSBCFG_TOUTCAL (7));
  KeStallExecutionProcessor (25000);

  Wr (D, DCTL, DCTL_SFTDISCON);
  Wr (D, DCFG, 0);
  Wr (D, PCGCCTL, 0);

  Wr (D, GRXFSIZ, RX_FIFO_WORDS);
  Wr (D, GNPTXFSIZ, (EP0_TXF_WORDS << 16) | RX_FIFO_WORDS);
  Start = RX_FIFO_WORDS + EP0_TXF_WORDS;
  for (Ep = 1; Ep < MAX_EPS; Ep++) {
    Wr (D, DIEPTXF (Ep), (EPN_TXF_WORDS << 16) | Start);
    Start += EPN_TXF_WORDS;
  }
  FlushFifos (D);

  Wr (D, DIEPMSK, DEPINT_XFERCOMPL);
  Wr (D, DOEPMSK, DEPINT_XFERCOMPL | DOEPINT_SETUP);
  Wr (D, DAINTMSK, (1u << 0) | (1u << 16));
  Wr (D, DIEPEMPMSK, 0);
  Wr (D, GINTSTS, 0xFFFFFFFF);
  Wr (D, GINTMSK, GINT_DEVICE_MASK);
  Wr (D, GAHBCFG, GAHBCFG_GLBLINTRMSK | GAHBCFG_TXFEMPLVL);

  OsLog ("exynosusb: DWC2 %08lx up", Rd (D, GSNPSID));
  return STATUS_SUCCESS;
}

static VOID
Ep0ArmOut (PDEVICE_CONTEXT D)
{
  Wr (D, DOEPTSIZ (0), DOEPTSIZ0_SUPCNT (3) | DEPTSIZ_PKTCNT (1) | EP0_MPS);
  Or (D, DOEPCTL (0), DEPCTL_EPENA | DEPCTL_CNAK);
}

static VOID
BusReset (PDEVICE_CONTEXT D)
{
  ULONG  Ep;

  for (Ep = 0; Ep < MAX_EPS; Ep++) {
    Or (D, DOEPCTL (Ep), DEPCTL_SNAK);
    if (Ep != 0) {
      Wr (D, DIEPCTL (Ep), 0);
      Wr (D, DOEPCTL (Ep), 0);
    }
    RtlZeroMemory (&D->In[Ep], sizeof (D->In[Ep]));
    RtlZeroMemory (&D->Out[Ep], sizeof (D->Out[Ep]));
  }
  FlushFifos (D);
  And (D, DCFG, ~DCFG_DEVADDR_MASK);
  Wr (D, DAINTMSK, (1u << 0) | (1u << 16));
  Wr (D, DIEPEMPMSK, 0);
  D->Configuration = 0;
  Ep0ArmOut (D);
}

static VOID
ActivateEndpoints (PDEVICE_CONTEXT D)
{
  ULONG  Ep;
  ULONG  Ctl;

  for (Ep = 1; Ep <= 2; Ep++) {
    if (((Rd (D, DIEPCTL (Ep)) | Rd (D, DOEPCTL (Ep))) & DEPCTL_USBACTEP) != 0) {
      Or (D, DIEPCTL (Ep), DEPCTL_SETD0PID);
      Or (D, DOEPCTL (Ep), DEPCTL_SETD0PID);
      continue;
    }
    Ctl = DEPCTL_USBACTEP | DEPCTL_SETD0PID | DEPCTL_SNAK | DEPCTL_EPTYPE (EP_BULK) | BULK_MPS;
    Wr (D, DIEPCTL (Ep), Ctl | DEPCTL_TXFNUM (Ep));
    Wr (D, DOEPCTL (Ep), Ctl);
    Or (D, DAINTMSK, (1u << Ep) | (1u << (16 + Ep)));
  }
}

static VOID
FillTxFifo (PDEVICE_CONTEXT D, ULONG Ep)
{
  XFER   *X;
  ULONG  End;
  ULONG  Index;
  ULONG  Space;
  ULONG  Chunk;
  ULONG  Word;

  X     = &D->In[Ep];
  End   = (Ep == 0) ? (X->Done + X->Queued) : X->Length;
  Index = X->Done;

  while (Index < End) {
    Space = (Rd (D, DTXFSTS (Ep)) & 0xFFFF) * 4;
    Chunk = min (End - Index, Space);
    if (Chunk == 0) {
      break;
    }
    while (Chunk > 0) {
      Word = 0;
      RtlCopyMemory (&Word, X->Buffer + Index, min (Chunk, 4));
      Wr (D, DFIFO (Ep), Word);
      Index += min (Chunk, 4);
      Chunk -= min (Chunk, 4);
    }
  }

  if (Ep != 0) {
    X->Done = Index;
  }
  if (Index < End) {
    Or (D, DIEPEMPMSK, 1u << Ep);
  } else {
    And (D, DIEPEMPMSK, ~(1u << Ep));
  }
}

static VOID
StartIn (PDEVICE_CONTEXT D, ULONG Ep)
{
  XFER   *X;
  ULONG  Mps;
  ULONG  Len;
  ULONG  Pkts;

  X   = &D->In[Ep];
  Mps = (Ep == 0) ? EP0_MPS : BULK_MPS;
  Len = X->Length - X->Done;
  if (Ep == 0) {
    Len = min (Len, Mps);
  }
  X->Queued = Len;

  Pkts = (Len == 0) ? 1 : (Len + Mps - 1) / Mps;
  Wr (D, DIEPTSIZ (Ep), DEPTSIZ_PKTCNT (Pkts) | Len);
  Or (D, DIEPCTL (Ep), DEPCTL_EPENA | DEPCTL_CNAK);
  if (Len != 0) {
    FillTxFifo (D, Ep);
  }
}

static VOID
StartOut (PDEVICE_CONTEXT D, ULONG Ep)
{
  ULONG  Pkts;

  Pkts = max ((D->Out[Ep].Length + BULK_MPS - 1) / BULK_MPS, 1);
  Wr (D, DOEPTSIZ (Ep), DEPTSIZ_PKTCNT (Pkts) | (Pkts * BULK_MPS));
  Or (D, DOEPCTL (Ep), DEPCTL_EPENA | DEPCTL_CNAK);
}

static VOID
SetStall (PDEVICE_CONTEXT D, ULONG Ep, BOOLEAN In, BOOLEAN Stall)
{
  ULONG  Reg;

  Reg = In ? DIEPCTL (Ep) : DOEPCTL (Ep);
  if (Stall) {
    Or (D, Reg, DEPCTL_STALL);
  } else {
    Wr (D, Reg, (Rd (D, Reg) & ~DEPCTL_STALL) | ((Ep != 0) ? DEPCTL_SETD0PID : 0));
  }
}

//
// ---------------------------------------------------------------------------
// Interrupt service (UsbFnDxe.c), at DIRQL or under the interrupt lock.
// ---------------------------------------------------------------------------
//

static VOID
QueueEvent (PDEVICE_CONTEXT D, USB_EVENT_TYPE Type, ULONG Ep, ULONG Bytes, const USB_SETUP *Setup)
{
  ULONG  Next;

  Next = (D->QueueHead + 1) % EVENT_QUEUE;
  if (Next == D->QueueTail) {
    return;
  }
  RtlZeroMemory (&D->Queue[D->QueueHead], sizeof (USB_EVENT));
  D->Queue[D->QueueHead].Type  = Type;
  D->Queue[D->QueueHead].Ep    = (UCHAR)Ep;
  D->Queue[D->QueueHead].Bytes = Bytes;
  if (Setup != NULL) {
    D->Queue[D->QueueHead].Setup = *Setup;
  }
  D->QueueHead = Next;
}

static VOID
Ep0Reply (PDEVICE_CONTEXT D, const VOID *Data, ULONG Length, USHORT WLength)
{
  XFER  *X;

  Length = min (min (Length, (ULONG)WLength), sizeof (D->Ep0Buffer));
  if (Length != 0) {
    RtlCopyMemory (D->Ep0Buffer, Data, Length);
  }
  X = &D->In[0];
  RtlZeroMemory (X, sizeof (*X));
  X->Active   = TRUE;
  X->Internal = TRUE;
  X->Buffer   = D->Ep0Buffer;
  X->Length   = Length;
  X->SendZlp  = (Length < WLength) && (Length != 0) && ((Length % EP0_MPS) == 0);
  StartIn (D, 0);
}

static VOID
Ep0Stall (PDEVICE_CONTEXT D)
{
  SetStall (D, 0, TRUE, TRUE);
  SetStall (D, 0, FALSE, TRUE);
}

static BOOLEAN
Ep0Standard (PDEVICE_CONTEXT D, const USB_SETUP *Req)
{
  ULONG   Ep;
  USHORT  Status;

  if ((Req->RequestType & 0x60) != 0) {
    return FALSE;
  }

  switch (Req->Request) {
    case REQ_SET_ADDRESS:
      Wr (D, DCFG, (Rd (D, DCFG) & ~DCFG_DEVADDR_MASK) | DCFG_DEVADDR (Req->Value & 0x7F));
      Ep0Reply (D, NULL, 0, 0);
      return TRUE;

    case REQ_GET_DESCRIPTOR:
      switch (Req->Value >> 8) {
        case DESC_DEVICE:
          Ep0Reply (D, mDeviceDescriptor, sizeof (mDeviceDescriptor), Req->Length);
          return TRUE;
        case DESC_CONFIG:
          Ep0Reply (D, mConfigDescriptor, sizeof (mConfigDescriptor), Req->Length);
          return TRUE;
        case DESC_STRING:
          return FALSE;
        default:
          Ep0Stall (D);
          return TRUE;
      }

    case REQ_SET_CONFIG:
      D->Configuration = (UCHAR)Req->Value;
      if (D->Configuration != 0) {
        ActivateEndpoints (D);
      }
      Ep0Reply (D, NULL, 0, 0);
      return TRUE;

    case REQ_GET_CONFIG:
      Ep0Reply (D, &D->Configuration, 1, Req->Length);
      return TRUE;

    case REQ_GET_STATUS:
      Status = 0;
      if ((Req->RequestType & 0x1F) == 2) {
        Ep     = Req->Index & 0xF;
        Status = (Ep < MAX_EPS) &&
                 ((Rd (D, ((Req->Index & 0x80) != 0) ? DIEPCTL (Ep) : DOEPCTL (Ep)) & DEPCTL_STALL) != 0);
      }
      Ep0Reply (D, &Status, sizeof (Status), Req->Length);
      return TRUE;

    case REQ_CLEAR_FEATURE:
    case REQ_SET_FEATURE:
      if (((Req->RequestType & 0x1F) == 2) && (Req->Value == 0)) {
        Ep = Req->Index & 0xF;
        if (Ep < MAX_EPS) {
          SetStall (D, Ep, (Req->Index & 0x80) != 0, Req->Request == REQ_SET_FEATURE);
        }
      }
      Ep0Reply (D, NULL, 0, 0);
      return TRUE;

    default:
      return FALSE;
  }
}

static VOID
Ep0Setup (PDEVICE_CONTEXT D)
{
  USB_SETUP  Req;

  RtlZeroMemory (&D->In[0], sizeof (D->In[0]));
  RtlZeroMemory (&D->Out[0], sizeof (D->Out[0]));
  RtlCopyMemory (&Req, D->SetupWords, sizeof (Req));
  if (!Ep0Standard (D, &Req)) {
    QueueEvent (D, EvSetup, 0, 0, &Req);
  }
  Ep0ArmOut (D);
}

static VOID
RxFifoPop (PDEVICE_CONTEXT D)
{
  ULONG  Sts;
  ULONG  Ep;
  ULONG  Count;
  ULONG  Index;
  ULONG  Word;
  ULONG  Take;
  XFER   *X;

  Sts   = Rd (D, GRXSTSP);
  Ep    = GRXSTS_EPNUM (Sts);
  Count = GRXSTS_BCNT (Sts);

  switch (GRXSTS_PKTSTS (Sts)) {
    case PKTSTS_SETUP_DATA:
      D->SetupWords[0] = Rd (D, DFIFO (0));
      D->SetupWords[1] = Rd (D, DFIFO (0));
      break;

    case PKTSTS_OUT_DATA:
      X = (Ep < MAX_EPS) ? &D->Out[Ep] : NULL;
      for (Index = 0; Index < Count; Index += 4) {
        Word = Rd (D, DFIFO (0));
        if ((X != NULL) && X->Active && (X->Done < X->Length)) {
          Take = min (min (Count - Index, 4), X->Length - X->Done);
          RtlCopyMemory (X->Buffer + X->Done, &Word, Take);
          X->Done += Take;
        }
      }
      if (X != NULL) {
        X->Queued = Count;
      }
      break;

    default:
      break;
  }
}

static VOID
OutComplete (PDEVICE_CONTEXT D, ULONG Ep)
{
  XFER  *X;

  X = &D->Out[Ep];
  if (Ep == 0) {
    Ep0ArmOut (D);
    return;
  }
  if (X->Active) {
    X->Active = FALSE;
    QueueEvent (D, EvRxDone, Ep, X->Done, NULL);
  }
}

static VOID
InComplete (PDEVICE_CONTEXT D, ULONG Ep)
{
  XFER  *X;

  X = &D->In[Ep];
  if (!X->Active) {
    return;
  }
  if (Ep == 0) {
    X->Done += X->Queued;
    if (X->Done < X->Length) {
      StartIn (D, 0);
      return;
    }
    if (X->SendZlp) {
      X->SendZlp = FALSE;
      StartIn (D, 0);
      return;
    }
  }
  X->Active = FALSE;
  if (!X->Internal) {
    QueueEvent (D, EvTxDone, Ep, X->Done, NULL);
  }
}

static BOOLEAN
ServiceController (PDEVICE_CONTEXT D)
{
  ULONG  Sts;
  ULONG  Daint;
  ULONG  Int;
  ULONG  Ep;

  Sts = Rd (D, GINTSTS) & Rd (D, GINTMSK);
  if (Sts == 0) {
    return FALSE;
  }

  if ((Sts & GINT_USBRST) != 0) {
    Wr (D, GINTSTS, GINT_USBRST);
    BusReset (D);
    QueueEvent (D, EvReset, 0, 0, NULL);
  }
  if ((Sts & GINT_ENUMDONE) != 0) {
    Wr (D, GINTSTS, GINT_ENUMDONE);
    And (D, DIEPCTL (0), ~DEPCTL_MPS_MASK);
    Or (D, DCTL, DCTL_CGNPINNAK);
  }
  if ((Sts & GINT_USBSUSP) != 0) {
    Wr (D, GINTSTS, GINT_USBSUSP);
  }
  if ((Sts & GINT_WKUPINT) != 0) {
    Wr (D, GINTSTS, GINT_WKUPINT);
  }

  while ((Rd (D, GINTSTS) & GINT_RXFLVL) != 0) {
    RxFifoPop (D);
  }

  if ((Sts & (GINT_OEPINT | GINT_IEPINT)) != 0) {
    Daint = Rd (D, DAINT) & Rd (D, DAINTMSK);
    for (Ep = 0; Ep < MAX_EPS; Ep++) {
      if ((Daint & (1u << (16 + Ep))) != 0) {
        Int = Rd (D, DOEPINT (Ep));
        Wr (D, DOEPINT (Ep), Int);
        if ((Ep == 0) && ((Int & DOEPINT_SETUP) != 0)) {
          Ep0Setup (D);
        } else if ((Int & DEPINT_XFERCOMPL) != 0) {
          OutComplete (D, Ep);
        }
      }
      if ((Daint & (1u << Ep)) != 0) {
        Int = Rd (D, DIEPINT (Ep));
        if ((Rd (D, DIEPEMPMSK) & (1u << Ep)) == 0) {
          Int &= ~DIEPINT_TXFE;
        }
        Wr (D, DIEPINT (Ep), Int & ~DIEPINT_TXFE);
        if ((Int & DIEPINT_TXFE) != 0) {
          FillTxFifo (D, Ep);
        }
        if ((Int & DEPINT_XFERCOMPL) != 0) {
          InComplete (D, Ep);
        }
      }
    }
  }
  return TRUE;
}

BOOLEAN
EvtIsr (WDFINTERRUPT Interrupt, ULONG MessageId)
{
  PDEVICE_CONTEXT  D;

  UNREFERENCED_PARAMETER (MessageId);

  D = GetDeviceContext (WdfInterruptGetDevice (Interrupt));
  if (!ServiceController (D)) {
    return FALSE;
  }
  WdfInterruptQueueDpcForIsr (Interrupt);
  return TRUE;
}

//
// ---------------------------------------------------------------------------
// Function side (UsbLogDxe.c), at DISPATCH_LEVEL.
// ---------------------------------------------------------------------------
//

/**
  Start a bulk transfer under the interrupt lock; FALSE if the endpoint is
  busy or the host has not configured the device yet.
**/
static BOOLEAN
Transfer (PDEVICE_CONTEXT D, ULONG Ep, BOOLEAN In, PUCHAR Buffer, ULONG Length)
{
  XFER     *X;
  BOOLEAN  Started;

  Started = FALSE;
  WdfInterruptAcquireLock (D->Interrupt);
  X = In ? &D->In[Ep] : &D->Out[Ep];
  if ((D->Configuration != 0) && !X->Active) {
    RtlZeroMemory (X, sizeof (*X));
    X->Active = TRUE;
    X->Buffer = Buffer;
    X->Length = Length;
    if (In) {
      StartIn (D, Ep);
    } else {
      StartOut (D, Ep);
    }
    Started = TRUE;
  }
  WdfInterruptReleaseLock (D->Interrupt);
  return Started;
}

/**
  String descriptors, like UsbLogDxe; anything else stalls.
**/
static VOID
HandleSetup (PDEVICE_CONTEXT D, const USB_SETUP *Req)
{
  const char  *Str;
  ULONG       Size;

  WdfInterruptAcquireLock (D->Interrupt);
  if ((Req->Request != REQ_GET_DESCRIPTOR) || ((Req->Value >> 8) != DESC_STRING)) {
    Ep0Stall (D);
    WdfInterruptReleaseLock (D->Interrupt);
    return;
  }

  switch (Req->Value & 0xFF) {
    case 0:  Str = NULL;                         break;
    case 1:  Str = "Samsung";                    break;
    case 2:  Str = "Galaxy Note Windows log";    break;
    default: Str = "GT-N7000";                   break;
  }

  D->Ep0Reply[1] = DESC_STRING;
  if (Str == NULL) {
    D->Ep0Reply[2] = 0x09;
    D->Ep0Reply[3] = 0x04;
    Size = 4;
  } else {
    for (Size = 2; (*Str != 0) && (Size + 2 <= sizeof (D->Ep0Reply)); Str++) {
      D->Ep0Reply[Size++] = (UCHAR)*Str;
      D->Ep0Reply[Size++] = 0;
    }
  }
  D->Ep0Reply[0] = (UCHAR)Size;
  Ep0Reply (D, D->Ep0Reply, Size, Req->Length);
  WdfInterruptReleaseLock (D->Interrupt);
}

static VOID
LogKick (PDEVICE_CONTEXT D)
{
  ULONG  Head;
  ULONG  Pending;
  ULONG  Index;

  if (D->TxBusy || (mOsLog == NULL) || (mOsLog->Magic != OS_LOG_MAGIC)) {
    return;
  }

  Head = mOsLog->Head;
  if (Head - D->ReadPos > OS_LOG_DATA_SIZE) {
    D->ReadPos = Head - OS_LOG_DATA_SIZE;
  }
  Pending = min (Head - D->ReadPos, LOG_CHUNK);
  if (Pending == 0) {
    return;
  }
  // Every transfer ends in a short packet, so the host read returns.
  if ((Pending % BULK_MPS) == 0) {
    Pending--;
  }
  for (Index = 0; Index < Pending; Index++) {
    D->Tx[Index] = mOsLog->Data[(D->ReadPos + Index) % OS_LOG_DATA_SIZE];
  }

  if (Transfer (D, LOG_EP, TRUE, D->Tx, Pending)) {
    D->TxEnd  = D->ReadPos + Pending;
    D->TxBusy = TRUE;
  }
}

static VOID
DbgPostRx (PDEVICE_CONTEXT D)
{
  if (!D->DbgRxPosted && Transfer (D, DBG_EP, FALSE, D->DbgRx, sizeof (D->DbgRx))) {
    D->DbgRxPosted = TRUE;
  }
}

/**
  Physical memory access for READ/WRITE32: mapped per request, uncached.
**/
static PVOID
MapPhysical (ULONG Address, ULONG Length, PULONG Offset)
{
  PHYSICAL_ADDRESS  Phys;

  Phys.QuadPart = Address & ~(PAGE_SIZE - 1);
  *Offset       = Address & (PAGE_SIZE - 1);
  return MmMapIoSpace (Phys, *Offset + Length, MmNonCached);
}

static VOID
DbgHandle (PDEVICE_CONTEXT D, ULONG Length)
{
  USBDBG_HEADER  Req;
  USBDBG_REPLY   *Rsp;
  PUCHAR         Data;
  PUCHAR         Va;
  ULONG          Offset;
  ULONG          Index;

  Rsp  = (USBDBG_REPLY *)D->DbgTx;
  Data = D->DbgTx + sizeof (USBDBG_REPLY);
  RtlZeroMemory (Rsp, sizeof (*Rsp));
  Rsp->Magic  = USBDBG_RSP_MAGIC;
  Rsp->Status = USBDBG_OK;

  RtlCopyMemory (&Req, D->DbgRx, min (Length, sizeof (Req)));
  if ((Length < sizeof (Req)) || (Req.Magic != USBDBG_REQ_MAGIC)) {
    Rsp->Status = USBDBG_EBADCMD;
  } else {
    switch (Req.Command) {
      case USBDBG_PING:
        RtlCopyMemory (Data, "PONG", 4);
        Rsp->Length = 4;
        break;

      case USBDBG_READ:
        if ((Req.Arg1 == 0) || (Req.Arg1 > USBDBG_MAX_READ)) {
          Rsp->Status = USBDBG_EBADARG;
          break;
        }
        Va = MapPhysical (Req.Arg0, Req.Arg1, &Offset);
        if (Va == NULL) {
          Rsp->Status = USBDBG_EBADARG;
          break;
        }
        if (((Req.Arg0 | Req.Arg1) & 3) == 0) {
          for (Index = 0; Index < Req.Arg1; Index += 4) {
            *(PULONG)(Data + Index) = READ_REGISTER_ULONG ((PULONG)(Va + Offset + Index));
          }
        } else {
          for (Index = 0; Index < Req.Arg1; Index++) {
            Data[Index] = READ_REGISTER_UCHAR (Va + Offset + Index);
          }
        }
        MmUnmapIoSpace (Va, Offset + Req.Arg1);
        Rsp->Length = Req.Arg1;
        break;

      case USBDBG_WRITE32:
        if ((Req.Arg0 & 3) != 0) {
          Rsp->Status = USBDBG_EBADARG;
          break;
        }
        Va = MapPhysical (Req.Arg0, 4, &Offset);
        if (Va == NULL) {
          Rsp->Status = USBDBG_EBADARG;
          break;
        }
        WRITE_REGISTER_ULONG ((PULONG)(Va + Offset), Req.Arg1);
        MmUnmapIoSpace (Va, Offset + 4);
        break;

      case USBDBG_RESET:
        // Windows reboots through the firmware's ResetSystem: normal mode only.
        D->PendingReset = TRUE;
        break;

      default:
        Rsp->Status = USBDBG_EBADCMD;
        break;
    }
  }

  if (Transfer (D, DBG_EP, TRUE, D->DbgTx, sizeof (USBDBG_REPLY) + Rsp->Length)) {
    D->DbgTxBusy = TRUE;
  } else if (D->PendingReset) {
    WdfWorkItemEnqueue (D->ResetWork);
  }
}

static VOID
Poll (PDEVICE_CONTEXT D)
{
  USB_EVENT  Ev;
  BOOLEAN    Have;

  for ( ; ; ) {
    WdfInterruptAcquireLock (D->Interrupt);
    ServiceController (D);       // a lost interrupt must not wedge us
    Have = (D->QueueTail != D->QueueHead);
    if (Have) {
      Ev           = D->Queue[D->QueueTail];
      D->QueueTail = (D->QueueTail + 1) % EVENT_QUEUE;
    }
    WdfInterruptReleaseLock (D->Interrupt);
    if (!Have) {
      break;
    }

    switch (Ev.Type) {
      case EvReset:
        D->TxBusy      = FALSE;
        D->DbgTxBusy   = FALSE;
        D->DbgRxPosted = FALSE;
        break;
      case EvSetup:
        HandleSetup (D, &Ev.Setup);
        break;
      case EvTxDone:
        if (Ev.Ep == LOG_EP) {
          D->ReadPos = D->TxEnd;
          D->TxBusy  = FALSE;
        } else if (Ev.Ep == DBG_EP) {
          D->DbgTxBusy = FALSE;
          if (D->PendingReset) {
            WdfWorkItemEnqueue (D->ResetWork);
          }
        }
        break;
      case EvRxDone:
        if (Ev.Ep == DBG_EP) {
          D->DbgRxPosted = FALSE;
          if (!D->DbgTxBusy) {
            DbgHandle (D, Ev.Bytes);
          }
        }
        break;
    }
  }

  DbgPostRx (D);
  LogKick (D);
}

VOID
EvtDpc (WDFINTERRUPT Interrupt, WDFOBJECT AssociatedObject)
{
  UNREFERENCED_PARAMETER (AssociatedObject);
  Poll (GetDeviceContext (WdfInterruptGetDevice (Interrupt)));
}

VOID
EvtTimer (WDFTIMER Timer)
{
  Poll (GetDeviceContext ((WDFDEVICE)WdfTimerGetParentObject (Timer)));
}

VOID
EvtResetWork (WDFWORKITEM WorkItem)
{
  UNREFERENCED_PARAMETER (WorkItem);
  OsLog ("exynosusb: reboot requested over USB");
  HalReturnToFirmware (HalRebootRoutine);
}

//
// ---------------------------------------------------------------------------
// PnP / power.
// ---------------------------------------------------------------------------
//

NTSTATUS
EvtD0Entry (WDFDEVICE Device, WDF_POWER_DEVICE_STATE PreviousState)
{
  PDEVICE_CONTEXT  D;
  NTSTATUS         Status;

  UNREFERENCED_PARAMETER (PreviousState);

  //
  // KMDF connects the interrupt after D0Entry: no lock needed, and the
  // core reset's 25 ms settle time stays out of DIRQL.
  //
  D = GetDeviceContext (Device);
  Status = CoreInit (D);
  D->QueueHead = D->QueueTail = 0;
  D->Configuration = 0;
  if (NT_SUCCESS (Status)) {
    And (D, DCTL, ~DCTL_SFTDISCON);          // connect
  }

  WdfTimerStart (D->Timer, WDF_REL_TIMEOUT_IN_MS (POLL_MS));
  return Status;
}

NTSTATUS
EvtD0Exit (WDFDEVICE Device, WDF_POWER_DEVICE_STATE TargetState)
{
  PDEVICE_CONTEXT  D;

  UNREFERENCED_PARAMETER (TargetState);

  D = GetDeviceContext (Device);
  WdfTimerStop (D->Timer, TRUE);
  WdfInterruptAcquireLock (D->Interrupt);
  Or (D, DCTL, DCTL_SFTDISCON);
  Wr (D, GINTMSK, 0);
  WdfInterruptReleaseLock (D->Interrupt);
  return STATUS_SUCCESS;
}

NTSTATUS
EvtPrepareHardware (WDFDEVICE Device, WDFCMRESLIST ResourcesRaw, WDFCMRESLIST ResourcesTranslated)
{
  PDEVICE_CONTEXT                  D;
  PCM_PARTIAL_RESOURCE_DESCRIPTOR  Desc;
  ULONG                            Index;
  ULONG                            Count;
  ULONG_PTR                        Gsi;

  D     = GetDeviceContext (Device);
  Gsi   = 0;
  Count = WdfCmResourceListGetCount (ResourcesTranslated);
  for (Index = 0; Index < Count; Index++) {
    Desc = WdfCmResourceListGetDescriptor (ResourcesTranslated, Index);
    if ((Desc->Type == CmResourceTypeMemory) && (D->Base == NULL)) {
      D->Size = Desc->u.Memory.Length;
      D->Base = MmMapIoSpace (Desc->u.Memory.Start, D->Size, MmNonCached);
      if (D->Base == NULL) {
        return STATUS_INSUFFICIENT_RESOURCES;
      }
    } else if (Desc->Type == CmResourceTypeInterrupt) {
      Gsi = WdfCmResourceListGetDescriptor (ResourcesRaw, Index)->u.Interrupt.Vector;
    }
  }
  if (D->Base == NULL) {
    OsLog ("exynosusb: no memory resource");
    return STATUS_DEVICE_CONFIGURATION_ERROR;
  }
  OsLog ("exynosusb: controller mapped, GSI %Iu, %lu resources", Gsi, Count);

#if LOAD_TEST
  KeBugCheckEx (USB_BUGCHECK,
                (ULONG_PTR)WdfCmResourceListGetDescriptor (ResourcesRaw, 0)->u.Memory.Start.QuadPart,
                Gsi, Count, 0);
#endif
  return STATUS_SUCCESS;
}

NTSTATUS
EvtReleaseHardware (WDFDEVICE Device, WDFCMRESLIST ResourcesTranslated)
{
  PDEVICE_CONTEXT  D;

  UNREFERENCED_PARAMETER (ResourcesTranslated);

  D = GetDeviceContext (Device);
  if (D->Base != NULL) {
    MmUnmapIoSpace (D->Base, D->Size);
    D->Base = NULL;
  }
  return STATUS_SUCCESS;
}

NTSTATUS
EvtDeviceAdd (WDFDRIVER Driver, PWDFDEVICE_INIT DeviceInit)
{
  WDF_PNPPOWER_EVENT_CALLBACKS  PnpPower;
  WDF_OBJECT_ATTRIBUTES         Attributes;
  WDF_INTERRUPT_CONFIG          InterruptConfig;
  WDF_TIMER_CONFIG              TimerConfig;
  WDF_WORKITEM_CONFIG           WorkConfig;
  PDEVICE_CONTEXT               D;
  WDFDEVICE                     Device;
  NTSTATUS                      Status;

  UNREFERENCED_PARAMETER (Driver);

  WDF_PNPPOWER_EVENT_CALLBACKS_INIT (&PnpPower);
  PnpPower.EvtDevicePrepareHardware = EvtPrepareHardware;
  PnpPower.EvtDeviceReleaseHardware = EvtReleaseHardware;
  PnpPower.EvtDeviceD0Entry         = EvtD0Entry;
  PnpPower.EvtDeviceD0Exit          = EvtD0Exit;
  WdfDeviceInitSetPnpPowerEventCallbacks (DeviceInit, &PnpPower);

  WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE (&Attributes, DEVICE_CONTEXT);
  Status = WdfDeviceCreate (&DeviceInit, &Attributes, &Device);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }
  D = GetDeviceContext (Device);

  WDF_INTERRUPT_CONFIG_INIT (&InterruptConfig, EvtIsr, EvtDpc);
  Status = WdfInterruptCreate (Device, &InterruptConfig, WDF_NO_OBJECT_ATTRIBUTES, &D->Interrupt);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }

  WDF_TIMER_CONFIG_INIT_PERIODIC (&TimerConfig, EvtTimer, POLL_MS);
  WDF_OBJECT_ATTRIBUTES_INIT (&Attributes);
  Attributes.ParentObject = Device;
  Status = WdfTimerCreate (&TimerConfig, &Attributes, &D->Timer);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }

  WDF_WORKITEM_CONFIG_INIT (&WorkConfig, EvtResetWork);
  WDF_OBJECT_ATTRIBUTES_INIT (&Attributes);
  Attributes.ParentObject = Device;
  return WdfWorkItemCreate (&WorkConfig, &Attributes, &D->ResetWork);
}

NTSTATUS
DriverEntry (PDRIVER_OBJECT DriverObject, PUNICODE_STRING RegistryPath)
{
  WDF_DRIVER_CONFIG  Config;

  OsLogInit ();
  OsLog ("exynosusb: DriverEntry");
  WDF_DRIVER_CONFIG_INIT (&Config, EvtDeviceAdd);
  return WdfDriverCreate (DriverObject, RegistryPath, WDF_NO_OBJECT_ATTRIBUTES, &Config, WDF_NO_HANDLE);
}
