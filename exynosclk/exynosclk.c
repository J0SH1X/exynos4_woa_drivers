/** @file
  Exynos4210 clock controller (CMU) for Windows RT 8.1 on the GT-N7000.

  Serves EXYNOS_CLOCK_INTERFACE (common/exynosclk.h) to other drivers via
  IRP_MN_QUERY_INTERFACE: a port of UEFI's ClockDxe.  Special clocks are
  mux -> ratio -> pre-ratio chains (dout_mmcN / sclk_mmcN in the kernel's
  clock-exynos4.c); gates are single bits in CLKGATE_IP_*.  Only MPLL and
  the 24 MHz crystals are computed as mux parents.

  ACPI: _HID EXYN0005 (CLK0), Memory32Fixed for the CMU (0x10030000).

  With LOAD_TEST it bugchecks 0x434C4B21 ("CLK!") in PrepareHardware with
  P1 = memory base, P2 = MPLL rate, P3 = resource count.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include <ntddk.h>
#include <wdf.h>
#include <initguid.h>
#define EXYNOS_CLK_SERVER
#include "exynosclk.h"
#include "oslog.h"

#ifndef LOAD_TEST
#define LOAD_TEST 0
#endif
#define CLK_BUGCHECK  0x434C4B21

//
// Offsets from the CMU base (ClockDxe.c).
//
#define CLKSRC_FSYS        0xC240
#define CLKSRC_MASK_FSYS   0xC340
#define CLKDIV_FSYS2       0xC548
#define CLKDIV_FSYS3       0xC54C
#define CLKDIV_STAT_FSYS2  0xC648
#define CLKDIV_STAT_FSYS3  0xC64C
#define CLKGATE_IP_FSYS    0xC940
#define CLKGATE_IP_PERIL   0xC950
#define MPLL_CON0          0x14108     // Exynos4210: in CMU_DMC
#define CMU_MIN_SIZE       0x15000

#define XTAL_HZ    24000000u
#define MUX_XXTI   0
#define MUX_XUSBXTI 1
#define MUX_MPLL   6

typedef struct {
  ULONG  SrcReg;      // 0: gate only
  UCHAR  SrcShift;
  ULONG  MaskReg;
  UCHAR  MaskBit;
  ULONG  DivReg;
  UCHAR  RatioShift;  // 4-bit ratio
  UCHAR  PreShift;    // 8-bit pre-ratio
  ULONG  StatReg;
  ULONG  GateReg;
  UCHAR  GateBit;
} CLOCK_DESC;

static const CLOCK_DESC mClocks[ExynosClkMax] = {
  [ExynosClkSclkMmc2]   = { CLKSRC_FSYS, 8,  CLKSRC_MASK_FSYS, 8,  CLKDIV_FSYS2, 0, 8, CLKDIV_STAT_FSYS2, 0, 0 },
  [ExynosClkSclkDwmci]  = { CLKSRC_FSYS, 16, CLKSRC_MASK_FSYS, 16, CLKDIV_FSYS3, 0, 8, CLKDIV_STAT_FSYS3, 0, 0 },
  [ExynosClkGateSdmmc2] = { 0, 0, 0, 0, 0, 0, 0, 0, CLKGATE_IP_FSYS, 7 },
  [ExynosClkGateDwmci]  = { 0, 0, 0, 0, 0, 0, 0, 0, CLKGATE_IP_FSYS, 9 },
  [ExynosClkGateUsbOtg] = { 0, 0, 0, 0, 0, 0, 0, 0, CLKGATE_IP_FSYS, 13 },
  [ExynosClkGateI2c0]   = { 0, 0, 0, 0, 0, 0, 0, 0, CLKGATE_IP_PERIL, 6 },
  [ExynosClkGateI2c1]   = { 0, 0, 0, 0, 0, 0, 0, 0, CLKGATE_IP_PERIL, 7 },
  [ExynosClkGateI2c2]   = { 0, 0, 0, 0, 0, 0, 0, 0, CLKGATE_IP_PERIL, 8 },
  [ExynosClkGateI2c3]   = { 0, 0, 0, 0, 0, 0, 0, 0, CLKGATE_IP_PERIL, 9 },
  [ExynosClkGateI2c4]   = { 0, 0, 0, 0, 0, 0, 0, 0, CLKGATE_IP_PERIL, 10 },
  [ExynosClkGateI2c5]   = { 0, 0, 0, 0, 0, 0, 0, 0, CLKGATE_IP_PERIL, 11 },
  [ExynosClkGateI2c6]   = { 0, 0, 0, 0, 0, 0, 0, 0, CLKGATE_IP_PERIL, 12 },
  [ExynosClkGateI2c7]   = { 0, 0, 0, 0, 0, 0, 0, 0, CLKGATE_IP_PERIL, 13 },
};

typedef struct {
  PUCHAR      Base;
  SIZE_T      Size;
  KSPIN_LOCK  Lock;
} DEVICE_CONTEXT, *PDEVICE_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME (DEVICE_CONTEXT, GetDeviceContext)

DRIVER_INITIALIZE                DriverEntry;
EVT_WDF_DRIVER_DEVICE_ADD        EvtDeviceAdd;
EVT_WDF_DEVICE_PREPARE_HARDWARE  EvtPrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE  EvtReleaseHardware;

static ULONG Rd (PDEVICE_CONTEXT C, ULONG R)          { return READ_REGISTER_ULONG ((PULONG)(C->Base + R)); }
static VOID  Wr (PDEVICE_CONTEXT C, ULONG R, ULONG V) { WRITE_REGISTER_ULONG ((PULONG)(C->Base + R), V); }

/**
  Exynos4210 MPLL is a PLL4508: Fout = Fin * M / (P << (S - 1)).
**/
static ULONG
MpllRate (PDEVICE_CONTEXT C)
{
  ULONG  Con;
  ULONG  M;
  ULONG  P;
  ULONG  S;

  Con = Rd (C, MPLL_CON0);
  M   = (Con >> 16) & 0x3FF;
  P   = (Con >> 8) & 0x3F;
  S   = Con & 0x7;
  if (P == 0) {
    return 0;
  }
  return (ULONG)(((ULONGLONG)XTAL_HZ * M) / (P << (S ? S - 1 : 0)));
}

static ULONG
MuxParentRate (PDEVICE_CONTEXT C, ULONG Mux)
{
  switch (Mux) {
    case MUX_XXTI:
    case MUX_XUSBXTI:
      return XTAL_HZ;
    case MUX_MPLL:
      return MpllRate (C);
    default:
      return 0;
  }
}

static NTSTATUS
ClockEnable (PVOID Context, EXYNOS_CLOCK_ID Id)
{
  PDEVICE_CONTEXT    C;
  const CLOCK_DESC  *Clk;
  KIRQL              Irql;

  if ((ULONG)Id >= ExynosClkMax) {
    return STATUS_INVALID_PARAMETER;
  }
  C   = Context;
  Clk = &mClocks[Id];
  KeAcquireSpinLock (&C->Lock, &Irql);
  if (Clk->GateReg != 0) {
    Wr (C, Clk->GateReg, Rd (C, Clk->GateReg) | (1u << Clk->GateBit));
  }
  if (Clk->MaskReg != 0) {
    Wr (C, Clk->MaskReg, Rd (C, Clk->MaskReg) | (1u << Clk->MaskBit));
  }
  KeReleaseSpinLock (&C->Lock, Irql);
  return STATUS_SUCCESS;
}

static NTSTATUS
ClockDisable (PVOID Context, EXYNOS_CLOCK_ID Id)
{
  PDEVICE_CONTEXT    C;
  const CLOCK_DESC  *Clk;
  KIRQL              Irql;

  if ((ULONG)Id >= ExynosClkMax) {
    return STATUS_INVALID_PARAMETER;
  }
  C   = Context;
  Clk = &mClocks[Id];
  KeAcquireSpinLock (&C->Lock, &Irql);
  if (Clk->GateReg != 0) {
    Wr (C, Clk->GateReg, Rd (C, Clk->GateReg) & ~(1u << Clk->GateBit));
  }
  if (Clk->MaskReg != 0) {
    Wr (C, Clk->MaskReg, Rd (C, Clk->MaskReg) & ~(1u << Clk->MaskBit));
  }
  KeReleaseSpinLock (&C->Lock, Irql);
  return STATUS_SUCCESS;
}

static NTSTATUS
ClockGetRate (PVOID Context, EXYNOS_CLOCK_ID Id, PULONG RateHz)
{
  PDEVICE_CONTEXT    C;
  const CLOCK_DESC  *Clk;
  ULONG              Parent;
  ULONG              Div;

  if (((ULONG)Id >= ExynosClkMax) || (RateHz == NULL)) {
    return STATUS_INVALID_PARAMETER;
  }
  C   = Context;
  Clk = &mClocks[Id];
  if (Clk->SrcReg == 0) {
    return STATUS_NOT_SUPPORTED;
  }

  Parent = MuxParentRate (C, (Rd (C, Clk->SrcReg) >> Clk->SrcShift) & 0xF);
  if (Parent == 0) {
    return STATUS_NOT_SUPPORTED;
  }
  Div     = Rd (C, Clk->DivReg);
  *RateHz = Parent / (((Div >> Clk->RatioShift) & 0xF) + 1) / (((Div >> Clk->PreShift) & 0xFF) + 1);
  return STATUS_SUCCESS;
}

static NTSTATUS
ClockSetRate (PVOID Context, EXYNOS_CLOCK_ID Id, ULONG RateHz, PULONG ActualHz)
{
  PDEVICE_CONTEXT    C;
  const CLOCK_DESC  *Clk;
  ULONG              Mpll;
  ULONG              Total;
  ULONG              Ratio;
  ULONG              Pre;
  ULONG              Retry;
  KIRQL              Irql;

  if (((ULONG)Id >= ExynosClkMax) || (RateHz == 0)) {
    return STATUS_INVALID_PARAMETER;
  }
  C   = Context;
  Clk = &mClocks[Id];
  if (Clk->SrcReg == 0) {
    return STATUS_NOT_SUPPORTED;
  }

  Mpll = MpllRate (C);
  if (Mpll == 0) {
    return STATUS_DEVICE_NOT_READY;
  }

  // Smallest total divider giving <= RateHz: ratio 1..16, pre-ratio 1..256.
  Total = (Mpll + RateHz - 1) / RateHz;
  Ratio = min (Total, 16);
  Pre   = (Total + Ratio - 1) / Ratio;
  if (Pre > 256) {
    return STATUS_INVALID_PARAMETER;
  }

  KeAcquireSpinLock (&C->Lock, &Irql);
  Wr (C, Clk->SrcReg, (Rd (C, Clk->SrcReg) & ~(0xFu << Clk->SrcShift)) | (MUX_MPLL << Clk->SrcShift));
  Wr (C, Clk->MaskReg, Rd (C, Clk->MaskReg) | (1u << Clk->MaskBit));
  Wr (C, Clk->DivReg, (Rd (C, Clk->DivReg) & ~((0xFu << Clk->RatioShift) | (0xFFu << Clk->PreShift))) |
                      ((Ratio - 1) << Clk->RatioShift) | ((Pre - 1) << Clk->PreShift));
  for (Retry = 0; Retry < 1000; Retry++) {
    if ((Rd (C, Clk->StatReg) & ((1u << Clk->RatioShift) | (1u << Clk->PreShift))) == 0) {
      break;
    }
    KeStallExecutionProcessor (10);
  }
  KeReleaseSpinLock (&C->Lock, Irql);

  if (ActualHz != NULL) {
    *ActualHz = Mpll / (Ratio * Pre);
  }
  OsLog ("exynosclk: clock %d -> %lu Hz (asked %lu)", Id, Mpll / (Ratio * Pre), RateHz);
  return STATUS_SUCCESS;
}

NTSTATUS
EvtPrepareHardware (WDFDEVICE Device, WDFCMRESLIST ResourcesRaw, WDFCMRESLIST ResourcesTranslated)
{
  PDEVICE_CONTEXT                  C;
  PCM_PARTIAL_RESOURCE_DESCRIPTOR  Desc;
  ULONG                            Index;
  ULONG                            Count;

  C     = GetDeviceContext (Device);
  Count = WdfCmResourceListGetCount (ResourcesTranslated);
  for (Index = 0; Index < Count; Index++) {
    Desc = WdfCmResourceListGetDescriptor (ResourcesTranslated, Index);
    if ((Desc->Type == CmResourceTypeMemory) && (C->Base == NULL)) {
      C->Size = Desc->u.Memory.Length;
      if (C->Size < CMU_MIN_SIZE) {
        OsLog ("exynosclk: CMU window 0x%Ix too small", C->Size);
        return STATUS_DEVICE_CONFIGURATION_ERROR;
      }
      C->Base = MmMapIoSpace (Desc->u.Memory.Start, C->Size, MmNonCached);
      if (C->Base == NULL) {
        return STATUS_INSUFFICIENT_RESOURCES;
      }
    }
  }
  if (C->Base == NULL) {
    OsLog ("exynosclk: no memory resource");
    return STATUS_DEVICE_CONFIGURATION_ERROR;
  }
  OsLog ("exynosclk: CMU mapped, MPLL %lu Hz", MpllRate (C));

#if LOAD_TEST
  KeBugCheckEx (CLK_BUGCHECK,
                (ULONG_PTR)WdfCmResourceListGetDescriptor (ResourcesRaw, 0)->u.Memory.Start.QuadPart,
                MpllRate (C), Count, 0);
#else
  UNREFERENCED_PARAMETER (ResourcesRaw);
#endif
  return STATUS_SUCCESS;
}

NTSTATUS
EvtReleaseHardware (WDFDEVICE Device, WDFCMRESLIST ResourcesTranslated)
{
  PDEVICE_CONTEXT  C;

  UNREFERENCED_PARAMETER (ResourcesTranslated);

  C = GetDeviceContext (Device);
  if (C->Base != NULL) {
    MmUnmapIoSpace (C->Base, C->Size);
    C->Base = NULL;
  }
  return STATUS_SUCCESS;
}

NTSTATUS
EvtDeviceAdd (WDFDRIVER Driver, PWDFDEVICE_INIT DeviceInit)
{
  WDF_PNPPOWER_EVENT_CALLBACKS  PnpPower;
  WDF_OBJECT_ATTRIBUTES         Attributes;
  WDF_QUERY_INTERFACE_CONFIG    QiConfig;
  EXYNOS_CLOCK_INTERFACE        Iface;
  PDEVICE_CONTEXT               C;
  WDFDEVICE                     Device;
  NTSTATUS                      Status;

  UNREFERENCED_PARAMETER (Driver);

  WDF_PNPPOWER_EVENT_CALLBACKS_INIT (&PnpPower);
  PnpPower.EvtDevicePrepareHardware = EvtPrepareHardware;
  PnpPower.EvtDeviceReleaseHardware = EvtReleaseHardware;
  WdfDeviceInitSetPnpPowerEventCallbacks (DeviceInit, &PnpPower);

  WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE (&Attributes, DEVICE_CONTEXT);
  Status = WdfDeviceCreate (&DeviceInit, &Attributes, &Device);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }
  C = GetDeviceContext (Device);
  KeInitializeSpinLock (&C->Lock);

  RtlZeroMemory (&Iface, sizeof (Iface));
  Iface.Header.Size                 = sizeof (Iface);
  Iface.Header.Version              = EXYNOS_CLOCK_INTERFACE_VERSION;
  Iface.Header.Context              = C;
  Iface.Header.InterfaceReference   = WdfDeviceInterfaceReferenceNoOp;
  Iface.Header.InterfaceDereference = WdfDeviceInterfaceDereferenceNoOp;
  Iface.Enable  = ClockEnable;
  Iface.Disable = ClockDisable;
  Iface.GetRate = ClockGetRate;
  Iface.SetRate = ClockSetRate;

  WDF_QUERY_INTERFACE_CONFIG_INIT (&QiConfig, (PINTERFACE)&Iface, &GUID_EXYNOS_CLOCK_INTERFACE, NULL);
  Status = WdfDeviceAddQueryInterface (Device, &QiConfig);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }

  return WdfDeviceCreateDeviceInterface (Device, &GUID_DEVINTERFACE_EXYNOS_CLOCK, NULL);
}

NTSTATUS
DriverEntry (PDRIVER_OBJECT DriverObject, PUNICODE_STRING RegistryPath)
{
  WDF_DRIVER_CONFIG  Config;

  OsLogInit ();
  OsLog ("exynosclk: DriverEntry");
  WDF_DRIVER_CONFIG_INIT (&Config, EvtDeviceAdd);
  return WdfDriverCreate (DriverObject, RegistryPath, WDF_NO_OBJECT_ATTRIBUTES, &Config, WDF_NO_HANDLE);
}
