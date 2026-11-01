/** @file
  Exynos4 GPIO controller, GPX0 bank (EINT0-7), as a GpioClx client for the
  GT-N7000 under Windows RT 8.1 (KMDF 1.13).

  ACPI: _HID EXYN0003, Memory32Fixed for GPIO part 2 (0x11000000) and one
  interrupt per pin: EINT0-7 each have their own SPI (16-23).  GpioClx
  pairs bank N with the N-th interrupt resource, so the 8 pins are reported
  as 8 banks of one pin; pin N of the controller is GPX0(N).

  Only GPX0 for now: it carries the touchscreen's CHG line (pin 4).

  With LOAD_TEST it bugchecks 0x4750494F ("GPIO") in PrepareController with
  P1 = memory base, P2 = first interrupt (raw GSI), P3 = resource count.
  A failed GpioClx registration is logged and fails the driver.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include <ntddk.h>
#include <wdf.h>
#include <gpioclx.h>
#include "oslog.h"

#ifndef LOAD_TEST
#define LOAD_TEST 0
#endif
#define GPIO_BUGCHECK  0x4750494F

#define GPIO_PINS  8

//
// Registers, offsets from GPIO part 2 (regs-gpio.h).
//
#define GPX0CON      0xC00    // 4 bits/pin: 0 input, 1 output, 0xF EINT
#define GPX0DAT      0xC04
#define GPX0PUD      0xC08    // 2 bits/pin: 0 none, 1 down, 3 up
#define EINT40_CON   0xE00    // 4 bits/pin, see EINT_*
#define EINT40_MASK  0xF00
#define EINT40_PEND  0xF40    // write 1 to clear

#define GPX_FUNC_INPUT   0x0
#define GPX_FUNC_OUTPUT  0x1
#define GPX_FUNC_EINT    0xF

#define EINT_LEVEL_LOW     0x0
#define EINT_LEVEL_HIGH    0x1
#define EINT_EDGE_FALLING  0x2
#define EINT_EDGE_RISING   0x3
#define EINT_EDGE_BOTH     0x4

#define PUD_NONE  0x0
#define PUD_DOWN  0x1
#define PUD_UP    0x3

typedef struct {
  PUCHAR  Base;
  SIZE_T  Size;
} GPIO_CONTEXT, *PGPIO_CONTEXT;

DRIVER_INITIALIZE          DriverEntry;
EVT_WDF_DRIVER_DEVICE_ADD  EvtDeviceAdd;
EVT_WDF_DRIVER_UNLOAD      EvtDriverUnload;

static ULONG
RegRead (PGPIO_CONTEXT Ctx, ULONG Reg)
{
  return READ_REGISTER_ULONG ((PULONG)(Ctx->Base + Reg));
}

static VOID
RegWrite (PGPIO_CONTEXT Ctx, ULONG Reg, ULONG Value)
{
  WRITE_REGISTER_ULONG ((PULONG)(Ctx->Base + Reg), Value);
}

static VOID
RegField (PGPIO_CONTEXT Ctx, ULONG Reg, ULONG Shift, ULONG Width, ULONG Value)
{
  ULONG  Mask;

  Mask = ((1u << Width) - 1) << Shift;
  RegWrite (Ctx, Reg, (RegRead (Ctx, Reg) & ~Mask) | ((Value << Shift) & Mask));
}

static ULONG
PullValue (UCHAR PullConfiguration)
{
  switch (PullConfiguration) {
    case GPIO_PIN_PULL_CONFIGURATION_PULLUP:   return PUD_UP;
    case GPIO_PIN_PULL_CONFIGURATION_PULLDOWN: return PUD_DOWN;
    default:                                   return PUD_NONE;
  }
}

static ULONG
EintType (KINTERRUPT_MODE Mode, KINTERRUPT_POLARITY Polarity)
{
  if (Mode == LevelSensitive) {
    return (Polarity == InterruptActiveLow) ? EINT_LEVEL_LOW : EINT_LEVEL_HIGH;
  }
  switch (Polarity) {
    case InterruptActiveLow:  return EINT_EDGE_FALLING;
    case InterruptActiveBoth: return EINT_EDGE_BOTH;
    default:                  return EINT_EDGE_RISING;
  }
}

//
// One pin per bank: the controller pin is the bank ID.
//
#define PIN(BankId)  ((ULONG)(BankId))

static NTSTATUS GPIO_EXPORT
GpioPrepareController (WDFDEVICE Device, PVOID Context, WDFCMRESLIST ResourcesRaw, WDFCMRESLIST ResourcesTranslated)
{
  PGPIO_CONTEXT                    Ctx;
  PCM_PARTIAL_RESOURCE_DESCRIPTOR  Desc;
  ULONG                            Index;
  ULONG                            Count;
  ULONG_PTR                        Gsi;

  UNREFERENCED_PARAMETER (Device);

  Ctx   = Context;
  Gsi   = 0;
  Count = WdfCmResourceListGetCount (ResourcesTranslated);
  for (Index = 0; Index < Count; Index++) {
    Desc = WdfCmResourceListGetDescriptor (ResourcesTranslated, Index);
    if ((Desc->Type == CmResourceTypeMemory) && (Ctx->Base == NULL)) {
      Ctx->Size = Desc->u.Memory.Length;
      Ctx->Base = MmMapIoSpace (Desc->u.Memory.Start, Ctx->Size, MmNonCached);
      if (Ctx->Base == NULL) {
        return STATUS_INSUFFICIENT_RESOURCES;
      }
    } else if ((Desc->Type == CmResourceTypeInterrupt) && (Gsi == 0)) {
      Gsi = WdfCmResourceListGetDescriptor (ResourcesRaw, Index)->u.Interrupt.Vector;
    }
  }

  if (Ctx->Base == NULL) {
    OsLog ("exynosgpio: no memory resource");
    return STATUS_DEVICE_CONFIGURATION_ERROR;
  }
  OsLog ("exynosgpio: controller mapped, first GSI %Iu, %lu resources", Gsi, Count);

#if LOAD_TEST
  KeBugCheckEx (GPIO_BUGCHECK,
                (ULONG_PTR)WdfCmResourceListGetDescriptor (ResourcesRaw, 0)->u.Memory.Start.QuadPart,
                Gsi, Count, 0);
#else
  UNREFERENCED_PARAMETER (Gsi);
#endif
  return STATUS_SUCCESS;
}

static NTSTATUS GPIO_EXPORT
GpioReleaseController (WDFDEVICE Device, PVOID Context)
{
  PGPIO_CONTEXT  Ctx;

  UNREFERENCED_PARAMETER (Device);

  Ctx = Context;
  if (Ctx->Base != NULL) {
    MmUnmapIoSpace (Ctx->Base, Ctx->Size);
    Ctx->Base = NULL;
  }
  return STATUS_SUCCESS;
}

/**
  Nothing may fire before GpioClx enables it: mask all EINTs and drop
  whatever UEFI left pending (its touch driver uses EINT4).
**/
static NTSTATUS GPIO_EXPORT
GpioStartController (PVOID Context, BOOLEAN RestoreContext, WDF_POWER_DEVICE_STATE PreviousPowerState)
{
  PGPIO_CONTEXT  Ctx;

  UNREFERENCED_PARAMETER (PreviousPowerState);

  Ctx = Context;
  if (!RestoreContext) {
    RegWrite (Ctx, EINT40_MASK, RegRead (Ctx, EINT40_MASK) | 0xFF);
    RegWrite (Ctx, EINT40_PEND, 0xFF);
  }
  return STATUS_SUCCESS;
}

static NTSTATUS GPIO_EXPORT
GpioStopController (PVOID Context, BOOLEAN SaveContext, WDF_POWER_DEVICE_STATE TargetState)
{
  UNREFERENCED_PARAMETER (Context);
  UNREFERENCED_PARAMETER (SaveContext);
  UNREFERENCED_PARAMETER (TargetState);
  return STATUS_SUCCESS;
}

static NTSTATUS GPIO_EXPORT
GpioQueryControllerBasicInformation (PVOID Context, PCLIENT_CONTROLLER_BASIC_INFORMATION Information)
{
  UNREFERENCED_PARAMETER (Context);

  Information->Version             = GPIO_CONTROLLER_BASIC_INFORMATION_VERSION;
  Information->Size                = sizeof (*Information);
  Information->TotalPins           = GPIO_PINS;
  Information->NumberOfPinsPerBank = 1;
  Information->Flags.MemoryMappedController   = 1;
  Information->Flags.FormatIoRequestsAsMasks  = 1;
  Information->Flags.EmulateDebouncing        = 1;
  return STATUS_SUCCESS;
}

static NTSTATUS GPIO_EXPORT
GpioEnableInterrupt (PVOID Context, PGPIO_ENABLE_INTERRUPT_PARAMETERS Params)
{
  PGPIO_CONTEXT  Ctx;
  ULONG          Pin;

  Ctx = Context;
  Pin = PIN (Params->BankId);
  OsLog ("exynosgpio: enable GPX0(%lu) %s active %s", Pin,
         (Params->InterruptMode == LevelSensitive) ? "level" : "edge",
         (Params->Polarity == InterruptActiveLow) ? "low" : "high");

  RegWrite (Ctx, EINT40_MASK, RegRead (Ctx, EINT40_MASK) | (1u << Pin));
  RegField (Ctx, GPX0PUD, Pin * 2, 2, PullValue (Params->PullConfiguration));
  RegField (Ctx, EINT40_CON, Pin * 4, 3, EintType (Params->InterruptMode, Params->Polarity));
  RegField (Ctx, GPX0CON, Pin * 4, 4, GPX_FUNC_EINT);
  RegWrite (Ctx, EINT40_PEND, 1u << Pin);
  RegWrite (Ctx, EINT40_MASK, RegRead (Ctx, EINT40_MASK) & ~(1u << Pin));
  return STATUS_SUCCESS;
}

static NTSTATUS GPIO_EXPORT
GpioDisableInterrupt (PVOID Context, PGPIO_DISABLE_INTERRUPT_PARAMETERS Params)
{
  PGPIO_CONTEXT  Ctx;
  ULONG          Pin;

  Ctx = Context;
  Pin = PIN (Params->BankId);
  RegWrite (Ctx, EINT40_MASK, RegRead (Ctx, EINT40_MASK) | (1u << Pin));
  RegWrite (Ctx, EINT40_PEND, 1u << Pin);
  return STATUS_SUCCESS;
}

static NTSTATUS GPIO_EXPORT
GpioMaskInterrupts (PVOID Context, PGPIO_MASK_INTERRUPT_PARAMETERS Params)
{
  PGPIO_CONTEXT  Ctx;

  Ctx = Context;
  if ((Params->PinMask & 1) != 0) {
    RegWrite (Ctx, EINT40_MASK, RegRead (Ctx, EINT40_MASK) | (1u << PIN (Params->BankId)));
  }
  Params->FailedMask = 0;
  return STATUS_SUCCESS;
}

static NTSTATUS GPIO_EXPORT
GpioUnmaskInterrupt (PVOID Context, PGPIO_ENABLE_INTERRUPT_PARAMETERS Params)
{
  PGPIO_CONTEXT  Ctx;

  Ctx = Context;
  RegWrite (Ctx, EINT40_MASK, RegRead (Ctx, EINT40_MASK) & ~(1u << PIN (Params->BankId)));
  return STATUS_SUCCESS;
}

static NTSTATUS GPIO_EXPORT
GpioQueryActiveInterrupts (PVOID Context, PGPIO_QUERY_ACTIVE_INTERRUPTS_PARAMETERS Params)
{
  PGPIO_CONTEXT  Ctx;

  Ctx = Context;
  Params->ActiveMask = ((RegRead (Ctx, EINT40_PEND) >> PIN (Params->BankId)) & 1) & Params->EnabledMask;
  return STATUS_SUCCESS;
}

static NTSTATUS GPIO_EXPORT
GpioClearActiveInterrupts (PVOID Context, PGPIO_CLEAR_ACTIVE_INTERRUPTS_PARAMETERS Params)
{
  PGPIO_CONTEXT  Ctx;

  Ctx = Context;
  if ((Params->ClearActiveMask & 1) != 0) {
    RegWrite (Ctx, EINT40_PEND, 1u << PIN (Params->BankId));
  }
  Params->FailedClearMask = 0;
  return STATUS_SUCCESS;
}

static NTSTATUS GPIO_EXPORT
GpioQueryEnabledInterrupts (PVOID Context, PGPIO_QUERY_ENABLED_INTERRUPTS_PARAMETERS Params)
{
  PGPIO_CONTEXT  Ctx;
  ULONG          Pin;

  Ctx = Context;
  Pin = PIN (Params->BankId);
  Params->EnabledMask = (((RegRead (Ctx, GPX0CON) >> (Pin * 4)) & 0xF) == GPX_FUNC_EINT) &&
                        (((RegRead (Ctx, EINT40_MASK) >> Pin) & 1) == 0);
  return STATUS_SUCCESS;
}

static NTSTATUS GPIO_EXPORT
GpioReconfigureInterrupt (PVOID Context, PGPIO_RECONFIGURE_INTERRUPTS_PARAMETERS Params)
{
  PGPIO_CONTEXT  Ctx;
  ULONG          Pin;

  Ctx = Context;
  Pin = PIN (Params->BankId);
  RegField (Ctx, EINT40_CON, Pin * 4, 3, EintType (Params->InterruptMode, Params->Polarity));
  RegWrite (Ctx, EINT40_PEND, 1u << Pin);
  return STATUS_SUCCESS;
}

static NTSTATUS GPIO_EXPORT
GpioConnectIoPins (PVOID Context, PGPIO_CONNECT_IO_PINS_PARAMETERS Params)
{
  PGPIO_CONTEXT  Ctx;
  ULONG          Pin;
  ULONG          Func;

  Ctx  = Context;
  Pin  = PIN (Params->BankId);
  Func = (Params->ConnectMode == ConnectModeOutput) ? GPX_FUNC_OUTPUT : GPX_FUNC_INPUT;

  RegField (Ctx, GPX0PUD, Pin * 2, 2, PullValue (Params->PullConfiguration));
  RegField (Ctx, GPX0CON, Pin * 4, 4, Func);
  return STATUS_SUCCESS;
}

static NTSTATUS GPIO_EXPORT
GpioDisconnectIoPins (PVOID Context, PGPIO_DISCONNECT_IO_PINS_PARAMETERS Params)
{
  PGPIO_CONTEXT  Ctx;

  Ctx = Context;
  if ((Params->DisconnectFlags.PreserveConfiguration) == 0) {
    RegField (Ctx, GPX0CON, PIN (Params->BankId) * 4, 4, GPX_FUNC_INPUT);
  }
  return STATUS_SUCCESS;
}

static NTSTATUS GPIO_EXPORT
GpioReadPins (PVOID Context, PGPIO_READ_PINS_MASK_PARAMETERS Params)
{
  PGPIO_CONTEXT  Ctx;

  Ctx = Context;
  *Params->PinValues = (RegRead (Ctx, GPX0DAT) >> PIN (Params->BankId)) & 1;
  return STATUS_SUCCESS;
}

static NTSTATUS GPIO_EXPORT
GpioWritePins (PVOID Context, PGPIO_WRITE_PINS_MASK_PARAMETERS Params)
{
  PGPIO_CONTEXT  Ctx;
  ULONG          Bit;
  ULONG          Dat;

  Ctx = Context;
  Bit = 1u << PIN (Params->BankId);
  Dat = RegRead (Ctx, GPX0DAT);
  if ((Params->SetMask & 1) != 0) {
    Dat |= Bit;
  }
  if ((Params->ClearMask & 1) != 0) {
    Dat &= ~Bit;
  }
  RegWrite (Ctx, GPX0DAT, Dat);
  return STATUS_SUCCESS;
}

NTSTATUS
EvtDeviceAdd (WDFDRIVER Driver, PWDFDEVICE_INIT DeviceInit)
{
  WDF_OBJECT_ATTRIBUTES  Attributes;
  WDFDEVICE              Device;
  NTSTATUS               Status;

  Status = GPIO_CLX_ProcessAddDevicePreDeviceCreate (Driver, DeviceInit, &Attributes);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }

  Status = WdfDeviceCreate (&DeviceInit, &Attributes, &Device);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }

  return GPIO_CLX_ProcessAddDevicePostDeviceCreate (Driver, Device);
}

VOID
EvtDriverUnload (WDFDRIVER Driver)
{
  GPIO_CLX_UnregisterClient (Driver);
}

NTSTATUS
DriverEntry (PDRIVER_OBJECT DriverObject, PUNICODE_STRING RegistryPath)
{
  WDF_DRIVER_CONFIG                Config;
  WDFDRIVER                        Driver;
  GPIO_CLIENT_REGISTRATION_PACKET  Packet;
  NTSTATUS                         Status;

  OsLogInit ();
  OsLog ("exynosgpio: DriverEntry");
  WDF_DRIVER_CONFIG_INIT (&Config, EvtDeviceAdd);
  Config.EvtDriverUnload = EvtDriverUnload;
  Status = WdfDriverCreate (DriverObject, RegistryPath, WDF_NO_OBJECT_ATTRIBUTES, &Config, &Driver);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }

  //
  // Version 2 (Windows 8.1): this header's packet is version 4, so register
  // only the part an 8.1 GpioClx knows.
  //
  RtlZeroMemory (&Packet, sizeof (Packet));
  Packet.Version               = 2;
  Packet.Size                  = FIELD_OFFSET (GPIO_CLIENT_REGISTRATION_PACKET, CLIENT_ConnectFunctionConfigPins);
  Packet.ControllerContextSize = sizeof (GPIO_CONTEXT);
  Packet.CLIENT_PrepareController              = GpioPrepareController;
  Packet.CLIENT_ReleaseController              = GpioReleaseController;
  Packet.CLIENT_StartController                = GpioStartController;
  Packet.CLIENT_StopController                 = GpioStopController;
  Packet.CLIENT_QueryControllerBasicInformation = GpioQueryControllerBasicInformation;
  Packet.CLIENT_EnableInterrupt                = GpioEnableInterrupt;
  Packet.CLIENT_DisableInterrupt               = GpioDisableInterrupt;
  Packet.CLIENT_UnmaskInterrupt                = GpioUnmaskInterrupt;
  Packet.CLIENT_MaskInterrupts                 = GpioMaskInterrupts;
  Packet.CLIENT_QueryActiveInterrupts          = GpioQueryActiveInterrupts;
  Packet.CLIENT_ClearActiveInterrupts          = GpioClearActiveInterrupts;
  Packet.CLIENT_ConnectIoPins                  = GpioConnectIoPins;
  Packet.CLIENT_DisconnectIoPins               = GpioDisconnectIoPins;
  Packet.CLIENT_ReadGpioPinsUsingMask          = GpioReadPins;
  Packet.CLIENT_WriteGpioPinsUsingMask         = GpioWritePins;
  Packet.CLIENT_ReconfigureInterrupt           = GpioReconfigureInterrupt;
  Packet.CLIENT_QueryEnabledInterrupts         = GpioQueryEnabledInterrupts;

  Status = GPIO_CLX_RegisterClient (Driver, &Packet, RegistryPath);
  if (!NT_SUCCESS (Status)) {
    OsLog ("exynosgpio: GpioClx registration v%u size %u failed: 0x%08lx", Packet.Version, Packet.Size, Status);
  }
  return Status;
}
