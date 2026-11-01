/** @file
  Exynos4 I2C controller (s3c2410-style) as an SPB controller for SpbCx,
  for the GT-N7000 under Windows RT 8.1 (KMDF 1.13, SPB 1.0).

  ACPI: _HID EXYN0002, one Memory32Fixed for the controller and its SPI.
  Pins and the clock gate are left set up by UEFI.

  Transfers are done by polling the byte-done (IRQPEND) bit, the same
  sequence as UEFI's I2CDxe (from u-boot / Linux s3c24xx_i2c), at PASSIVE
  level in the SpbCx I/O callbacks.  The controller interrupt is not used
  yet.

  With LOAD_TEST it bugchecks 0x49324321 ("I2C!") in PrepareHardware with
  P1 = memory base, P2 = interrupt (raw GSI), P3 = resource count.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include <ntddk.h>
#include <wdf.h>
#include <spb.h>
#include <spbcx.h>
#include <reshub.h>
#include "oslog.h"

#ifndef LOAD_TEST
#define LOAD_TEST 0
#endif
#define LOAD_TEST_BUGCHECK  0x49324321


//
// The I2C flavour of the resource hub's serial bus descriptor (ACPI
// I2cSerialBus), as the Windows I2C controller samples define it.
//
#include <pshpack1.h>
typedef struct _PNP_I2C_SERIAL_BUS_DESCRIPTOR {
  PNP_SERIAL_BUS_DESCRIPTOR  SerialBusDescriptor;
  ULONG                      ConnectionSpeed;
  USHORT                     SlaveAddress;
} PNP_I2C_SERIAL_BUS_DESCRIPTOR, *PPNP_I2C_SERIAL_BUS_DESCRIPTOR;
#include <poppack.h>

#define I2C_SERIAL_BUS_TYPE                         0x01
#define I2C_SERIAL_BUS_SPECIFIC_FLAG_10BIT_ADDRESS  0x0001

//
// Registers and bits (plat/regs-iic.h).
//
#define IICCON   0x00
#define IICSTAT  0x04
#define IICADD   0x08
#define IICDS    0x0C

#define IICCON_ACKEN    (1u << 7)
#define IICCON_TXDIV    (1u << 6)   // prescaler 16 (0) / 512 (1)
#define IICCON_IRQEN    (1u << 5)
#define IICCON_IRQPEND  (1u << 4)
#define IICCON_SCALE    0x0Fu

#define IICSTAT_MODE_RX  (2u << 6)
#define IICSTAT_MODE_TX  (3u << 6)
#define IICSTAT_START    (1u << 5)  // write: start/stop, read: bus busy
#define IICSTAT_TXRXEN   (1u << 4)
#define IICSTAT_NACK     (1u << 0)

#define I2C_INPUT_CLOCK_HZ  100000000   // ACLK_100
#define I2C_DEFAULT_HZ      100000
#define I2C_BYTE_TIMEOUT_US 10000

typedef struct {
  PUCHAR  Base;
  SIZE_T  Size;
} DEVICE_CONTEXT, *PDEVICE_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME (DEVICE_CONTEXT, GetDeviceContext)

typedef struct {
  USHORT  Address;
  ULONG   Speed;
} TARGET_CONTEXT, *PTARGET_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME (TARGET_CONTEXT, GetTargetContext)

typedef enum {
  XferOk,
  XferNack,
  XferTimeout
} XFER_RESULT;

DRIVER_INITIALIZE                DriverEntry;
EVT_WDF_DRIVER_DEVICE_ADD        EvtDeviceAdd;
EVT_WDF_DEVICE_PREPARE_HARDWARE  EvtPrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE  EvtReleaseHardware;
EVT_SPB_TARGET_CONNECT           EvtTargetConnect;
EVT_SPB_CONTROLLER_READ          EvtIoRead;
EVT_SPB_CONTROLLER_WRITE         EvtIoWrite;
EVT_SPB_CONTROLLER_SEQUENCE      EvtIoSequence;

static ULONG
RegRead (PDEVICE_CONTEXT Ctx, ULONG Reg)
{
  return READ_REGISTER_ULONG ((PULONG)(Ctx->Base + Reg));
}

static VOID
RegWrite (PDEVICE_CONTEXT Ctx, ULONG Reg, ULONG Value)
{
  WRITE_REGISTER_ULONG ((PULONG)(Ctx->Base + Reg), Value);
}

/**
  Prescaler/divider for the target's bus speed, then idle in master
  transmit mode (u-boot i2c_ch_init()).
**/
static VOID
I2cInit (PDEVICE_CONTEXT Ctx, ULONG Speed)
{
  ULONG  Pres;
  ULONG  Div;
  ULONG  Con;

  Pres = 16;
  if ((I2C_INPUT_CLOCK_HZ / Pres / (IICCON_SCALE + 1)) > Speed) {
    Pres = 512;
  }
  for (Div = 0; (Div < IICCON_SCALE) && ((I2C_INPUT_CLOCK_HZ / Pres / (Div + 1)) > Speed); Div++) {
  }

  Con = Div | IICCON_ACKEN | IICCON_IRQEN | ((Pres == 512) ? IICCON_TXDIV : 0);
  RegWrite (Ctx, IICCON, Con);
  RegWrite (Ctx, IICSTAT, 0);
  RegWrite (Ctx, IICADD, 0);
  RegWrite (Ctx, IICSTAT, IICSTAT_MODE_TX | IICSTAT_TXRXEN);
}

static XFER_RESULT
I2cWaitByte (PDEVICE_CONTEXT Ctx)
{
  ULONG  Us;

  for (Us = 0; Us < I2C_BYTE_TIMEOUT_US; Us++) {
    if ((RegRead (Ctx, IICCON) & IICCON_IRQPEND) != 0) {
      return ((RegRead (Ctx, IICSTAT) & IICSTAT_NACK) != 0) ? XferNack : XferOk;
    }
    KeStallExecutionProcessor (1);
  }
  return XferTimeout;
}

/**
  Clearing the pend bit acknowledges the finished byte and clocks the next.
**/
static VOID
I2cNextByte (PDEVICE_CONTEXT Ctx)
{
  RegWrite (Ctx, IICCON, RegRead (Ctx, IICCON) & ~IICCON_IRQPEND);
}

/**
  Bytes of one SPB transfer live in an MDL chain.
**/
typedef struct {
  PMDL    Mdl;
  PUCHAR  Va;
  SIZE_T  Offset;
} MDL_CURSOR;

static BOOLEAN
CursorNext (MDL_CURSOR *Cursor, PUCHAR *Byte)
{
  while (Cursor->Mdl != NULL) {
    if (Cursor->Va == NULL) {
      Cursor->Va = MmGetSystemAddressForMdlSafe (Cursor->Mdl, NormalPagePriority | MdlMappingNoExecute);
      if (Cursor->Va == NULL) {
        return FALSE;
      }
    }
    if (Cursor->Offset < MmGetMdlByteCount (Cursor->Mdl)) {
      *Byte = Cursor->Va + Cursor->Offset++;
      return TRUE;
    }
    Cursor->Mdl    = Cursor->Mdl->Next;
    Cursor->Va     = NULL;
    Cursor->Offset = 0;
  }
  return FALSE;
}

/**
  One transfer with a (repeated) start; the caller sends STOP.
**/
static XFER_RESULT
I2cTransfer (
  PDEVICE_CONTEXT  Ctx,
  USHORT           Address,
  BOOLEAN          IsRead,
  PMDL             Mdl,
  SIZE_T           Length,
  BOOLEAN          Repeated,
  SIZE_T           *Done
  )
{
  ULONG        Mode;
  XFER_RESULT  Result;
  MDL_CURSOR   Cursor;
  PUCHAR       Byte;
  SIZE_T       Index;

  *Done  = 0;
  Mode   = IsRead ? IICSTAT_MODE_RX : IICSTAT_MODE_TX;
  Cursor.Mdl    = Mdl;
  Cursor.Va     = NULL;
  Cursor.Offset = 0;

  // A previous read NACKed its last byte: ACK again.
  RegWrite (Ctx, IICCON, RegRead (Ctx, IICCON) | IICCON_ACKEN);

  // On a repeated start the previous byte's pend bit still holds SCL low;
  // clear it only once START is armed, or the address goes out as data.
  RegWrite (Ctx, IICSTAT, IICSTAT_TXRXEN | Mode);
  RegWrite (Ctx, IICDS, (ULONG)(Address << 1) | (IsRead ? 1 : 0));
  KeStallExecutionProcessor (1);
  RegWrite (Ctx, IICSTAT, IICSTAT_TXRXEN | IICSTAT_START | Mode);
  if (Repeated) {
    I2cNextByte (Ctx);
  }

  Result = I2cWaitByte (Ctx);
  if (Result != XferOk) {
    return Result;
  }

  for (Index = 0; Index < Length; Index++) {
    if (!CursorNext (&Cursor, &Byte)) {
      return XferTimeout;
    }
    if (IsRead) {
      if (Index == Length - 1) {
        // NACK the last byte so the slave releases the bus.
        RegWrite (Ctx, IICCON, RegRead (Ctx, IICCON) & ~IICCON_ACKEN);
      }
      I2cNextByte (Ctx);
      Result = I2cWaitByte (Ctx);
      *Byte  = (UCHAR)RegRead (Ctx, IICDS);
      if ((Result == XferNack) && (Index == Length - 1)) {
        Result = XferOk;
      }
    } else {
      RegWrite (Ctx, IICDS, *Byte);
      I2cNextByte (Ctx);
      Result = I2cWaitByte (Ctx);
    }
    if (Result != XferOk) {
      return Result;
    }
    (*Done)++;
  }

  return XferOk;
}

static VOID
I2cStop (PDEVICE_CONTEXT Ctx, BOOLEAN LastWasRead)
{
  RegWrite (Ctx, IICSTAT, IICSTAT_TXRXEN | (LastWasRead ? IICSTAT_MODE_RX : IICSTAT_MODE_TX));
  I2cNextByte (Ctx);
}

/**
  Run Count transfers of Request back to back and complete it.
**/
static VOID
I2cRunRequest (
  WDFDEVICE   Controller,
  SPBTARGET   Target,
  SPBREQUEST  Request,
  ULONG       Count
  )
{
  PDEVICE_CONTEXT          Ctx;
  PTARGET_CONTEXT          Tgt;
  SPB_TRANSFER_DESCRIPTOR  Desc;
  PMDL                     Mdl;
  ULONG                    Index;
  SIZE_T                   Done;
  SIZE_T                   Total;
  XFER_RESULT              Result;
  BOOLEAN                  IsRead;
  NTSTATUS                 Status;

  Ctx    = GetDeviceContext (Controller);
  Tgt    = GetTargetContext (Target);
  Total  = 0;
  Result = XferOk;
  IsRead = FALSE;

  I2cInit (Ctx, Tgt->Speed);

  for (Index = 0; (Index < Count) && (Result == XferOk); Index++) {
    SPB_TRANSFER_DESCRIPTOR_INIT (&Desc);
    SpbRequestGetTransferParameters (Request, Index, &Desc, &Mdl);
    if (Desc.DelayInUs != 0) {
      KeStallExecutionProcessor (Desc.DelayInUs);
    }
    IsRead = (Desc.Direction == SpbTransferDirectionFromDevice);
    Result = I2cTransfer (Ctx, Tgt->Address, IsRead, Mdl, Desc.TransferLength, Index != 0, &Done);
    Total += Done;
  }

  I2cStop (Ctx, IsRead);

  if (Result != XferOk) {
    OsLog ("exynosi2c: 0x%02x transfer %lu of %lu: %s after %Iu bytes", Tgt->Address, Index,
           Count, (Result == XferNack) ? "NACK" : "timeout", Total);
  }

  if (Result == XferOk) {
    Status = STATUS_SUCCESS;
  } else if ((Result == XferNack) && (Total == 0)) {
    Status = STATUS_NO_SUCH_DEVICE;       // address not acknowledged
  } else if (Result == XferNack) {
    Status = STATUS_SUCCESS;              // partial write: SPB reports the count
  } else {
    Status = STATUS_IO_TIMEOUT;
  }

  WdfRequestSetInformation ((WDFREQUEST)Request, Total);
  SpbRequestComplete (Request, Status);
}

VOID
EvtIoRead (WDFDEVICE Controller, SPBTARGET Target, SPBREQUEST Request, size_t Length)
{
  UNREFERENCED_PARAMETER (Length);
  I2cRunRequest (Controller, Target, Request, 1);
}

VOID
EvtIoWrite (WDFDEVICE Controller, SPBTARGET Target, SPBREQUEST Request, size_t Length)
{
  UNREFERENCED_PARAMETER (Length);
  I2cRunRequest (Controller, Target, Request, 1);
}

VOID
EvtIoSequence (WDFDEVICE Controller, SPBTARGET Target, SPBREQUEST Request, ULONG TransferCount)
{
  I2cRunRequest (Controller, Target, Request, TransferCount);
}

/**
  Pick the slave address and speed out of the target's I2cSerialBus
  connection descriptor.
**/
NTSTATUS
EvtTargetConnect (WDFDEVICE Controller, SPBTARGET Target)
{
  SPB_CONNECTION_PARAMETERS                     Params;
  PRH_QUERY_CONNECTION_PROPERTIES_OUTPUT_BUFFER Conn;
  PPNP_SERIAL_BUS_DESCRIPTOR                    Bus;
  PPNP_I2C_SERIAL_BUS_DESCRIPTOR                I2c;
  PTARGET_CONTEXT                               Tgt;

  UNREFERENCED_PARAMETER (Controller);

  OsLog ("exynosi2c: target connect");
  SPB_CONNECTION_PARAMETERS_INIT (&Params);
  SpbTargetGetConnectionParameters (Target, &Params);
  Conn = (PRH_QUERY_CONNECTION_PROPERTIES_OUTPUT_BUFFER)Params.ConnectionParameters;
  OsLog ("exynosi2c: connection parameters at %p", Conn);
  if (Conn == NULL) {
    return STATUS_INVALID_PARAMETER;
  }
  OsLog ("exynosi2c: properties %lu bytes", Conn->PropertiesLength);
  if (Conn->PropertiesLength < sizeof (PNP_I2C_SERIAL_BUS_DESCRIPTOR)) {
    return STATUS_INVALID_PARAMETER;
  }

  Bus = (PPNP_SERIAL_BUS_DESCRIPTOR)Conn->ConnectionProperties;
  if (Bus->SerialBusType != I2C_SERIAL_BUS_TYPE) {
    return STATUS_INVALID_PARAMETER;
  }

  I2c = (PPNP_I2C_SERIAL_BUS_DESCRIPTOR)Bus;
  if ((I2c->SerialBusDescriptor.TypeSpecificFlags & I2C_SERIAL_BUS_SPECIFIC_FLAG_10BIT_ADDRESS) != 0) {
    return STATUS_NOT_SUPPORTED;
  }

  Tgt          = GetTargetContext (Target);
  Tgt->Address = I2c->SlaveAddress;
  Tgt->Speed   = (I2c->ConnectionSpeed != 0) ? I2c->ConnectionSpeed : I2C_DEFAULT_HZ;
  OsLog ("exynosi2c: target 0x%02x at %lu Hz", Tgt->Address, Tgt->Speed);
  return STATUS_SUCCESS;
}

NTSTATUS
EvtPrepareHardware (WDFDEVICE Device, WDFCMRESLIST ResourcesRaw, WDFCMRESLIST ResourcesTranslated)
{
  PDEVICE_CONTEXT                  Ctx;
  PCM_PARTIAL_RESOURCE_DESCRIPTOR  Desc;
  ULONG                            Index;
  ULONG                            Count;
  ULONG_PTR                        Gsi;

  Ctx   = GetDeviceContext (Device);
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
    } else if (Desc->Type == CmResourceTypeInterrupt) {
      Gsi = WdfCmResourceListGetDescriptor (ResourcesRaw, Index)->u.Interrupt.Vector;
    }
  }

  if (Ctx->Base == NULL) {
    OsLog ("exynosi2c: no memory resource");
    return STATUS_DEVICE_CONFIGURATION_ERROR;
  }
  OsLog ("exynosi2c: controller mapped, GSI %Iu, %lu resources", Gsi, Count);

#if LOAD_TEST
  KeBugCheckEx (LOAD_TEST_BUGCHECK,
                (ULONG_PTR)WdfCmResourceListGetDescriptor (ResourcesRaw, 0)->u.Memory.Start.QuadPart,
                Gsi, Count, 0);
#else
  UNREFERENCED_PARAMETER (Gsi);
#endif
  return STATUS_SUCCESS;
}

NTSTATUS
EvtReleaseHardware (WDFDEVICE Device, WDFCMRESLIST ResourcesTranslated)
{
  PDEVICE_CONTEXT  Ctx;

  UNREFERENCED_PARAMETER (ResourcesTranslated);

  Ctx = GetDeviceContext (Device);
  if (Ctx->Base != NULL) {
    MmUnmapIoSpace (Ctx->Base, Ctx->Size);
    Ctx->Base = NULL;
  }
  return STATUS_SUCCESS;
}

NTSTATUS
EvtDeviceAdd (WDFDRIVER Driver, PWDFDEVICE_INIT DeviceInit)
{
  WDF_PNPPOWER_EVENT_CALLBACKS  PnpPower;
  WDF_OBJECT_ATTRIBUTES         Attributes;
  SPB_CONTROLLER_CONFIG         SpbConfig;
  WDFDEVICE                     Device;
  NTSTATUS                      Status;

  UNREFERENCED_PARAMETER (Driver);

  Status = SpbDeviceInitConfig (DeviceInit);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }

  WDF_PNPPOWER_EVENT_CALLBACKS_INIT (&PnpPower);
  PnpPower.EvtDevicePrepareHardware = EvtPrepareHardware;
  PnpPower.EvtDeviceReleaseHardware = EvtReleaseHardware;
  WdfDeviceInitSetPnpPowerEventCallbacks (DeviceInit, &PnpPower);

  //
  // Passive level: transfers poll with KeStallExecutionProcessor.
  //
  WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE (&Attributes, DEVICE_CONTEXT);
  Attributes.ExecutionLevel = WdfExecutionLevelPassive;
  Status = WdfDeviceCreate (&DeviceInit, &Attributes, &Device);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }

  SPB_CONTROLLER_CONFIG_INIT (&SpbConfig);
  SpbConfig.ControllerDispatchType = WdfIoQueueDispatchSequential;
  SpbConfig.EvtSpbTargetConnect    = EvtTargetConnect;
  SpbConfig.EvtSpbIoRead           = EvtIoRead;
  SpbConfig.EvtSpbIoWrite          = EvtIoWrite;
  SpbConfig.EvtSpbIoSequence       = EvtIoSequence;
  Status = SpbDeviceInitialize (Device, &SpbConfig);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }

  WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE (&Attributes, TARGET_CONTEXT);
  SpbControllerSetTargetAttributes (Device, &Attributes);
  return STATUS_SUCCESS;
}

NTSTATUS
DriverEntry (PDRIVER_OBJECT DriverObject, PUNICODE_STRING RegistryPath)
{
  WDF_DRIVER_CONFIG  Config;

  OsLogInit ();
  OsLog ("exynosi2c: DriverEntry");
  WDF_DRIVER_CONFIG_INIT (&Config, EvtDeviceAdd);
  return WdfDriverCreate (DriverObject, RegistryPath, WDF_NO_OBJECT_ATTRIBUTES, &Config, WDF_NO_HANDLE);
}
