/** @file
  Exynos4210 clock controller (CMU) interface, served by exynosclk.sys.

  The same clocks as UEFI's ClockDxe (Exynos441xPkg Include/Protocol/
  ExynosClock.h): gates are single CLKGATE_IP_* bits, special clocks
  (sclk_mmcN) are mux -> ratio -> pre-ratio chains fed from MPLL.

  A consumer asks for the interface with ExynosClockOpen() (PASSIVE_LEVEL);
  it finds the clock device through its device interface and sends it
  IRP_MN_QUERY_INTERFACE, so it also works from drivers without a WDF
  device (sdport miniports).  No ACPI _DEP on \_SB.CLK0: Windows 8.1 holds a
  _DEP device until the dependency provides an operation region, which the
  clock driver never does; a consumer copes with STATUS_NOT_FOUND instead.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#ifndef EXYNOS_CLK_H_
#define EXYNOS_CLK_H_

// {97c8ab37-907d-4672-baea-1aa4f84d4613}: device interface of exynosclk.
DEFINE_GUID (GUID_DEVINTERFACE_EXYNOS_CLOCK,
             0x97c8ab37, 0x907d, 0x4672, 0xba, 0xea, 0x1a, 0xa4, 0xf8, 0x4d, 0x46, 0x13);

// {655fa550-0371-4e42-8fdc-9f1555ab30da}: IRP_MN_QUERY_INTERFACE type.
DEFINE_GUID (GUID_EXYNOS_CLOCK_INTERFACE,
             0x655fa550, 0x0371, 0x4e42, 0x8f, 0xdc, 0x9f, 0x15, 0x55, 0xab, 0x30, 0xda);

#define EXYNOS_CLOCK_INTERFACE_VERSION  1

typedef enum {
  ExynosClkSclkMmc2 = 0,      // sclk_mmc2: HSMMC2 (microSD)
  ExynosClkSclkDwmci,         // sclk_dwmci (dout_mmc4): MSHCI (eMMC)
  ExynosClkGateSdmmc2,
  ExynosClkGateDwmci,
  ExynosClkGateUsbOtg,
  ExynosClkGateI2c0,          // i2c0..i2c7: add the bus number
  ExynosClkGateI2c1,
  ExynosClkGateI2c2,
  ExynosClkGateI2c3,
  ExynosClkGateI2c4,
  ExynosClkGateI2c5,
  ExynosClkGateI2c6,
  ExynosClkGateI2c7,
  ExynosClkMax
} EXYNOS_CLOCK_ID;

typedef NTSTATUS (*PEXYNOS_CLOCK_ENABLE) (PVOID Context, EXYNOS_CLOCK_ID Id);
typedef NTSTATUS (*PEXYNOS_CLOCK_DISABLE) (PVOID Context, EXYNOS_CLOCK_ID Id);
typedef NTSTATUS (*PEXYNOS_CLOCK_GET_RATE) (PVOID Context, EXYNOS_CLOCK_ID Id, PULONG RateHz);
typedef NTSTATUS (*PEXYNOS_CLOCK_SET_RATE) (PVOID Context, EXYNOS_CLOCK_ID Id, ULONG RateHz, PULONG ActualHz);

typedef struct {
  INTERFACE               Header;       // Context = first argument of the calls
  PEXYNOS_CLOCK_ENABLE    Enable;       // gate on / special clock unmasked
  PEXYNOS_CLOCK_DISABLE   Disable;
  PEXYNOS_CLOCK_GET_RATE  GetRate;      // special clocks only
  PEXYNOS_CLOCK_SET_RATE  SetRate;      // special clocks only, from MPLL, <= RateHz
} EXYNOS_CLOCK_INTERFACE, *PEXYNOS_CLOCK_INTERFACE;

#ifndef EXYNOS_CLK_SERVER

/**
  Get the clock interface from exynosclk (PASSIVE_LEVEL).  Release with
  Iface->Header.InterfaceDereference (Iface->Header.Context).

  @retval STATUS_NOT_FOUND  The clock driver has not started (yet).
**/
static NTSTATUS
ExynosClockOpen (PEXYNOS_CLOCK_INTERFACE Iface)
{
  PWSTR               List;
  UNICODE_STRING      Name;
  PFILE_OBJECT        File;
  PDEVICE_OBJECT      Device;
  PIRP                Irp;
  PIO_STACK_LOCATION  Stack;
  KEVENT              Event;
  IO_STATUS_BLOCK     Iosb;
  NTSTATUS            Status;

  Status = IoGetDeviceInterfaces (&GUID_DEVINTERFACE_EXYNOS_CLOCK, NULL, 0, &List);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }
  if (List[0] == L'\0') {
    ExFreePool (List);
    return STATUS_NOT_FOUND;
  }

  RtlInitUnicodeString (&Name, List);
  Status = IoGetDeviceObjectPointer (&Name, FILE_READ_DATA, &File, &Device);
  ExFreePool (List);
  if (!NT_SUCCESS (Status)) {
    return Status;
  }

  Device = IoGetAttachedDeviceReference (Device);
  KeInitializeEvent (&Event, NotificationEvent, FALSE);
  Irp = IoBuildSynchronousFsdRequest (IRP_MJ_PNP, Device, NULL, 0, NULL, &Event, &Iosb);
  if (Irp == NULL) {
    Status = STATUS_INSUFFICIENT_RESOURCES;
  } else {
    RtlZeroMemory (Iface, sizeof (*Iface));
    Irp->IoStatus.Status = STATUS_NOT_SUPPORTED;
    Stack = IoGetNextIrpStackLocation (Irp);
    Stack->MinorFunction                           = IRP_MN_QUERY_INTERFACE;
    Stack->Parameters.QueryInterface.InterfaceType = &GUID_EXYNOS_CLOCK_INTERFACE;
    Stack->Parameters.QueryInterface.Size          = sizeof (*Iface);
    Stack->Parameters.QueryInterface.Version       = EXYNOS_CLOCK_INTERFACE_VERSION;
    Stack->Parameters.QueryInterface.Interface     = (PINTERFACE)Iface;
    Stack->Parameters.QueryInterface.InterfaceSpecificData = NULL;

    Status = IoCallDriver (Device, Irp);
    if (Status == STATUS_PENDING) {
      KeWaitForSingleObject (&Event, Executive, KernelMode, FALSE, NULL);
      Status = Iosb.Status;
    }
  }

  ObDereferenceObject (Device);
  ObDereferenceObject (File);
  return Status;
}

#endif

#endif
