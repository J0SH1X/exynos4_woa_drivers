# Exynos441x Windows drivers

Windows RT 8.1 (ARM32, KMDF 1.13) drivers (and a Windows 10 sdport miniport) for the Samsung Galaxy Note GT-N7000
(Exynos4210), for the ACPI devices Exynos441xPkg describes:

| Driver       | ACPI     | What                                             | Bugcheck code      |
|--------------|----------|--------------------------------------------------|--------------------|
| `exynosi2c`  | EXYN0002 | I2C3 as an SpbCx controller (polled transfers)   | `0x49324321` I2C!  |
| `exynosgpio` | EXYN0003 | GPX0 / EINT0-7 as a GpioClx controller           | `0x4750494F` GPIO  |
| `mxt540e`    | EXYN0004 | mXT540E touchscreen, HID (lower filter of mshidkmdf) | `0x4D585421` MXT!  |
| `exynosusb`  | EXYN0001 | DWC2 USB device: same 1209:0001 log + debug channel as UEFI (OS log ring, peek/poke/dump) | `0x55534221` USB!  |
| `exynosclk`  | EXYN0005 | CMU: clock gates/rates for other drivers (`common/exynosclk.h`) | `0x434C4B21` CLK!  |
| `exynosstor` | EXYN0006 | microSD (HSMMC2) as a Storport disk for **8.1**; SD code shared with UEFI (`mmccore.c`, `sdhci.c`) | `0x53544F52` STOR  |
| `exynossd`   | EXYN0006 | microSD (HSMMC2) sdport miniport, **Windows 10 only** | `0x53444321` SDC!  |

The touchscreen talks to the controller through SPB (I2cSerialBus) and gets
its interrupt as a GpioInt, so it needs both controller drivers.

Drivers log to the firmware's OS log ring (`common/oslog.h`, 0x40E00000),
which survives a warm reset: `usbdbg.py oslog`, or live over USB once
`exynosusb` runs.

## Building (Linux)

Needs clang, lld-link, osslsigncode and:

- the WDK 10.0.19041 ARM kit (the last WDK with ARM32 kernel libraries; it
  still has KMDF 1.13 for 8.1), unpacked from its MSIs with msitools, at
  `~/s2/winsdk/wdk10`;
- SDK shared + MSVC CRT headers from xwin at `~/s2/winsdk/splat`;
- `mkcaseshim.py ~/s2/winsdk/caseshim <include dirs>` for the headers'
  case-insensitive `#include`s;
- the test certificate at `~/s2/winsdk/testsign/`.

    ./build.sh               # drivers, into Artifacts/<name>/
    LOAD_TEST=1 ./build.sh   # load tests: bugcheck as soon as Windows starts them

Debugging is the OS log over USB (exynosusb + Tools/usblog.py, or
usbdbg.py oslog after a warm reset).  Failures are logged and fail the
device; the drivers do not bugcheck (except LOAD_TEST builds).

## Installing

    dism /Image:<mount> /Add-Driver /Driver:Artifacts/<name>/<name>.inf /ForceUnsigned

with `testsigning on` in the BCD.
