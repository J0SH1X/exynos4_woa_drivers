#!/bin/sh
# Build all drivers into Artifacts/<name>/ (<name>.sys + <name>.inf), ready for
#   dism /Image:<mount> /Add-Driver /Driver:Artifacts/<name>/<name>.inf /ForceUnsigned
#
# Drivers log to the OS log ring, streamed over USB by exynosusb; failures
# are logged and fail the device, they do not bugcheck.  LOAD_TEST=1
# builds the old load tests, which bugcheck as soon as Windows starts them:
#   exynosi2c   0x49324321 "I2C!"     exynosusb   0x55534221 "USB!"
#   exynosgpio  0x4750494F "GPIO"     exynosclk   0x434C4B21 "CLK!"
#   mxt540e     0x4D585421 "MXT!"     exynosstor  0x53544F52 "STOR" (8.1)
#                                     exynossd    0x53444321 "SDC!" (Win 10)
set -e
cd "$(dirname "$0")"

LIB=${WDK:-$HOME/s2/winsdk/wdk10}/Lib/10.0.19041.0/km/arm
if [ -n "$LOAD_TEST" ]; then
  export EXTRA_CFLAGS="$EXTRA_CFLAGS -DLOAD_TEST=$LOAD_TEST"
fi

./build-driver.sh exynosi2c exynosi2c/exynosi2c.c -- "$LIB/spb/1.0/spbcxstubs.lib" "$LIB/ntstrsafe.lib"
./build-driver.sh exynosgpio exynosgpio/exynosgpio.c -- "$LIB/msgpioclxstub.lib" "$LIB/ntstrsafe.lib"
./build-driver.sh mxt540e mxt540e/mxt540e.c -- "$LIB/ntstrsafe.lib"
./build-driver.sh exynosusb exynosusb/exynosusb.c -- "$LIB/ntstrsafe.lib"
./build-driver.sh exynosclk exynosclk/exynosclk.c -- "$LIB/ntstrsafe.lib"

# Storport miniport (not KMDF): microSD on Windows 8.1, SD code shared with UEFI.
WDM=1 ./build-driver.sh exynosstor exynosstor/exynosstor.c exynosstor/mmccore.c exynosstor/sdhci.c -- \
  "$LIB/storport.lib" "$LIB/ntstrsafe.lib"

# sdport miniport (not KMDF), Windows 10 only: sdport does not exist in 8.1.
WDM=1 EXTRA_CFLAGS="$EXTRA_CFLAGS -UNTDDI_VERSION -U_WIN32_WINNT -UWINVER -DNTDDI_VERSION=0x0A000000 -D_WIN32_WINNT=0x0A00 -DWINVER=0x0A00" \
  ./build-driver.sh exynossd exynossd/exynossd.c -- "$LIB/sdport.lib" "$LIB/ntstrsafe.lib"

# Artifacts/<name>/: the .sys and its .inf, one folder per driver so DISM
# can add them one at a time (exynosstor and exynossd claim the same device).
for d in exynosi2c exynosgpio mxt540e exynosusb exynosclk exynosstor exynossd; do
  cp "$d/$d.inf" "$d/build/"
  mkdir -p "Artifacts/$d"
  cp "$d/build/$d.sys" "$d/$d.inf" "Artifacts/$d/"
done
echo "Artifacts/: $(ls Artifacts | tr '\n' ' ')"
