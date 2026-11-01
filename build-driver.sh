#!/bin/sh
# Build an ARM32 KMDF 1.13 driver (Windows RT 8.1) on Linux with clang +
# lld-link, against the WDK 10.0.19041 ARM kit (the last WDK with ARM32
# kernel libraries; it still ships KMDF 1.11/1.13 stubs for 8.1).
#
#   build-driver.sh NAME SOURCE.c... [-- EXTRA_LIBS...]
#
# Output: NAME/build/NAME.sys, test-signed when the test certificate exists.
set -e
cd "$(dirname "$0")"

WDK=${WDK:-$HOME/s2/winsdk/wdk10}
CRT=${CRT:-$HOME/s2/winsdk/splat/crt/include}
SDKSHARED=${SDKSHARED:-$HOME/s2/winsdk/splat/sdk/include/shared}
# Case-correct symlinks for the headers' #includes (mkcaseshim.py).
SHIM=${SHIM:-$HOME/s2/winsdk/caseshim}
SIGN=${SIGN:-$HOME/s2/winsdk/testsign/testsign}
KMDF=1.13
INC="$WDK/Include/10.0.19041.0"
LIB="$WDK/Lib/10.0.19041.0/km/arm"

NAME=$1; shift
SRCS=""; LIBS=""
while [ $# -gt 0 ]; do
  [ "$1" = "--" ] && { shift; LIBS="$*"; break; }
  SRCS="$SRCS $1"; shift
done

OUT=$NAME/build
mkdir -p "$OUT"

# -mno-unaligned-access: packed structures (resource hub descriptors, ...)
# are read bytewise instead of with word loads that can fault on ARM.
CFLAGS="--target=thumbv7-pc-windows-msvc -fms-extensions -fms-compatibility
  -mno-unaligned-access -O2 -fno-stack-protector -ffreestanding -Wall -Wno-microsoft -Wno-unknown-pragmas
  -Wno-ignored-attributes -Wno-ignored-pragma-intrinsic -Wno-pragma-pack
  -include vafix.h -I$NAME -Icommon -isystem $SHIM -isystem $INC/km -isystem $INC/km/crt -isystem $INC/shared
  -isystem $INC/km/spb/1.0 -isystem $WDK/Include/wdf/kmdf/$KMDF -isystem $CRT -isystem $SDKSHARED
  -D_ARM_=1 -D_KERNEL_MODE -D_ARM_WINAPI_PARTITION_DESKTOP_SDK_AVAILABLE=1
  -DNTDDI_VERSION=0x06030000 -D_WIN32_WINNT=0x0603 -DWINVER=0x0603
  -DKMDF_VERSION_MAJOR=1 -DKMDF_VERSION_MINOR=13 $EXTRA_CFLAGS"

OBJS=""
for s in $SRCS; do
  o="$OUT/$(basename "${s%.c}").obj"
  clang $CFLAGS -c "$s" -o "$o"
  OBJS="$OBJS $o"
done

# KMDF drivers start in the framework stub; WDM=1 (e.g. sdport miniports)
# in GsDriverEntry, which sets up the stack cookie and calls DriverEntry.
if [ -n "$WDM" ]; then
  ENTRY=GsDriverEntry
  FRAMEWORK=""
else
  ENTRY=FxDriverEntry
  FRAMEWORK="$WDK/Lib/wdf/kmdf/arm/$KMDF/wdfdriverentry.lib $WDK/Lib/wdf/kmdf/arm/$KMDF/wdfldr.lib"
fi

lld-link /DRIVER /RELEASE /SUBSYSTEM:NATIVE /MACHINE:ARM /ENTRY:$ENTRY \
  /NODEFAULTLIB /OUT:"$OUT/$NAME.sys" $OBJS $FRAMEWORK \
  "$LIB/ntoskrnl.lib" "$LIB/hal.lib" "$LIB/bufferoverflowfastfailk.lib" $LIBS "$LIB/libcntpr.lib"

if [ -f "$SIGN.key" ]; then
  osslsigncode sign -certs "$SIGN.crt" -key "$SIGN.key" -h sha256 \
    -in "$OUT/$NAME.sys" -out "$OUT/$NAME.signed.sys" > /dev/null
  mv "$OUT/$NAME.signed.sys" "$OUT/$NAME.sys"
fi
echo "$OUT/$NAME.sys"
