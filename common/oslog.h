/** @file
  RAM log for the Windows drivers: appends lines to the firmware's OS log
  ring (Exynos441xPkg Include/ExynosOsLog.h), which UEFI keeps out of the
  memory map and never clears, so it survives a warm reset and can be read
  with `usbdbg.py oslog`.

  Mapped uncached, so every line is in DRAM before the next instruction
  (the next instruction may be a bugcheck).  Only CPU0 runs, so raising to
  HIGH_LEVEL keeps writers from interleaving; exclusives (Interlocked*)
  are not usable on uncached memory.

  Call OsLogInit() once (DriverEntry), then OsLog("fmt", ...).

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#ifndef EXYNOS_OSLOG_H_
#define EXYNOS_OSLOG_H_

#include <ntstrsafe.h>

//
// The MSVC CRT's ARM va_start takes the address of the last named argument
// plus 4, which only works with MSVC's register spill layout; with clang it
// points at unrelated stack.  clang's builtin does it right.
//
#define OSLOG_VA_START(Args, Last)  __builtin_va_start (Args, Last)
#define OSLOG_VA_END(Args)          __builtin_va_end (Args)

#define OS_LOG_BASE   0x5E900000
#define OS_LOG_SIZE   0x00080000
#define OS_LOG_MAGIC  0x474C534F           // 'OSLG'

typedef struct {
  ULONG  Magic;
  ULONG  Head;
  ULONG  Reserved[2];
  UCHAR  Data[1];
} OS_LOG_RING;

#define OS_LOG_DATA_SIZE  (OS_LOG_SIZE - FIELD_OFFSET (OS_LOG_RING, Data))

//
// One ring pointer per driver: a driver built from several files defines
// OSLOG_GLOBAL in the file that calls OsLogInit() and OSLOG_EXTERN in the
// others.
//
#if defined (OSLOG_EXTERN)
extern volatile OS_LOG_RING *mOsLog;
#elif defined (OSLOG_GLOBAL)
volatile OS_LOG_RING *mOsLog;
#else
static volatile OS_LOG_RING *mOsLog;
#endif

static __inline VOID
OsLogInit (VOID)
{
  PHYSICAL_ADDRESS  Base;

  Base.QuadPart = OS_LOG_BASE;
  mOsLog = MmMapIoSpace (Base, OS_LOG_SIZE, MmNonCached);
}

static __inline VOID
OsLogWrite (const char *Text, size_t Length)
{
  volatile OS_LOG_RING  *Ring;
  ULONG                 Head;
  KIRQL                 OldIrql;

  Ring = mOsLog;
  if (Ring == NULL) {
    return;
  }

  KeRaiseIrql (HIGH_LEVEL, &OldIrql);
  if (Ring->Magic != OS_LOG_MAGIC) {
    Ring->Head  = 0;
    Ring->Magic = OS_LOG_MAGIC;
  }
  Head = Ring->Head;
  while (Length-- > 0) {
    Ring->Data[Head % OS_LOG_DATA_SIZE] = (UCHAR)*Text++;
    Head++;
  }
  Ring->Head = Head;
  KeLowerIrql (OldIrql);
}

/**
  printf-style; a newline is appended.  Prefix lines with the driver name.
**/
static __inline VOID
OsLog (const char *Format, ...)
{
  char     Line[160];
  size_t   Length;
  va_list  Args;

  Line[0] = '\0';
  OSLOG_VA_START (Args, Format);
  if (NT_SUCCESS (RtlStringCbVPrintfA (Line, sizeof (Line) - 1, Format, Args)) ||
      (Line[0] != '\0')) {
    RtlStringCbLengthA (Line, sizeof (Line) - 1, &Length);
    Line[Length++] = '\n';
    OsLogWrite (Line, Length);
  }
  OSLOG_VA_END (Args);
}

#endif
