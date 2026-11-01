/** @file
  Just enough of UEFI's base types and library calls for the shared SD code
  from Exynos441xPkg SdCardDxe (mmccore.c, sdhci.c) to build in the Windows
  kernel unchanged.  Pointers are 32-bit on ARM, so a host's UINT32 Base
  holds the mapped virtual address of its registers.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#ifndef EFI_SHIM_H_
#define EFI_SHIM_H_

#include <ntddk.h>
#ifndef OSLOG_GLOBAL
#define OSLOG_EXTERN        // the ring pointer lives in exynosstor.c
#endif
#include "oslog.h"

typedef unsigned char       UINT8;
typedef unsigned short      UINT16;
typedef unsigned int        UINT32;
typedef unsigned long long  UINT64;
typedef ULONG_PTR           UINTN;
typedef LONG_PTR            INTN;
typedef UINT64              EFI_LBA;
typedef UINTN               EFI_STATUS;

#define STATIC    static
#define CONST     const
#define IN
#define OUT
#define OPTIONAL
#define EFIAPI

#define EFI_ENCODE_ERROR(x)    ((EFI_STATUS)(0x80000000u | (x)))
#define EFI_SUCCESS            ((EFI_STATUS)0)
#define EFI_INVALID_PARAMETER  EFI_ENCODE_ERROR (2)
#define EFI_DEVICE_ERROR       EFI_ENCODE_ERROR (7)
#define EFI_NO_MEDIA           EFI_ENCODE_ERROR (12)
#define EFI_TIMEOUT            EFI_ENCODE_ERROR (18)
#define EFI_ERROR(s)           (((INTN)(s)) < 0)

#define MIN(a, b)       (((a) < (b)) ? (a) : (b))
#define ARRAY_SIZE(a)   (sizeof (a) / sizeof ((a)[0]))
#define ZeroMem(p, n)   RtlZeroMemory ((p), (n))

static __inline UINT8  MmioRead8 (UINTN A)            { return READ_REGISTER_UCHAR ((volatile UCHAR *)A); }
static __inline UINT16 MmioRead16 (UINTN A)           { return READ_REGISTER_USHORT ((volatile USHORT *)A); }
static __inline UINT32 MmioRead32 (UINTN A)           { return READ_REGISTER_ULONG ((volatile ULONG *)A); }
static __inline VOID   MmioWrite8 (UINTN A, UINT8 V)  { WRITE_REGISTER_UCHAR ((volatile UCHAR *)A, V); }
static __inline VOID   MmioWrite16 (UINTN A, UINT16 V){ WRITE_REGISTER_USHORT ((volatile USHORT *)A, V); }
static __inline VOID   MmioWrite32 (UINTN A, UINT32 V){ WRITE_REGISTER_ULONG ((volatile ULONG *)A, V); }

//
// gBS->Stall (Microseconds): busy wait (the SD code runs at PASSIVE level
// in the storport worker, or during HwFindAdapter).
//
typedef struct {
  VOID  (*Stall)(UINTN Microseconds);
} EFI_SHIM_BOOT_SERVICES;

static VOID EfiShimStall (UINTN Microseconds) { KeStallExecutionProcessor ((ULONG)Microseconds); }
static EFI_SHIM_BOOT_SERVICES  mEfiShimBs = { EfiShimStall };
#define gBS  (&mEfiShimBs)

//
// DEBUG ((Level, Format, ...)) goes to the OS log ring.
//
#define DEBUG_INFO   0x00000040
#define DEBUG_ERROR  0x80000000
#define DEBUG(Expression)  EfiShimDebug Expression

static __inline VOID
EfiShimDebug (ULONG Level, const char *Format, ...)
{
  char     Line[160];
  size_t   Length;
  va_list  Args;

  UNREFERENCED_PARAMETER (Level);

  Line[0] = '\0';
  OSLOG_VA_START (Args, Format);
  RtlStringCbVPrintfA (Line, sizeof (Line), Format, Args);
  OSLOG_VA_END (Args);
  RtlStringCbLengthA (Line, sizeof (Line), &Length);
  OsLogWrite (Line, Length);           // Format carries its own newline
}

#endif
