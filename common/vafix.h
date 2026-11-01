/** @file
  Forced into every compile (build-driver.sh -include).

  The MSVC CRT's ARM va_start (vadefs.h) is "address of the last named
  argument + 4", which relies on MSVC spilling r0-r3 next to the stacked
  arguments.  clang does not, so every varargs function compiled from the
  WDK headers (ntstrsafe's RtlUnicodeStringPrintf behind
  RESOURCE_HUB_CREATE_PATH_FROM_ID, ...) would read its arguments from
  unrelated stack.  clang's builtins get it right; va_list is char * either
  way, so the result still goes to the kernel's _vsnprintf unchanged.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#ifndef EXYNOS_VAFIX_H_
#define EXYNOS_VAFIX_H_

#include <stdarg.h>

#undef va_start
#undef va_arg
#undef va_end
#define va_start(ap, v)  __builtin_va_start (ap, v)
#define va_arg(ap, t)    __builtin_va_arg (ap, t)
#define va_end(ap)       __builtin_va_end (ap)

#endif
