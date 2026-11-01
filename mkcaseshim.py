#!/usr/bin/env python3
"""The WDK/SDK headers #include each other with Windows' case-insensitivity
(e.g. "WdfQueryInterface.h" for wdfqueryinterface.h).  Make a directory of
symlinks with the spellings the headers actually use: shim DIR INCDIR..."""
import os, re, sys
shim, dirs = sys.argv[1], sys.argv[2:]
os.makedirs(shim, exist_ok=True)
files = {}                                   # lower-case relative path -> real path
for d in dirs:
    for root, _, names in os.walk(d):
        for n in names:
            rel = os.path.relpath(os.path.join(root, n), d)
            files.setdefault(rel.lower(), os.path.join(root, n))
wanted = set()
for d in dirs:
    for root, _, names in os.walk(d):
        for n in names:
            try:
                text = open(os.path.join(root, n), encoding='latin-1').read()
            except OSError:
                continue
            wanted.update(re.findall(r'#\s*include\s*[<"]([^>"]+)[>"]', text))
made = 0
for inc in wanted:
    inc = inc.replace('\\', '/')
    real = files.get(inc.lower())
    if real is None or os.path.basename(real) == os.path.basename(inc) and os.path.exists(os.path.join(os.path.dirname(real), os.path.basename(inc))):
        continue
    link = os.path.join(shim, inc)
    if not os.path.lexists(link):
        os.makedirs(os.path.dirname(link), exist_ok=True)
        os.symlink(os.path.abspath(real), link)
        made += 1
print(f"{made} links in {shim}")
