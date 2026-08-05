#!/usr/bin/env python3
"""Repair quoting in the temporary PR7 patch applicator, then remove itself."""

import re
from pathlib import Path


path = Path("scripts/pr7_apply_patch.py")
text = path.read_text(encoding="utf-8")
fixed, count = re.subn(r'(?<!\\)\\"', '"', text)
if count < 10:
    raise SystemExit(f"expected to normalize ordinary quoted C++ tokens, got {count}")
if fixed == text:
    raise SystemExit("temporary PR7 patch applicator did not change")
path.write_text(fixed, encoding="utf-8")
Path(__file__).unlink()
