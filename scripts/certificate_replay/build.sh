#!/usr/bin/env bash
# Build the offline certificate replay against the configured workcell_builder
# build directory (same compiler flags and MoveIt overlay as the support-contact
# adapter). Diagnostic tool; not part of the commissioning capability.
# usage: build.sh [output binary]   (source the workspace overlays first)
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
workspace=$(cd "$here/../../../.." && pwd)
out=${1:-$workspace/build/workcell_builder/certificate_replay}
python3 - "$workspace/build/workcell_builder/CMakeFiles/workcell_support_contact.dir" "$here/certificate_replay.cpp" "$out" <<'PY'
import re,shlex,subprocess,sys
from pathlib import Path
build,src,out=Path(sys.argv[1]),sys.argv[2],sys.argv[3]
flags={k:v for k,v in re.findall(r'^(CXX_\w+) = (.*)$',(build/'flags.make').read_text(),re.M)}
link=(build/'link.txt').read_text().split()
libs=[x for x in link if (x.endswith('.so') or re.search(r'\.so\.\d',x) or x.startswith(('-l','-Wl,-rpath'))) and 'libworkcell_support_contact.so' not in x]
cmd=['c++',*shlex.split(flags['CXX_DEFINES']),*shlex.split(flags['CXX_INCLUDES']),*shlex.split(flags['CXX_FLAGS']),
     '-Wno-unused-parameter','-o',out,src,*libs,'/opt/ros/humble/lib/libsrdfdom.so','-lurdfdom_model','-lurdfdom_world']
subprocess.run(cmd,check=True);print(out)
PY
