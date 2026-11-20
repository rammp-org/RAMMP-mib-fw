#!/bin/zsh
# domain_size.sh <build_dir>: sizeof(rtps::Domain) and a few pools under that build's sdkconfig.
B=$1
CMD=$(python3 - "$B" <<'PY'
import json,sys
cc=json.load(open(sys.argv[1]+"/compile_commands.json"))
e=next(e for e in cc if e["file"].endswith("mib_system.cpp"))
print(e["command"])
PY
)
cat > /tmp/domain_probe.cpp <<'CPP'
#include "rtps/entities/Domain.hpp"
#include "rtps/discovery/ParticipantProxyData.hpp"
template <unsigned long N> struct Size;
Size<sizeof(rtps::Domain)> domain;
Size<sizeof(rtps::ParticipantProxyData)> ppd;
CPP
DIR=$(echo "$CMD" | sed -E 's/.* -o ([^ ]+) -c .*/\1/' | xargs dirname)
C2=$(echo "$CMD" | sed -E 's# -o [^ ]+ -c [^ ]+# -fsyntax-only -c /tmp/domain_probe.cpp#')
(cd $B && eval "$C2") 2>&1 | grep -oE "Size<[0-9]+>" | sort -u
