#!/bin/sh
# Build and run the push_assist host test with the host's C++ compiler.
set -e
here=$(cd "$(dirname "$0")" && pwd)
out=$(mktemp -d)
${CXX:-c++} -std=c++20 -Wall -Wextra -Werror -I"$here/../include" \
  "$here/push_assist_host_test.cpp" -o "$out/push_assist_host_test"
"$out/push_assist_host_test"
