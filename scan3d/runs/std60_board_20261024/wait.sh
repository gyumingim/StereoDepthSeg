#!/usr/bin/env bash
# wait.sh <file> <pattern>: wait until pattern appears in file (max ~100 min)
for i in $(seq 1 600); do grep -q "$2" "$1" && break; sleep 10; done
cat "$1"
