#!/bin/bash
# prints SKIN AP battery charging; exit 3 if over limits (SKIN>=45 AP>=50 bat<20), exit 2 if not under start gate
A="adb -s 100.79.156.20:46821"
t=$($A shell "dumpsys thermalservice | grep -A8 'Current temperatures from HAL' | grep -E 'mName=(AP|SKIN)'" | sed -E 's/.*mValue=([0-9.]+).*mName=([A-Z]+).*/\2=\1/' | tr -d '\r' | tr '\n' ' ')
b=$($A shell "dumpsys battery | grep -E '^  (AC powered|USB powered|level):'" | tr -d '\r ' | tr '\n' ',')
w=$($A shell "dumpsys power | grep mWakefulness=" | tr -d '\r ')
skin=$(echo "$t" | grep -o 'SKIN=[0-9.]*' | cut -d= -f2); ap=$(echo "$t" | grep -o 'AP=[0-9.]*' | cut -d= -f2); lvl=$(echo "$b" | grep -o 'level:[0-9]*' | cut -d: -f2)
echo "$(date +%T) $t bat=$b $w"
{ [ -z "$skin" ] || [ -z "$ap" ] || [ -z "$lvl" ]; } && { echo "phone unreachable / no readings"; exit 4; }
awk "BEGIN{exit !($skin >= 45 || $ap >= 50 || $lvl < 20)}" && exit 3
awk "BEGIN{exit !($skin >= 40 || $ap >= 46)}" && exit 2
exit 0
