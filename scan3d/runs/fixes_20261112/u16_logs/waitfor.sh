# waitfor.sh LOG [max_s]: wait until LOG has a line starting with EXIT, then print its tail
F=$1; M=${2:-3600}; t=0
while ! grep -q '^EXIT' "$F" 2>/dev/null; do sleep 15; t=$((t+15)); [ $t -gt $M ] && { echo "still running after $M s"; tail -n 3 "$F"; exit 0; }; done
tail -n 40 "$F"
