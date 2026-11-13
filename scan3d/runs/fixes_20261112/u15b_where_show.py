#!/usr/bin/env python3
"""fixes 11-13 unit 15b: print the 'removed by guard' rows of u15_where_*.json files (read only). Usage: u15b_where_show.py JSON..."""
import json
import sys

for p in sys.argv[1:]:
    r = json.load(open(p))
    print(p.rsplit("/", 1)[-1], json.dumps(r["A_removed_by_guard"]), "kept gain", json.dumps(r["K_gain_with_guard"]))
