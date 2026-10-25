import json, sys
d = json.load(open(sys.argv[1]))
for c, s in d["summary"].items():
    print(c, "div", s["div"], "big", s.get("big_changes"), "ident", s.get("identical_cells"))
