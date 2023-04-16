import json

with open("sampleData/bus2-2.json", encoding="utf-8") as f:
    li = json.load(f)

objs = {"lists" : []}
for key, val in li.items():
    objs["lists"].append(
        {"id": key, "pos": val}
    )

with open("sampleData/bus2-2_4u.json", encoding="utf-8", mode="w") as f:
    json.dump(objs, f)
