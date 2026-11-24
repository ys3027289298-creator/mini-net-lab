#!/usr/bin/env python3
"""Fill tools/report.template.html with the CSVs in out/ -> report.html."""
import csv
import json
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def rows(name):
    out = []
    with open(os.path.join(ROOT, "out", name), newline="") as f:
        for r in csv.DictReader(f):
            out.append({k: (float(v) if _num(v) else v) for k, v in r.items()})
    return out


def _num(v):
    try:
        float(v)
        return True
    except ValueError:
        return False


def main():
    tpl = open(os.path.join(ROOT, "tools", "report.template.html")).read()
    for token, name in (("__CURVES__", "curves.csv"),
                        ("__EXP__", "exp.csv"),
                        ("__EQUAL__", "equal.csv")):
        tpl = tpl.replace(token, json.dumps(rows(name)))
    with open(os.path.join(ROOT, "report.html"), "w") as f:
        f.write(tpl)
    print("wrote report.html")


if __name__ == "__main__":
    main()
