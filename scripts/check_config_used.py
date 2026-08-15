#!/usr/bin/env python3
"""Every EngineConfig field must be read by the engine.

docs/ARCHITECTURE.md says tuning constants live in `bs::EngineConfig` and
"nothing is hard-coded in the algorithms". A field nothing reads breaks that
promise twice over: the behaviour it names is either hard-coded elsewhere or
absent entirely, and someone setting it in the JSON passed to `bs_create`
gets silence instead of an error.

That is the same defect as `keyframe_ids` being empty in every session ever
captured, or `regions` claiming a room existed before anything was mapped —
a documented field that lies — and it is invisible to every other test,
because nothing that works can notice a knob that does nothing.

Found eight on the first run, several of them named behaviour the plan
describes as shipped: the live keyframe cap, the H/E bootstrap gate, BoW
retrieval, the pair cap, track completion, radius-outlier removal.

    scripts/check_config_used.py            # from the repo root
"""
import re
import subprocess
import sys
from pathlib import Path

CONFIG_H = Path("core/src/common/config.h")
# Where a field is DECLARED and PARSED. Reads anywhere else count as use.
DECL_ONLY = {"core/src/common/config.h", "core/src/common/config.cpp"}


def main() -> int:
    if not CONFIG_H.is_file():
        print(f"ERROR: run from the repo root ({CONFIG_H} not found)",
              file=sys.stderr)
        return 1

    fields = re.findall(
        r"^\s{2}(?:int|float|double|bool|uint32_t|uint64_t|size_t)\s+(\w+)\s*=",
        CONFIG_H.read_text(), re.M)
    if len(fields) < 20:
        print(f"ERROR: parsed only {len(fields)} config fields — the "
              f"declaration style changed and this check has gone blind",
              file=sys.stderr)
        return 1

    unused = []
    for field in fields:
        out = subprocess.run(
            ["grep", "-rlE", rf"\b(config|config_|c|cfg)\.{field}\b",
             "core/src", "core/tests", "tools", "--include=*.cpp",
             "--include=*.h"],
            capture_output=True, text=True)
        readers = {line for line in out.stdout.split() if line not in DECL_ONLY}
        if not readers:
            unused.append(field)

    print(f"config fields: {len(fields)}, read by the engine: "
          f"{len(fields) - len(unused)}")
    if unused:
        print("\nERROR: these EngineConfig fields are read by nothing. Either "
              "the behaviour they name is missing, or it is hard-coded "
              "somewhere and the field is decoration:", file=sys.stderr)
        for field in unused:
            print(f"  {field}", file=sys.stderr)
        print("\nImplement it, or delete the field and say so in "
              "docs/BACKLOG.md. Do not leave a knob that does nothing.",
              file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
