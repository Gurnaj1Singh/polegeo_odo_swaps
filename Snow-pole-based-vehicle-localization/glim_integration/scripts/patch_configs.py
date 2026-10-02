#!/usr/bin/env python3
"""
Apply config/overrides.json on top of the GLIM default configs that were
extracted from the image. stdlib-only (runs in any python).

Strategy: for each target file, set every override key WHEREVER it appears in
the JSON tree (recursive). GLIM keys (points_topic, imu_topic, acc_scale,
T_lidar_imu, config_odometry, ...) are unique enough that this is unambiguous
and, crucially, does not depend on the exact top-level wrapper the image ships.
Missing keys are reported (so we notice if the image's schema ever drifts).

GLIM config files may contain // and /* */ comments (JSON5-ish); we strip them
before parsing. Comments are lost in the patched copies (overrides.json is the
authoritative record of what we changed).

Usage:
  patch_configs.py --config-dir <dir> [--overrides <overrides.json>]
"""
import argparse
import json
import sys
from pathlib import Path


def strip_json_comments(text: str) -> str:
    """Remove // line comments and /* */ block comments outside of strings."""
    out = []
    i, n = 0, len(text)
    in_str = False
    esc = False
    while i < n:
        c = text[i]
        if in_str:
            out.append(c)
            if esc:
                esc = False
            elif c == "\\":
                esc = True
            elif c == '"':
                in_str = False
            i += 1
            continue
        if c == '"':
            in_str = True
            out.append(c)
            i += 1
        elif c == "/" and i + 1 < n and text[i + 1] == "/":
            while i < n and text[i] != "\n":
                i += 1
        elif c == "/" and i + 1 < n and text[i + 1] == "*":
            i += 2
            while i + 1 < n and not (text[i] == "*" and text[i + 1] == "/"):
                i += 1
            i += 2
        else:
            out.append(c)
            i += 1
    return "".join(out)


def load_jsonish(path: Path):
    raw = path.read_text()
    try:
        return json.loads(raw)
    except json.JSONDecodeError:
        return json.loads(strip_json_comments(raw))


def set_key_recursive(obj, key, value):
    """Set `key`=`value` everywhere `key` occurs in a nested dict/list. Return count."""
    hits = 0
    if isinstance(obj, dict):
        for k in list(obj.keys()):
            if k == key:
                obj[k] = value
                hits += 1
            else:
                hits += set_key_recursive(obj[k], key, value)
    elif isinstance(obj, list):
        for item in obj:
            hits += set_key_recursive(item, key, value)
    return hits


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config-dir", required=True)
    ap.add_argument("--overrides", default=str(Path(__file__).resolve().parent.parent / "config" / "overrides.json"))
    args = ap.parse_args()

    cfg_dir = Path(args.config_dir)
    overrides = json.loads(strip_json_comments(Path(args.overrides).read_text()))

    rc = 0
    for fname, kv in overrides.items():
        if fname.startswith("__"):
            continue
        target = cfg_dir / fname
        if not target.exists():
            print(f"[patch] MISSING target {target} — skipping (schema drift?)", file=sys.stderr)
            rc = 1
            continue
        data = load_jsonish(target)
        for key, val in kv.items():
            hits = set_key_recursive(data, key, val)
            if hits == 0:
                # Do NOT append: putting a key in the wrong scope (e.g. outside the
                # glim_ros wrapper) is silently ineffective. Surface it for a human.
                print(f"[patch] WARN {fname}: key '{key}' not found — left unset "
                      f"(check the image's schema / key name)")
            else:
                print(f"[patch] {fname}: {key} = {json.dumps(val)}  ({hits} site(s))")
        target.write_text(json.dumps(data, indent=2) + "\n")
    print("[patch] done.")
    return rc


if __name__ == "__main__":
    sys.exit(main())
