"""Guard against silent drift of the FAAL filter core.

Mainline (2026-11-15): faal_B_lastsign.rhai. The only other host script is
faal_ps.rhai (player-space, raw stage). Both must share one `axis()` body; a
core change is written once in faal_B_lastsign.rhai and copied over with
--sync, never edited twice.

Usage:
    python check_core.py            # verify the guarded scripts
    python check_core.py --hash     # print the canonical hash only
    python check_core.py --sync     # copy axis() from the reference into the rest

The structural check is the one from the 2026-11-14 incident: the per-axis
state width is duplicated in six places (init, two axis calls, two store-backs,
the returns), and getting it wrong fails silently. The check derives the width
from axis()'s return arity and verifies every site agrees.
"""
import hashlib
import pathlib
import re
import sys

BASE = pathlib.Path(__file__).resolve().parent
PREFIXES = ["faal_B_lastsign", "faal_B_rev_mag", "faal_ps"]


def newest(prefix: str) -> str | None:
    """Highest _vN file for a prefix, so bumping the version never breaks this."""
    found = []
    for path in BASE.glob(prefix + "_v*.rhai"):
        m = re.search(r"_v(\d+)\.rhai$", path.name)
        if m:
            found.append((int(m.group(1)), path.name))
    return max(found)[1] if found else None


REFERENCE = newest("faal_B_lastsign")
GUARDED = [n for n in (newest(p) for p in PREFIXES) if n]
EXPECTED = "62cbf6919484"


def grab(text: str, name: str) -> str | None:
    """Extract a top-level fn body by brace matching."""
    i = text.find("fn " + name + "(")
    if i < 0:
        return None
    depth = 0
    started = False
    for j in range(i, len(text)):
        if text[j] == "{":
            depth += 1
            started = True
        elif text[j] == "}":
            depth -= 1
            if started and depth == 0:
                return text[i : j + 1]
    return None


def core_hash(text: str) -> str:
    body = grab(text, "axis")
    if body is None:
        return "missing"
    clean = "\n".join(
        re.sub(r"//.*$", "", line).strip()
        for line in body.split("\n")
        if re.sub(r"//.*$", "", line).strip()
    )
    return hashlib.sha1(clean.encode()).hexdigest()[:12]


def split_top(text: str) -> list[str]:
    """Split on commas that are not inside (), [] or {}."""
    parts, depth, cur = [], 0, []
    for ch in text:
        if ch in "([{":
            depth += 1
        elif ch in ")]}":
            depth -= 1
        if ch == "," and depth == 0:
            parts.append("".join(cur))
            cur = []
        else:
            cur.append(ch)
    parts.append("".join(cur))
    return [p for p in parts if p.strip()]


def count_items(between: str) -> int:
    return len(split_top(between))


def axis_calls(text: str) -> list[str]:
    """Return the state-array argument of every axis(...) call."""
    out = []
    for m in re.finditer(r"\baxis\(", text):
        i = m.end()
        depth = 1
        while i < len(text) and depth:
            if text[i] == "(":
                depth += 1
            elif text[i] == ")":
                depth -= 1
            i += 1
        args = text[m.end() : i - 1]
        # the state array is the only bracket group right after the second comma;
        # subscripts like S[0] inside it must not be mistaken for the outer one
        start = args.find("[S[")
        if start < 0:
            continue
        end = args.rfind("]")
        if end > start:
            out.append(args[start + 1 : end])
    return out


def structure(text: str) -> list[str]:
    """Return a list of problems with the per-axis state width plumbing."""
    problems = []
    body = grab(text, "axis")
    if body is None:
        return ["axis() missing"]
    rets = re.findall(r"return \[([^\]]*)\];", body)
    if not rets:
        return ["axis() has no return"]
    slots = count_items(rets[-1]) - 1  # out + N states
    for r in rets:
        if count_items(r) != slots + 1:
            problems.append(f"axis return has {count_items(r)} values, expected {slots + 1}")
    states = re.findall(r"this\.S = \[(.*?)\];", text, re.S)
    if not states:
        return problems + ["this.S not initialised"]
    total = count_items(states[0])
    if slots and total % slots:
        problems.append(f"this.S has {total} slots, not a multiple of {slots}")
    axes = total // slots if slots else 0
    calls = axis_calls(text)
    for i, c in enumerate(calls):
        if count_items(c) != slots:
            problems.append(f"axis call #{i} passes {count_items(c)} slots, expected {slots}")
    if len(calls) != axes:
        problems.append(f"{len(calls)} axis calls for {axes} axes")
    for i, s in enumerate(states[1:], 1):
        if count_items(s) != total:
            problems.append(f"this.S store #{i} has {count_items(s)} values, expected {total}")
    # indices, not just counts: a stripped slot leaves reads past the end, which
    # throws every frame and silently bypasses the whole filter (2026-11-15)
    st_idx = [int(x) for x in re.findall(r"st\[(\d+)\]", body)]
    if st_idx and max(st_idx) >= slots:
        problems.append(f"axis reads st[{max(st_idx)}] but a filter only carries {slots} slots")
    s_idx = [int(x) for x in re.findall(r"\bS\[(\d+)\]", text)]
    if s_idx and max(s_idx) >= total:
        problems.append(f"process reads S[{max(s_idx)}] but this.S has {total} slots")
    k_arr = re.search(r"this\.K = \[(.*?)\];", text, re.S)
    if k_arr:
        kn = count_items(k_arr.group(1))
        k_idx = [int(x) for x in re.findall(r"k\[(\d+)\]", body)]
        if k_idx and max(k_idx) >= kn:
            problems.append(f"axis reads k[{max(k_idx)}] but this.K has {kn} entries")
    return problems


def main() -> int:
    if "--hash" in sys.argv:
        print(core_hash((BASE / REFERENCE).read_text(encoding="utf-8")))
        return 0

    ref_path = BASE / REFERENCE
    if not ref_path.exists():
        print(f"missing reference script: {REFERENCE}")
        return 2
    ref_text = ref_path.read_text(encoding="utf-8")
    ref_hash = core_hash(ref_text)
    ref_body = grab(ref_text, "axis")

    if "--sync" in sys.argv:
        changed = 0
        for name in GUARDED:
            if name == REFERENCE:
                continue
            path = BASE / name
            text = path.read_text(encoding="utf-8")
            old = grab(text, "axis")
            if old is None:
                print(f"{name}: no axis()")
                return 2
            if old == ref_body:
                print(f"{name}: already in sync")
                continue
            path.write_text(text.replace(old, ref_body, 1), encoding="utf-8")
            print(f"{name}: axis() replaced ({len(old.splitlines())} -> {len(ref_body.splitlines())} lines)")
            changed += 1
        if not changed:
            return 0

    ok = True
    print(f"reference {REFERENCE}: {ref_hash}")
    if EXPECTED != "PENDING" and ref_hash != EXPECTED:
        print(f"  WARNING: expected {EXPECTED}; update EXPECTED after an intentional core change")
        ok = False

    for name in GUARDED:
        path = BASE / name
        if not path.exists():
            print(f"{name:22} MISSING")
            ok = False
            continue
        text = path.read_text(encoding="utf-8")
        h = core_hash(text)
        state = "same" if h == ref_hash else "DIFFERS"
        print(f"{name:22} {h:14} {state}  ({len(structure(text))} structural problems)")
        if state != "same":
            ok = False
            print("  run: python check_core.py --sync")
        for p in structure(text):
            print("    - " + p)
            ok = False

    print("\nOK" if ok else "\nMISMATCH")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
