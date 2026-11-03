#!/usr/bin/env python3
"""Check Markdown math so it renders in both GitLab and the VSCode built-in preview.

Rules: see MARKDOWN_MATH_GUIDE.md / MARKDOWN_MATH_GUIDE_EN.md (R1-R7).
Usage:  python3 check_md_math.py FILE.md [FILE.md ...]
Exit code 0 = no ERROR found. WARN lines do not change the exit code.
"""
import re
import sys

FENCE = re.compile(r"^\s*(```+|~~~+)")
CODE_SPAN = re.compile(r"(`+)(.+?)(?<!`)\1(?!`)")
MARK = "\x01"


def strip_code_spans(line: str):
    """Replace `code` spans with a marker. Returns (line, has_gitlab_math)."""
    line = CODE_SPAN.sub(lambda m: MARK, line)
    has_gitlab = ("$" + MARK + "$") in line
    line = line.replace("$" + MARK + "$", " ")  # GitLab-only $`...`$
    return line.replace(MARK, " "), has_gitlab


def check(path: str) -> int:
    errors = 0

    def report(level, n, msg):
        nonlocal errors
        if level == "ERROR":
            errors += 1
        print(f"{path}:{n}: {level}: {msg}")

    with open(path, encoding="utf-8") as fh:
        lines = fh.read().split("\n")

    in_fence = None
    in_block = False
    block_start = 0
    for n, raw in enumerate(lines, 1):
        m = FENCE.match(raw)
        if in_fence:
            if m and m.group(1)[0] == in_fence[0] and len(m.group(1)) >= len(in_fence):
                in_fence = None
            continue
        if m and not in_block:
            in_fence = m.group(1)
            if raw.strip().startswith("```math"):
                report("ERROR", n, "```math fence: VSCode preview does not render it, use $$ (R7)")
            continue

        stripped = raw.strip()
        if stripped == "$$":
            if raw != raw.lstrip():
                report("ERROR", n, "$$ line is indented; put it at column 0, outside lists/quotes (R2)")
            if not in_block:
                in_block, block_start = True, n
            else:
                in_block = False
            continue
        if in_block:
            if stripped == "":
                report("ERROR", n, f"blank line inside $$ block opened at line {block_start} (R2)")
            if re.search(r"[^\x00-\x7f]", raw):
                report("WARN", n, "non-ASCII character inside math (R5)")
            for bad in (r"\label", r"\eqref", r"\ref{", r"\newcommand", r"\bm{", r"\require"):
                if bad in raw:
                    report("ERROR", n, f"{bad} is not supported/portable in KaTeX (R4)")
            continue

        line, has_gitlab = strip_code_spans(raw)
        if has_gitlab:
            report("WARN", n, "$`...`$ renders only in GitLab/GitHub, not VSCode; prefer $...$ (R6)")
        if re.search(r"\\\(|\\\)", line):
            report("ERROR", n, r"\( \) delimiter: use $...$ (R7)")
        if re.search(r"(^|[^\\])\\\[|^\s*\\\]", line):
            report("ERROR", n, r"\[ \] delimiter: use $$ on its own lines (R7)")
        if "$$" in line:
            report("ERROR", n, "$$ must be alone on its own line (R2)")
            continue

        line = line.replace(r"\$", "  ")
        dollars = [i for i, c in enumerate(line) if c == "$"]
        if len(dollars) % 2:
            report("ERROR", n, "odd number of $ on this line (unclosed inline math or unescaped currency \\$)")
            continue
        is_table = line.lstrip().startswith("|")
        for a, b in zip(dollars[0::2], dollars[1::2]):
            body = line[a + 1:b]
            if not body.strip():
                continue
            if body[0].isspace() or body[-1].isspace():
                report("ERROR", n, f"space just inside $...$: '${body}$' (R1)")
            if b + 1 < len(line) and line[b + 1].isdigit():
                report("ERROR", n, f"digit right after closing $: '${body}${line[b + 1]}' (R1)")
            if is_table and "|" in body:
                report("ERROR", n, "| inside math in a table row; use \\lvert/\\rvert, \\lVert/\\rVert, \\mid (R6)")
            if re.search(r"[^\x00-\x7f]", body):
                report("WARN", n, f"non-ASCII character inside math: '${body}$' (R5)")

    if in_block:
        report("ERROR", block_start, "$$ block never closed")
    return errors


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    total = sum(check(p) for p in sys.argv[1:])
    print(f"{total} error(s)")
    return 1 if total else 0


if __name__ == "__main__":
    sys.exit(main())
