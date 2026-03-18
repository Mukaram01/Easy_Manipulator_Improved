#!/usr/bin/env python3
import ast
import re
import sys
from pathlib import Path

START_RE = re.compile(r"(?:^|[=(\s])python(?:3)?\s+-\s+<<'?(?P<tag>[A-Za-z_][A-Za-z0-9_]*)'?")


def iter_python_heredocs(path: Path):
    lines = path.read_text(encoding='utf-8').splitlines()
    idx = 0
    while idx < len(lines):
        match = START_RE.search(lines[idx])
        if not match:
            idx += 1
            continue
        tag = match.group('tag')
        start_line = idx + 2
        idx += 1
        body = []
        while idx < len(lines) and lines[idx] != tag:
            body.append(lines[idx])
            idx += 1
        if idx == len(lines):
            raise SyntaxError(f"{path}: unterminated Python heredoc starting at line {start_line - 1}")
        yield start_line, "\n".join(body) + "\n"
        idx += 1


def main(argv: list[str]) -> int:
    failed = False
    for name in argv[1:]:
        path = Path(name)
        try:
            found = False
            for start_line, source in iter_python_heredocs(path):
                found = True
                try:
                    ast.parse(source, filename=f"{path}:{start_line}")
                except SyntaxError as exc:
                    failed = True
                    print(f"{path}:{start_line}: invalid embedded Python: {exc.msg}", file=sys.stderr)
            if not found:
                print(f"{path}: warning: no Python heredocs found", file=sys.stderr)
        except Exception as exc:
            failed = True
            print(f"{path}: {exc}", file=sys.stderr)
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
