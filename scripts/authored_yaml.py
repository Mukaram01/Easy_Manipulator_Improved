"""Atomic semantic YAML updates, preserving unchanged top-level source text."""
import os
from pathlib import Path
import tempfile
import yaml


def write_preserving(path: Path, data):
    original = path.read_text(encoding='utf-8') if path.exists() else ''
    old = yaml.safe_load(original)
    if old == data:
        return
    rendered = yaml.safe_dump(data, sort_keys=False, allow_unicode=True)
    # Preserve unrelated comments, numeric spellings, and anchor definitions.
    # Verify the assembled document: cross-section aliases may require the
    # deterministic whole-document representation instead.
    if isinstance(old, dict) and isinstance(data, dict) and old.keys() == data.keys():
        node = yaml.compose(original)
        replacements = []
        for key, value in node.value:
            name = key.value
            if old[name] == data[name]:
                continue
            start = key.start_mark.index
            end = value.end_mark.index
            while end < len(original) and original[end] != '\n':
                end += 1
            if end < len(original):
                end += 1
            replacements.append((start, end, yaml.safe_dump({name: data[name]}, sort_keys=False, allow_unicode=True)))
        candidate = original
        for start, end, replacement in reversed(replacements):
            candidate = candidate[:start] + replacement + candidate[end:]
        try:
            if yaml.safe_load(candidate) == data:
                rendered = candidate
        except yaml.YAMLError:
            pass
    if yaml.safe_load(rendered) != data:
        raise ValueError(f'YAML semantic verification failed: {path}')
    fd, temporary = tempfile.mkstemp(dir=path.parent, prefix=f'.{path.name}.')
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            stream.write(rendered)
            stream.flush()
            os.fsync(stream.fileno())
        if path.exists():
            os.chmod(temporary, path.stat().st_mode)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
