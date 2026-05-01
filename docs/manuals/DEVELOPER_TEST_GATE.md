# Developer test gate notes

## Local `ament_flake8` environment mismatch

On some developer machines, `ament_flake8` can fail before repository code is checked if a user-local pip installation shadows the ROS-compatible flake8 version.

Typical symptom:

- `ValueError: 'string' is not callable`
- Local path resembles `~/.local/lib/python3.x/site-packages/flake8`

## Diagnostics

Run:

```bash
python3 -m pip show flake8
which flake8
flake8 --version
```

## Local remediation

If `flake8` resolves to a user-local install instead of the ROS/apt toolchain version:

```bash
python3 -m pip uninstall -y flake8
hash -r
```

Then re-run package tests.
