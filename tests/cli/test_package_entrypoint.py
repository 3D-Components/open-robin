import runpy
import sys


def test_package_entrypoint_runs_help(monkeypatch):
    # Execute `python -m robin --help` via runpy to cover robin/__main__.py
    monkeypatch.setattr(sys, 'argv', ['robin', '--help'])
    try:
        runpy.run_module('robin', run_name='__main__')
    except SystemExit as exc:  # Typer/click may call sys.exit(0)
        assert exc.code == 0
