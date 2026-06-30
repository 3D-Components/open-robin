import pytest   # NoQA: ignore unused import

from typer.testing import CliRunner

from robin.cli import app


def test_cli_help_shows_commands():
    runner = CliRunner()
    result = runner.invoke(app, ['--help'])
    assert result.exit_code == 0
    out = result.stdout
    # Core commands should be present in help
    assert 'create-process' in out or 'create_process' in out
    assert 'add-measurement' in out or 'add_measurement' in out
    assert 'add-recommendation' in out or 'add_recommendation' in out
    assert 'status' in out
