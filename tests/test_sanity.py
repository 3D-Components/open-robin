"""Sanity tests for the ROBIN project."""

import sys
from pathlib import Path


def test_python_version():
    """Test that we're running a supported Python version."""
    assert sys.version_info >= (3, 12), 'Python 3.12+ is required'


def test_package_structure():
    """Test that the basic package structure exists."""
    # Get the project root
    project_root = Path(__file__).parent.parent

    # Check that src directory exists
    src_dir = project_root / 'robin'
    assert src_dir.exists(), 'robin source directory should exist'
    assert (src_dir / '__init__.py').exists(), 'robin/__init__.py should exist'


def test_basic_imports():
    """Test that basic Python functionality works."""
    # Test standard library imports
    import os
    import sys
    import pathlib

    assert os is not None
    assert sys is not None
    assert pathlib is not None


def test_src_module_import():
    """Test that the src module can be imported."""
    try:
        # Add src to path if not already there
        project_root = Path(__file__).parent.parent
        src_path = str(project_root / 'robin')
        if src_path not in sys.path:
            sys.path.insert(0, src_path)

        # This should work since robin/__init__.py exists
        import robin  # noqa: F401

    except ImportError as e:
        # If import fails, at least we know the test framework is working
        assert False, f'Could not import src module: {e}'
