"""ROBIN core package."""

import importlib

__all__ = ['ai']


def __getattr__(name):
    if name == 'ai':
        return importlib.import_module('robin.ai')
    raise AttributeError(f'module {__name__!r} has no attribute {name!r}')
