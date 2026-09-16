#!/usr/bin/env python3
"""
Make a conda-based virtualenv able to import its own standard library on Windows.

A venv created from a conda interpreter --

    C:\\...\\.venv\\pyvenv.cfg:  home = C:\\Users\\Ali\\miniconda3

-- borrows conda's standard library, and several of those stdlib modules are C
extensions that need DLLs conda keeps in `Library\\bin`:

    _ssl      -> libssl-*.dll, libcrypto-*.dll      (https, so google-genai, pip)
    _sqlite3  -> sqlite3.dll                        (jupyter's session store)
    _hashlib  -> libcrypto-*.dll
    _lzma     -> liblzma.dll
    _bz2      -> libbz2.dll
    _ctypes   -> libffi-*.dll

conda puts `Library\\bin` on PATH when an environment is activated, which is why
this never bites inside `conda activate`. A venv does not inherit it, and since
Python 3.8 extension-module DLL resolution deliberately ignores PATH -- see
https://docs.python.org/3/whatsnew/3.8.html#bpo-36085 -- every one of those
imports fails with:

    ImportError: DLL load failed while importing _ssl: The specified module
    could not be found.

The supported way back is os.add_dll_directory(), which is what this does, for
the whole class of modules at once rather than one DLL at a time.

Import this as early as possible -- before anything that might pull in ssl or
sqlite3 -- and call :func:`ensure_dll_directories`. It is a no-op everywhere
except a Windows venv whose base interpreter ships DLLs in a separate directory,
so it is safe to call unconditionally, on any platform, more than once.

This fixes processes that run repo code. It cannot fix a process that fails
before repo code is imported -- notably the Jupyter server, which dies inside
`import notebook`. For that, `robotic_testing/setup_windows_env.py` installs the
same fix as a `sitecustomize.py` in the environment itself, where Python runs it
during startup.
"""
import os
import sys

# Relative to the base interpreter's prefix. `Library\bin` is where conda puts the
# DLLs; the others are cheap to check and are where conda's own activation script
# points, so a stray dependency in one of them resolves too.
_CONDA_DLL_SUBDIRS = (
    ('Library', 'bin'),
    ('Library', 'mingw-w64', 'bin'),
    ('Library', 'usr', 'bin'),
    ('DLLs',),
)

_added = None       # cache: the work is done once per process


def base_prefix():
    """
    Where this interpreter's standard library lives.

    In a venv, sys.base_prefix is the interpreter the venv was built from -- the
    same directory pyvenv.cfg records as `home` -- and outside one it is just
    sys.prefix. Either way it is the root to look for bundled DLLs under.
    """
    return getattr(sys, 'base_prefix', sys.prefix)


def candidate_directories(prefix=None):
    """The DLL directories that exist under `prefix`, in the order they are added."""
    root = prefix if prefix is not None else base_prefix()
    found = []
    for parts in _CONDA_DLL_SUBDIRS:
        path = os.path.join(root, *parts)
        if os.path.isdir(path):
            found.append(path)
    return found


def ensure_dll_directories():
    """
    Add the base interpreter's DLL directories to the search path.

    Returns the list of directories added (empty on non-Windows, on a
    self-contained interpreter, or on a second call).
    """
    global _added
    if _added is not None:
        return _added
    _added = []

    # add_dll_directory exists only on Windows, and only from 3.8. Anywhere else
    # there is nothing to fix: the stdlib's extension modules find their own
    # dependencies through the platform's normal search.
    if os.name != 'nt' or not hasattr(os, 'add_dll_directory'):
        return _added

    for path in candidate_directories():
        try:
            os.add_dll_directory(path)
        except OSError:
            continue        # vanished between the isdir() and here; not worth failing over
        _added.append(path)
    return _added


def describe():
    """A line an operator can read, for --version-style output and error messages."""
    if os.name != 'nt':
        return 'not Windows; no DLL directories needed'
    found = candidate_directories()
    if not found:
        return 'no extra DLL directories under {} (self-contained interpreter)'.format(
            base_prefix())
    return 'DLL directories from {}:\n  {}'.format(base_prefix(), '\n  '.join(found))


# Importing this module is enough. The call is here rather than left to the caller
# because the whole point is to run before the first `import ssl` anywhere in the
# process, and an import at the top of a file is the earliest hook there is.
ensure_dll_directories()


if __name__ == '__main__':
    print(describe())
    print()
    added = ensure_dll_directories()
    print('added {} director{}'.format(len(added), 'y' if len(added) == 1 else 'ies'))
    failures = []
    for name in ('ssl', 'sqlite3', 'hashlib', 'lzma', 'bz2', 'ctypes'):
        try:
            __import__(name)
        except Exception as error:          # noqa: BLE001 -- report, never raise
            failures.append((name, error))
            print('  {:<9} FAILED  {}'.format(name, error))
        else:
            print('  {:<9} ok'.format(name))
    sys.exit(1 if failures else 0)
