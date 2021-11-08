import os
import json


# FIXME: Currently AUTHORS gets saved as the license automatically by setuptools. This isn't an issue, but can lead to
#   future confusion.

name = 'c1c0_object_detection'
README_NAME = 'README.md'
description = ''

_env_falsey_vals = ['n', 'no', 'false', '0']

# import setuptools' setup or import distutils.core.setup if not possible
try:
    if os.environ.get('PYTHON_USE_SETUPTOOLS', '').lower() in _env_falsey_vals:
        raise ImportError
    # noinspection PyUnresolvedReferences
    from setuptools import setup, find_packages
    SETUPTOOLS = True
except ImportError:
    from distutils.core import setup
    SETUPTOOLS = False


def _load_authors():
    f = open('AUTHORS', 'r')
    try:
        author_data = f.read().split('\n')
        return author_data[0], author_data[1], author_data[2]
    except IndexError:
        e = 'Error retrieving Authors (Malformed)'
        return e, e, e
    finally:
        f.close()


def _load_manual_packages():
    with open('PACKAGES', 'r') as f:
        return json.load(f)['packages']


# Used if SETUPTOOLS is false
_packages = find_packages() if SETUPTOOLS else _load_manual_packages()
print('SETUPTOOLS=', SETUPTOOLS)
print('packages=', _packages)


def readme(_readme):
    with open(os.path.join(os.path.dirname(__file__), _readme), 'r') as f:
        return f.read()


def get_short_description(_readme: str):
    lines = _readme.split('\n')
    idx = -1
    for i in range(len(lines)):
        if lines[i].startswith(name):
            # 2 indexes forward because of rst formatting
            idx = i+2
            break
    if idx != -1:
        return lines[idx]
    return 'No description found.'


def get_version():
    with open('VERSION', 'r') as f:
        return f.read().strip()

def _load_installs():
    with open('requirements.txt', 'r') as f:
        return f.read().splitlines()

def _setup(_readme):
    print('name=', name)
    long_description = readme(_readme)
    short_description = get_short_description(long_description)
    print('short_description=', short_description)
    author, author_email, url = _load_authors()
    print('author=', author)
    print('author_email=', author_email)
    print('url=', url)
    version = get_version()
    print('version=', version)

    installs = _load_installs()
    setup(
        name=name,
        version=version,
        author=author,
        author_email=author_email,
        url=url,
        description=short_description,
        long_description=long_description,
        packages=_packages,
        install_requires=installs
    )


_setup(README_NAME)

