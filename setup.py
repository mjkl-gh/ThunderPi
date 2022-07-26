"""setup.py"""

import os
import sys

from setuptools import setup
from setuptools.command.test import test as TestCommand


class Tox(TestCommand):
    user_options = [("tox-args=", "a", "Arguments to pass to tox")]

    def initialize_options(self):
        TestCommand.initialize_options(self)
        self.tox_args = None

    def finalize_options(self):
        TestCommand.finalize_options(self)
        self.test_args = []
        self.test_suite = True

    def run_tests(self):
        import shlex

        import tox

        if self.tox_args:
            errno = tox.cmdline(args=shlex.split(self.tox_args))
        else:
            errno = tox.cmdline(self.test_args)
        sys.exit(errno)


def read_content(filepath):
    with open(filepath) as fobj:
        return fobj.read()


classifiers = [
    "Development Status :: 3 - Alpha",
    "Intended Audience :: Developers",
    "License :: OSI Approved :: GNU General Public License v3 or later (GPLv3+)",
    "Programming Language :: Python",
    "Programming Language :: Python :: 3",
    "Programming Language :: Python :: 3.7",
    "Programming Language :: Python :: 3.8",
    "Programming Language :: Python :: Implementation :: CPython",
    "Programming Language :: Python :: Implementation :: PyPy",
]


long_description = read_content("README.rst") + read_content(
    os.path.join("docs/source", "CHANGELOG.rst")
)

requires = ["setuptools"]

extras_require = {
    "reST": ["Sphinx"],
}
if os.environ.get("READTHEDOCS", None):
    extras_require["reST"].append("recommonmark")

setup(
    name="thunderpi",
    version="0.1.0",
    description="##### ToDo: Rewrite me #####",
    long_description=long_description,
    long_description_content_type="text/x-rst",
    author="mjkl-gh",
    author_email="9350991+mjkl-gh@users.noreply.github.com",
    url="https://github.com/mjkl/thunderpi",
    classifiers=classifiers,
    packages=["thunderpi"],
    data_files=[],
    install_requires=requires,
    include_package_data=True,
    extras_require=extras_require,
    tests_require=["tox"],
    cmdclass={"test": Tox},
)
