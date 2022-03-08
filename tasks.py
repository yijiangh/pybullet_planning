# -*- coding: utf-8 -*-
from __future__ import print_function

import codecs
import contextlib
import glob
import os
import sys
from shutil import rmtree
from xml.dom.minidom import parse

from invoke import Collection, Exit, task

# For automatic doc deployment
# from paramiko import SSHClient
# from paramiko.client import AutoAddPolicy
# from scp import SCPClient

try:
    input = raw_input
except NameError:
    pass
BASE_FOLDER = os.path.dirname(__file__)
PACKAGE_NAME = 'pybullet_planning'


class Log(object):
    def __init__(self, out=sys.stdout, err=sys.stderr):
        self.out = out
        self.err = err

    def flush(self):
        self.out.flush()
        self.err.flush()

    def write(self, message):
        self.flush()
        self.out.write(message + '\n')
        self.out.flush()

    def info(self, message):
        self.write('[INFO] %s' % message)

    def warn(self, message):
        self.write('[WARN] %s' % message)


log = Log()


def confirm(question):
    while True:
        response = input(question).lower().strip()

        if not response or response in ('n', 'no'):
            return False

        if response in ('y', 'yes'):
            return True

        print('Focus! It is either (y)es or (n)o', file=sys.stderr)


@task(default=True)
def help(ctx):
    """Lists available tasks and usage."""
    ctx.run('invoke --list')
    log.write('Use "invoke -h <taskname>" to get detailed help for a task.')


@task(help={
    'docs': 'True to generate documentation, otherwise False',
    'bytecode': 'True to clean up compiled python files, otherwise False.',
    'builds': 'True to clean up build/packaging artifacts, otherwise False.'})
def clean(ctx, docs=True, bytecode=True, builds=True):
    """Cleans the local copy from compiled artifacts."""

    with chdir(BASE_FOLDER):
        if builds:
            ctx.run('python setup.py clean')

        if bytecode:
            for root, dirs, files in os.walk(BASE_FOLDER):
                for f in files:
                    if f.endswith('.pyc'):
                        os.remove(os.path.join(root, f))
                if '.git' in dirs:
                    dirs.remove('.git')

        folders = []

        if docs:
            folders.append('docs/_build/')
            folders.append('dist/')

        if bytecode:
            folders.append('src/{}/__pycache__'.format(PACKAGE_NAME))

        if builds:
            folders.append('build/')
            folders.append('src/{}.egg-info/'.format(PACKAGE_NAME))

        for folder in folders:
            rmtree(os.path.join(BASE_FOLDER, folder), ignore_errors=True)

@task(help={
      'rebuild': 'True to clean all previously built docs before starting, otherwise False.',
      'doctest': 'True to run doctest snippets, otherwise False.',
      # 'check_links': 'True to check all web links in docs for validity, otherwise False.'
      })
def docs(ctx, rebuild=False, doctest=False): #, check_links=False):
    """Builds package's HTML documentation."""

    with chdir(BASE_FOLDER):
        if rebuild:
            clean(ctx)

        if doctest:
            ctx.run('sphinx-build -b doctest docs dist/docs/{}'.format(PACKAGE_NAME))

        ctx.run('sphinx-build -b html docs dist/docs/{}'.format(PACKAGE_NAME))

        # if check_links:
        #     ctx.run('sphinx-build -b linkcheck -c docs . dist/docs/{}'.format(PACKAGE_NAME))


@task()
def check(ctx):
    """Check the consistency of documentation, coding style and a few other things."""
    with chdir(BASE_FOLDER):
        log.write('Checking ReStructuredText formatting...')
        ctx.run('python setup.py check --strict --metadata --restructuredtext')

        # log.write('Running flake8 python linter...')
        # ctx.run('flake8 src setup.py')

        # log.write('Checking python imports...')
        # ctx.run('isort --check-only --diff --recursive src tests setup.py')

        # log.write('Checking MANIFEST.in...')
        # ctx.run('check-manifest')


@task(help={
      'checks': 'True to run all checks before testing, otherwise False.',
      'doctest': 'True to run doctest modules, otherwise False.',
      'codeblock': 'True to run codeblocks present in the documentation, otherwise False',
      'coverage': 'True to generate coverage report using pytest-cov',
      })
def test(ctx, checks=False, doctest=False, codeblock=False, coverage=False):
    """Run all tests."""

    with chdir(BASE_FOLDER):
        if checks:
            check(ctx)

        # if build:
        #     log.write('Checking build')
        #     ctx.run('python setup.py clean --all sdist') #bdist_wheel
        #     if sys.platform == 'win32':
        #         ctx.run('powershell -Command "& pip install --verbose $(ls dist/*.tar.gz | % {$_.FullName})"')
        #     else:
        #         ctx.run('pip install --verbose dist/*.tar.gz')

        # log.write('Running pytest')
        # ctx.run('pytest --doctest-modules --cov=pybullet_planning tests')

        pytest_args = ['pytest']
        if doctest:
            pytest_args.append('--doctest-modules')
        if coverage:
            pytest_args.append('--cov=pybullet_planning')

        ctx.run(" ".join(pytest_args))

        # Using --doctest-modules together with docs as the testpaths goes bananas
        if codeblock:
            ctx.run('pytest docs')

@task
def prepare_changelog(ctx):
    """Prepare changelog for next release."""
    UNRELEASED_CHANGELOG_TEMPLATE = '\nUnreleased\n----------\n\n**Added**\n\n**Changed**\n\n**Fixed**\n\n**Deprecated**\n\n**Removed**\n'

    with chdir(BASE_FOLDER):
        # Preparing changelog for next release
        with open('CHANGELOG.rst', 'r+') as changelog:
            content = changelog.read()
            start_index = content.index('----------')
            start_index = content.rindex('\n', 0, start_index - 1)
            last_version = content[start_index:start_index + 11].strip()

            if last_version == 'Unreleased':
                log.write('Already up-to-date')
                return

            changelog.seek(0)
            changelog.write(content[0:start_index] + UNRELEASED_CHANGELOG_TEMPLATE + content[start_index:])

        ctx.run('git add CHANGELOG.rst && git commit -m "Prepare changelog for next release"')


@task(help={
      'release_type': 'Type of release follows semver rules. Must be one of: major, minor, patch.',
      'bump_version': 'Bumpversion, true or false, default to false'})
def release(ctx, release_type, bump_version=False):
    """Releases the project in one swift command!"""
    if release_type not in ('patch', 'minor', 'major'):
        raise Exit('The release type parameter is invalid.\nMust be one of: major, minor, patch')

    # Run checks
    ctx.run('invoke check test') # docs

    # Bump version and git tag it
    if bump_version:
        ctx.run('bumpversion %s --verbose' % release_type)

    # Build project
    ctx.run('python setup.py clean --all sdist bdist_wheel') #bdist_wheel

    # Prepare changelog for next release
    prepare_changelog(ctx)

    # Clean up local artifacts
    clean(ctx)

    # Upload to pypi
    if confirm('Everything is ready. You are about to push to git. Are you sure? [y/N]'):
        ctx.run('git push --tags && git push')
    else:
        raise Exit('You need to manually revert the tag/commits created.')

    # if confirm('You are about to upload the release to pypi.org. Are you sure? [y/N]'):
    #     files = ['dist/*.whl', 'dist/*.gz', 'dist/*.zip']
    #     dist_files = ' '.join([pattern for f in files for pattern in glob.glob(f)])

    #     if len(dist_files):
    #         ctx.run('twine upload --skip-existing %s' % dist_files)
    #     else:
    #         raise Exit('No files found to release')
    # else:
    #     raise Exit('Aborted release')


@contextlib.contextmanager
def chdir(dirname=None):
    current_dir = os.getcwd()
    try:
        if dirname is not None:
            os.chdir(dirname)
        yield
    finally:
        os.chdir(current_dir)
