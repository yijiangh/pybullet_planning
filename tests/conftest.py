import pytest
from fixtures import *

def pytest_addoption(parser):
    parser.addoption('--viewer', action='store_true', help='Enables the pybullet viewer')
    parser.addoption('--print_debug', action='store_true', help='print out info')

@pytest.fixture
def viewer(request):
    return request.config.getoption("--viewer")

@pytest.fixture
def print_debug(request):
    return request.config.getoption("--print_debug")
