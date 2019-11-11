import pytest

def pytest_addoption(parser):
    parser.addoption('--viewer', action='store_true', help='Enables the pybullet viewer')

@pytest.fixture
def viewer(request):
    return request.config.getoption("--viewer")
