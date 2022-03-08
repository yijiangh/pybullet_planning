import os
import time
import platform
import math
import inspect
import signal
from contextlib import contextmanager
import pstats
import cProfile
import logging

from .shared_const import INF

def is_remote():
    return 'SSH_CONNECTION' in os.environ

def is_darwin(): # TODO: change loading accordingly
    return platform.system() == 'Darwin' # platform.release()
    #return sys.platform == 'darwin'

def is_windows():
    return platform.system() == 'Windows'

##################################################

def get_function_name(depth=1):
   return inspect.stack()[depth][3]

##################################################

BYTES_PER_KILOBYTE = math.pow(2, 10)
BYTES_PER_GIGABYTE = math.pow(2, 30)
KILOBYTES_PER_GIGABYTE = BYTES_PER_GIGABYTE / BYTES_PER_KILOBYTE

def get_memory_in_kb():
    # https://pypi.org/project/psutil/
    # https://psutil.readthedocs.io/en/latest/
    import psutil
    #rss: aka "Resident Set Size", this is the non-swapped physical memory a process has used. (bytes)
    #vms: aka "Virtual Memory Size", this is the total amount of virtual memory used by the process. (bytes)
    #shared: (Linux) memory that could be potentially shared with other processes.
    #text (Linux, BSD): aka TRS (text resident set) the amount of memory devoted to executable code.
    #data (Linux, BSD): aka DRS (data resident set) the amount of physical memory devoted to other than executable code.
    #lib (Linux): the memory used by shared libraries.
    #dirty (Linux): the number of dirty pages.
    #pfaults (macOS): number of page faults.
    #pageins (macOS): number of actual pageins.
    process = psutil.Process(os.getpid())
    #process.pid()
    #process.ppid()
    pmem = process.memory_info() # this seems to actually get the current memory!
    return pmem.vms / BYTES_PER_KILOBYTE
    #print(process.memory_full_info())
    #print(process.memory_percent())
    # process.rlimit(psutil.RLIMIT_NOFILE)  # set resource limits (Linux only)
    #print(psutil.virtual_memory())
    #print(psutil.swap_memory())
    #print(psutil.pids())

def elapsed_time(start_time):
    return time.time() - start_time

def raise_timeout(signum, frame):
    raise TimeoutError()

@contextmanager
def timeout(duration):
    # https://www.jujens.eu/posts/en/2018/Jun/02/python-timeout-function/
    # https://code-maven.com/python-timeout
    # https://pypi.org/project/func-timeout/
    # https://pypi.org/project/timeout-decorator/
    # https://eli.thegreenplace.net/2011/08/22/how-not-to-set-a-timeout-on-a-computation-in-python
    # https://docs.python.org/3/library/signal.html
    # https://docs.python.org/3/library/contextlib.html
    # https://stackoverflow.com/a/22348885
    # TODO: this is *nix only, raise on Windows environment
    if is_windows():
        warnings.warn('Currently, we do not support clone bodies that are created from an obj file.')
        return

    assert 0 < duration
    if duration == INF:
        yield
        return
    # Register a function to raise a TimeoutError on the signal
    signal.signal(signal.SIGALRM, raise_timeout)
    # Schedule the signal to be sent after ``duration``
    signal.alarm(int(math.ceil(duration)))
    try:
        yield
    except TimeoutError as e:
        print('Timeout after {} sec'.format(duration))
        #traceback.print_exc()
        pass
    finally:
        # Unregister the signal so it won't be triggered
        # if the timeout is not reached
        signal.signal(signal.SIGALRM, signal.SIG_IGN)

#######################################

@contextmanager
def profiler(field='tottime', num=10):
    pr = cProfile.Profile()
    pr.enable()
    yield
    pr.disable()
    pstats.Stats(pr).sort_stats(field).print_stats(num) # cumtime | tottime

#######################################
# borrowed from: https://github.com/compas-dev/compas_fab/blob/3efe608c07dc5b08653ee4132a780a3be9fb93af/src/compas_fab/backends/pybullet/utils.py#L83
def get_logger(name):
    logger = logging.getLogger(name)

    try:
        from colorlog import ColoredFormatter
        formatter = ColoredFormatter("%(log_color)s%(levelname)-8s%(reset)s %(message)s",
                                     datefmt=None,
                                     reset=True,
                                     log_colors={'DEBUG': 'cyan', 'INFO': 'green',
                                                 'WARNING': 'yellow',
                                                 'ERROR': 'red', 'CRITICAL': 'red',
                                                 }
                                     )
    except ImportError:
        formatter = logging.Formatter('[%(levelname)s] %(message)s')

    handler = logging.StreamHandler()
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    logger.setLevel(logging.INFO)

    return logger

LOGGER = get_logger(__name__)
