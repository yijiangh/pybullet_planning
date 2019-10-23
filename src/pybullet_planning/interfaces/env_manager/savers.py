import os
import pybullet as p

from pybullet_planning.utils import CLIENT, set_client

#####################################
# Savers
# TODO: contextlib

class Saver(object):
    def restore(self):
        raise NotImplementedError()
    def __enter__(self):
        # TODO: move the saving to enter?
        pass
    def __exit__(self, type, value, traceback):
        self.restore()

class ClientSaver(Saver):
    def __init__(self, new_client=None):
        self.client = CLIENT
        if new_client is not None:
            set_client(new_client)

    def restore(self):
        set_client(self.client)

    def __repr__(self):
        return '{}({})'.format(self.__class__.__name__, self.client)

class VideoSaver(Saver):
    def __init__(self, path):
        self.path = path
        if path is None:
            self.log_id = None
        else:
            name, ext = os.path.splitext(path)
            assert ext == '.mp4'
            # STATE_LOGGING_PROFILE_TIMINGS, STATE_LOGGING_ALL_COMMANDS
            # p.submitProfileTiming("pythontest")
            self.log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, fileName=path, physicsClientId=CLIENT)

    def restore(self):
        if self.log_id is not None:
            p.stopStateLogging(self.log_id)
