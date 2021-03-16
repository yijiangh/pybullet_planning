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
        # pass
        return self
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

#####################################

class ConfSaver(Saver):
    def __init__(self, body): #, joints):
        from pybullet_planning.interfaces.robots.joint import get_configuration
        self.body = body
        self.conf = get_configuration(body)

    def apply_mapping(self, mapping):
        self.body = mapping.get(self.body, self.body)

    def restore(self):
        from pybullet_planning.interfaces.robots.joint import set_configuration
        set_configuration(self.body, self.conf)

    def __repr__(self):
        return '{}({})'.format(self.__class__.__name__, self.body)

class BodySaver(Saver):
    def __init__(self, body): #, pose=None):
        #if pose is None:
        #    pose = get_pose(body)
        self.body = body
        self.pose_saver = PoseSaver(body)
        self.conf_saver = ConfSaver(body)
        self.savers = [self.pose_saver, self.conf_saver]
        # TODO: store velocities

    def apply_mapping(self, mapping):
        for saver in self.savers:
            saver.apply_mapping(mapping)

    def restore(self):
        for saver in self.savers:
            saver.restore()

    def __repr__(self):
        return '{}({})'.format(self.__class__.__name__, self.body)

class WorldSaver(Saver):
    def __init__(self):
        from pybullet_planning.interfaces.robots.body import get_bodies
        self.body_savers = [BodySaver(body) for body in get_bodies()]
        # TODO: add/remove new bodies

    def restore(self):
        for body_saver in self.body_savers:
            body_saver.restore()

class PoseSaver(Saver):
    def __init__(self, body):
        from pybullet_planning.interfaces.robots.body import get_pose, get_velocity
        self.body = body
        self.pose = get_pose(self.body)
        self.velocity = get_velocity(self.body)

    def apply_mapping(self, mapping):
        self.body = mapping.get(self.body, self.body)

    def restore(self):
        from pybullet_planning.interfaces.robots.body import set_pose, set_velocity
        set_pose(self.body, self.pose)
        set_velocity(self.body, *self.velocity)

    def __repr__(self):
        return '{}({})'.format(self.__class__.__name__, self.body)
