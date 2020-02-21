from pybullet_planning.interfaces import get_relative_pose, get_link_subtree, clone_body, set_static, get_link_pose, \
    set_pose, multiply

##############################################

class EndEffector(object):
    """a convenient class for creating and manipulating an end effector

    Note: the end effector needs to be modeled in the robot's URDF.

    """
    def __init__(self, robot, ee_link, tool_link, **kwargs):
        """[summary]

        Parameters
        ----------
        robot : [type]
            [description]
        ee_link : int
            pb link index of the link where the end effector gets attached to
        tool_link : int
            pb link index of the TCP (tip) link
        """
        self.robot = robot
        self.ee_link = ee_link
        self.tool_link = tool_link
        self.tool_from_ee = get_relative_pose(self.robot, self.ee_link, self.tool_link)
        tool_links = get_link_subtree(robot, self.ee_link)
        self.body = clone_body(robot, links=tool_links, **kwargs)
        set_static(self.body)
        # for link in get_all_links(tool_body):
        #    set_color(tool_body, np.zeros(4), link)
    def get_tool_pose(self):
        return get_link_pose(self.robot, self.tool_link)
    def set_pose(self, tool_pose):
        pose = multiply(tool_pose, self.tool_from_ee)
        set_pose(self.body, pose)
        return pose
    @property
    def tool_from_root(self):
        return self.tool_from_ee
    def __repr__(self):
        return '{}({}, {})'.format(self.__class__.__name__, self.robot, self.body)
