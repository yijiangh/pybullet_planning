#####################################
# TODO: clean up these data paths

DRAKE_PATH = 'models/drake/'

# Models

# Robots
MODEL_DIRECTORY = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, 'models/'))
ROOMBA_URDF = 'models/turtlebot/roomba.urdf'
TURTLEBOT_URDF = 'models/turtlebot/turtlebot_holonomic.urdf'
DRAKE_IIWA_URDF = 'models/drake/iiwa_description/urdf/iiwa14_polytope_collision.urdf'
WSG_50_URDF = 'models/drake/wsg_50_description/urdf/wsg_50_mesh_visual.urdf' # wsg_50 | wsg_50_mesh_visual | wsg_50_mesh_collision
#SCHUNK_URDF = 'models/drake/wsg_50_description/sdf/schunk_wsg_50.sdf'
PANDA_HAND_URDF = "models/franka_description/robots/hand.urdf"
PANDA_ARM_URDF = "models/franka_description/robots/panda_arm_hand.urdf"

# Pybullet Robots
#PYBULLET_DIRECTORY = add_data_path()
KUKA_IIWA_URDF = "kuka_iiwa/model.urdf"
KUKA_IIWA_GRIPPER_SDF = "kuka_iiwa/kuka_with_gripper.sdf"
R2D2_URDF = "r2d2.urdf"
MINITAUR_URDF = "quadruped/minitaur.urdf"
HUMANOID_MJCF = "mjcf/humanoid.xml"
HUSKY_URDF = "husky/husky.urdf"
RACECAR_URDF = 'racecar/racecar.urdf' # racecar_differential.urdf
PR2_GRIPPER = 'pr2_gripper.urdf'
WSG_GRIPPER = 'gripper/wsg50_one_motor_gripper.sdf' # wsg50_one_motor_gripper | wsg50_one_motor_gripper_new

# Pybullet Objects
KIVA_SHELF_SDF = "kiva_shelf/model.sdf"
FLOOR_URDF = 'plane.urdf'
TABLE_URDF = 'table'

# Objects
SMALL_BLOCK_URDF = "models/drake/objects/block_for_pick_and_place.urdf"
BLOCK_URDF = "models/drake/objects/block_for_pick_and_place_mid_size.urdf"
SINK_URDF = 'models/sink.urdf'
STOVE_URDF = 'models/stove.urdf'
