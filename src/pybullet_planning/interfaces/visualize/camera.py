import math
import numpy as np
from collections import namedtuple
import pybullet as p

from pybullet_planning.utils import CLIENT

CameraInfo = namedtuple('CameraInfo', ['width', 'height', 'viewMatrix', 'projectionMatrix', 'cameraUp', 'cameraForward',
                                       'horizontal', 'vertical', 'yaw', 'pitch', 'dist', 'target'])

def get_camera():
    return CameraInfo(*p.getDebugVisualizerCamera(physicsClientId=CLIENT))

def set_camera(yaw, pitch, distance, target_position=np.zeros(3)):
    p.resetDebugVisualizerCamera(distance, yaw, pitch, target_position, physicsClientId=CLIENT)

def get_pitch(point):
    dx, dy, dz = point
    return np.math.atan2(dz, np.sqrt(dx ** 2 + dy ** 2))

def get_yaw(point):
    dx, dy, dz = point
    return np.math.atan2(dy, dx)

def set_camera_pose(camera_point, target_point=np.zeros(3)):
    delta_point = np.array(target_point) - np.array(camera_point)
    distance = np.linalg.norm(delta_point)
    yaw = get_yaw(delta_point) - np.pi/2 # TODO: hack
    pitch = get_pitch(delta_point)
    p.resetDebugVisualizerCamera(distance, math.degrees(yaw), math.degrees(pitch),
                                 target_point, physicsClientId=CLIENT)

def set_camera_pose2(world_from_camera, distance=2):
    target_camera = np.array([0, 0, distance])
    target_world = tform_point(world_from_camera, target_camera)
    camera_world = point_from_pose(world_from_camera)
    set_camera_pose(camera_world, target_world)
    #roll, pitch, yaw = euler_from_quat(quat_from_pose(world_from_camera))
    # TODO: assert that roll is about zero?
    #p.resetDebugVisualizerCamera(cameraDistance=distance, cameraYaw=math.degrees(yaw), cameraPitch=math.degrees(-pitch),
    #                             cameraTargetPosition=target_world, physicsClientId=CLIENT)

CameraImage = namedtuple('CameraImage', ['rgbPixels', 'depthPixels', 'segmentationMaskBuffer'])

def demask_pixel(pixel):
    # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/segmask_linkindex.py
    # Not needed when p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX is not enabled
    #if 0 <= pixel:
    #    return None
    # Returns a large value when undefined
    body = pixel & ((1 << 24) - 1)
    link = (pixel >> 24) - 1
    return body, link

def save_image(filename, rgba):
    import scipy
    if filename.endswith('.jpg'):
        scipy.misc.imsave(filename, rgba[:, :, :3])
    elif filename.endswith('.png'):
        scipy.misc.imsave(filename, rgba)  # (480, 640, 4)
        # scipy.misc.toimage(image_array, cmin=0.0, cmax=...).save('outfile.jpg')
    else:
        raise ValueError(filename)
    print('Saved image at {}'.format(filename))

def get_projection_matrix(width, height, vertical_fov, near, far):
    """
    OpenGL projection matrix
    :param width:
    :param height:
    :param vertical_fov: vertical field of view in radians
    :param near:
    :param far:
    :return:
    """
    # http://www.songho.ca/opengl/gl_projectionmatrix.html
    # http://www.songho.ca/opengl/gl_transform.html#matrix
    # https://www.edmundoptics.fr/resources/application-notes/imaging/understanding-focal-length-and-field-of-view/
    # gluPerspective() requires only 4 parameters; vertical field of view (FOV),
    # the aspect ratio of width to height and the distances to near and far clipping planes.
    aspect = float(width) / height
    fov_degrees = math.degrees(vertical_fov)
    projection_matrix = p.computeProjectionMatrixFOV(fov=fov_degrees, aspect=aspect,
                                                     nearVal=near, farVal=far, physicsClientId=CLIENT)
    # projection_matrix = p.computeProjectionMatrix(0, width, height, 0, near, far, physicsClientId=CLIENT)
    return projection_matrix
    #return np.reshape(projection_matrix, [4, 4])

def apply_alpha(color, alpha=1.0):
    return tuple(color[:3]) + (alpha,)

def spaced_colors(n, s=1, v=1):
    return [colorsys.hsv_to_rgb(h, s, v) for h in np.linspace(0, 1, n, endpoint=False)]

def image_from_segmented(segmented, color_from_body=None):
    if color_from_body is None:
        bodies = get_bodies()
        color_from_body = dict(zip(bodies, spaced_colors(len(bodies))))
    image = np.zeros(segmented.shape[:2] + (3,))
    for r in range(segmented.shape[0]):
        for c in range(segmented.shape[1]):
            body, link = segmented[r, c, :]
            image[r, c, :] = color_from_body.get(body, (0, 0, 0))
    return image

def get_image(camera_pos, target_pos, width=640, height=480, vertical_fov=60.0, near=0.02, far=5.0,
              segment=False, segment_links=False):
    # computeViewMatrixFromYawPitchRoll
    view_matrix = p.computeViewMatrix(cameraEyePosition=camera_pos, cameraTargetPosition=target_pos,
                                      cameraUpVector=[0, 0, 1], physicsClientId=CLIENT)
    projection_matrix = get_projection_matrix(width, height, vertical_fov, near, far)
    if segment:
        if segment_links:
            flags = p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX
        else:
            flags = 0
    else:
        flags = p.ER_NO_SEGMENTATION_MASK
    image = CameraImage(*p.getCameraImage(width, height, viewMatrix=view_matrix,
                                          projectionMatrix=projection_matrix,
                                          shadow=False,
                                          flags=flags,
                                          renderer=p.ER_TINY_RENDERER, # p.ER_BULLET_HARDWARE_OPENGL
                                          physicsClientId=CLIENT)[2:])
    depth = far * near / (far - (far - near) * image.depthPixels)
    # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/pointCloudFromCameraImage.py
    # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/getCameraImageTest.py
    segmented = None
    if segment:
        segmented = np.zeros(image.segmentationMaskBuffer.shape + (2,))
        for r in range(segmented.shape[0]):
            for c in range(segmented.shape[1]):
                pixel = image.segmentationMaskBuffer[r, c]
                segmented[r, c, :] = demask_pixel(pixel)
    return CameraImage(image.rgbPixels, depth, segmented)

def set_default_camera():
    set_camera(160, -35, 2.5, Point())
