from pyquaternion import Quaternion
import numpy as np

def convertTransformToQuaternionDict(transform):

    R = transform[0:3,0:3]
    t = transform[0:3,-1]

    q = Quaternion(matrix=transform)

    dict = {}
    dict["x"]=t[0]
    dict["y"]=t[1]
    dict["z"]=t[2]
    # dict["qx"]=
    # dict["qy"]=
    # dict["qz"]=
    # dict["qw"]=

    return dict


def convertQuaternionTotransform(quaternion):

    return


def convertRostfTotransform(translation,rotation):

    q = Quaternion(a=rotation[-1], b=rotation[0], c=rotation[1], d=rotation[2])
    v = np.array(translation)

    R = q.rotation_matrix
    V = v.reshape(3, -1)
    homo = np.array([0, 0, 0, 1])
    homo = homo.reshape(-1, 4)
    T = np.hstack((R, V))
    T = np.vstack((T, homo))

    return T

def convertTransformToRostf(transform):

    # v=transform[:3,3]
    q=Quaternion(matrix=transform)



def transform_to_pq(msg):
    """Convert a C{geometry_msgs/Transform} into position/quaternion np arrays

    @param msg: ROS message to be converted
    @return:
      - p: position as a np.array
      - q: quaternion as a numpy array (order = [x,y,z,w])
    """
    p = np.array([msg.translation.x, msg.translation.y, msg.translation.z])
    q = np.array([msg.rotation.x, msg.rotation.y,
                  msg.rotation.z, msg.rotation.w])
    return p, q

def convert_pq_to_Transform(transform,rot):
    R = rot
    transform=np.array(transform)
    V = transform.reshape(3, -1)
    homo = np.array([0, 0, 0, 1])
    homo = homo.reshape(-1, 4)
    T = np.hstack((R, V))
    T = np.vstack((T, homo))

    return T



T=np.array([[ 0.99807053 ,-0.02215161 ,-0.05800459 ,-0.11887706],
 [ 0.00306962 ,-0.91544885  ,0.40242263 ,-1.03285854],
 [-0.06201454 ,-0.40182422 ,-0.91361452 , 0.97005261],
 [ 0.    ,      0.    ,      0.    ,      1.        ]])


q=Quaternion(matrix=T)
print q[0]
print q[1]

D=12


