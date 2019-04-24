from math import pi
from hpp.corbaserver.manipulation import Client, ConstraintGraph, \
  Constraints, ProblemSolver
from hpp.corbaserver.manipulation.ur5 import Robot
from hpp.gepetto.manipulation import ViewerFactory
from hpp.gepetto import PathPlayer
from hpp.corbaserver import loadServerPlugin
loadServerPlugin ("corbaserver", "manipulation-corba.so")
Client ().problem.resetProblem ()

Robot.urdfName = "ur3_gripper"
Robot.urdfSuffix = ""
Robot.srdfSuffix = ""

class Cylinder_08 (object):
  rootJointType = 'freeflyer'
  packageName = 'hpp_environments'
  urdfName = 'construction_set/cylinder_08'
  urdfSuffix = ""
  srdfSuffix = ""

class Cylinder_13 (object):
  rootJointType = 'freeflyer'
  packageName = 'hpp_environments'
  urdfName = 'construction_set/cylinder_13'
  urdfSuffix = ""
  srdfSuffix = ""

class Sphere (object):
  rootJointType = 'freeflyer'
  packageName = 'hpp_environments'
  urdfName = 'construction_set/sphere'
  urdfSuffix = ""
  srdfSuffix = ""

class Ground (object):
  rootJointType = 'anchor'
  packageName = 'hpp_environments'
  urdfName = 'construction_set/ground'
  urdfSuffix = ""
  srdfSuffix = ""

robot = Robot ('2ur5-sphere', 'r0')
robot.setJointPosition ('r0/root_joint', [-.25, 0, 0, 0, 0, 0, 1])

ps = ProblemSolver (robot)
ps.setErrorThreshold (1e-4)
ps.setMaxIterProjection (40)

vf = ViewerFactory (ps)

#
#  Load robots and objets
#    - 2 UR3,
#    - 4 spheres,
#    - 4 short cylinders,

vf.loadRobotModel (Robot, "r1")
robot.setJointPosition ('r1/root_joint', [.25, 0, 0, 0, 0, 1, 0])

# Change bounds of robots to increase workspace and avoid some collisions
robot.setJointBounds ('r0/shoulder_pan_joint', [-pi, 4])
robot.setJointBounds ('r1/shoulder_pan_joint', [-pi, 4])
robot.setJointBounds ('r0/shoulder_lift_joint', [-pi, 0])
robot.setJointBounds ('r1/shoulder_lift_joint', [-pi, 0])
robot.setJointBounds ('r0/elbow_joint', [-2.6, 2.6])
robot.setJointBounds ('r1/elbow_joint', [-2.6, 2.6])

vf.loadEnvironmentModel (Ground, 'ground')

nSphere = 2
nCylinder = 2

objects = list ()
for i in range (nSphere):
  vf.loadObjectModel (Sphere, 'sphere{0}'.format (i))
  robot.setJointBounds ('sphere{0}/root_joint'.format (i),
                        [-1.,1.,-1.,1.,-.1,1.,-1.0001, 1.0001,-1.0001, 1.0001,
                         -1.0001, 1.0001,-1.0001, 1.0001,])
  objects.append ('sphere{0}'.format (i))

for i in range (nCylinder):
  vf.loadObjectModel (Cylinder_08, 'cylinder{0}'.format (i))
  robot.setJointBounds ('cylinder{0}/root_joint'.format (i),
                        [-1.,1.,-1.,1.,-.1,1.,-1.0001, 1.0001,-1.0001, 1.0001,
                         -1.0001, 1.0001,-1.0001, 1.0001,])
  objects.append ('cylinder{0}'.format (i))

r_joints = []
for i in range (2):
  r_joints.append (['r{0}/shoulder_pan_joint'.format (i),
                    'r{0}/shoulder_lift_joint'.format (i),
                    'r{0}/elbow_joint'.format (i),
                    'r{0}/wrist_1_joint'.format (i),
                    'r{0}/wrist_2_joint'.format (i),
                    'r{0}/wrist_3_joint'.format (i),])

## Gripper
#
grippers = ['cylinder{0}/magnet0'.format (i) for i in range (nCylinder)]
grippers += ['cylinder{0}/magnet1'.format (i) for i in range (nCylinder)]
grippers += ["r{0}/gripper".format (i) for i in range (2)]

## Handles
#
handlesPerObjects = [['sphere{0}/handle'.format (i),
                      'sphere{0}/magnet'.format (i)] for i in range (nSphere)]
handlesPerObjects += [['cylinder{0}/handle'.format (i)] for i in
                      range (nCylinder)]
## Contact surfaces
shapesPerObject = [[] for o in objects]

## Constraints
#
for i in range (nSphere):
  placementName = "place_sphere{0}".format (i)
  ps.createTransformationConstraint (placementName, "",
                                     "sphere{0}/root_joint".format (i),
                                     [0, 0, 0.025, 0, 0, 0, 1],
                                     [False, False, True, True, True, False])
for i in range (nCylinder):
  placementName = "place_cylinder{0}".format (i)
  ps.createTransformationConstraint (placementName, "",
                                     "cylinder{0}/root_joint".format (i),
                                     [0, 0, 0.025, 0, 0, 0, 1],
                                     [False, False, True, True, True, False])

for i in range (nSphere):
  placementName = "preplace_sphere{0}".format (i)
  ps.createTransformationConstraint (placementName, "",
                                     "sphere{0}/root_joint".format (i),
                                     [0, 0, 0.075, 0, 0, 0, 1],
                                     [False, False, True, True, True, False])
for i in range (nCylinder):
  placementName = "preplace_cylinder{0}".format (i)
  ps.createTransformationConstraint (placementName, "",
                                     "cylinder{0}/root_joint".format (i),
                                     [0, 0, 0.075, 0, 0, 0, 1],
                                     [False, False, True, True, True, False])
