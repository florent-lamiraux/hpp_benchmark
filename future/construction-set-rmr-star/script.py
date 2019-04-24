#
#  Copyright (2017) CNRS
#
#  Author: Florent Lamiraux
#
# Start hppcorbaserver before running this script
# Note that an instance of omniNames should be running in background
#

import re, os
from math import pi, fabs, sqrt
from copy import deepcopy
import hpp
from hpp.corbaserver.manipulation import ConstraintGraphFactory, Rule
from setup import ConstraintGraph, Constraints, grippers, handlesPerObjects, nCylinder, nSphere, objects, ps, robot, shapesPerObject, vf
from hpp.gepetto import PathPlayer
from state_name import StateName
import time

def cleanPaths (ps, solutions) :
  offset = 0
  for s, i in zip (solutions, xrange (100000)):
    for j in range (s - offset):
      ps.erasePath (i)
      offset += 1
    offset += 1

## Write log in a file
#
devel_dir = os.getenv ('DEVEL_DIR')

def makeRule (grasps):
  '''
  Build a rule that will generate a state where each specified gripper will
  grasp the specify handle.

  This rule is used in the construction of the constraint graph to generate the
  corresponding state
  '''
  _grippers = list ()
  _handles = list ()
  for (g,h) in grasps:
    _grippers.append (g)
    _handles.append (h)
  for g in grippers:
    if not g in _grippers:
      _grippers.append (g)
  _handles += (len (_grippers) - len (_handles)) * ['^$']
  return Rule (grippers = _grippers, handles = _handles, link = True)

def getTransitionConnectingStates (graph, s1, s2):
  '''
  Get transition edge connecting two states of the constraint graph

  Input:
    - graph: the manipulation constraint graph,
    - s1, s2: states of the constraint graph
  '''
  # Filter out waypoint and levelset edges from the constraint graph
  p = re.compile ('\|.*_.*')
  _edges = set (filter (lambda s : p.search (s) is None, graph.edges.keys ()))
  for edge in _edges:
    node1, node2 = cg.getNodesConnectedByEdge (edge)
    if StateName (node1) == s1 and StateName (node2) == s2:
      return edge



def getEdges (graph, nodes, exploreNodes):
  '''
  Build list of edges linking a list of nodes

  Input:
    - graph: the manipulation constraint graph,
    - nodes: list of nodes the solution path should visit,
    - exploreNodes: states in which to perform path planning to cross an edge
                    this list contains the same number of element as the
                    resulting list of edges.
  Return
    - edges: list of edges the resulting path should cross,
    - loops: list of loop transitions of each element of list "exploreNodes".
             These transitions are used to perform exploration in order to cross
             edges of list "edges".
  '''
  edges = list ()
  loops = list ()
  # Filter out waypoint and levelset edges from the constraint graph
  p = re.compile ('\|.*_.*')
  _edges = set (filter (lambda s : p.search (s) is None, graph.edges.keys ()))
  for n1, n2, n3 in zip (nodes, nodes [1:], exploreNodes):
    edgeFound = False
    loopFound = False
    for edge in _edges:
      node1, node2 = cg.getNodesConnectedByEdge (edge)
      if StateName (node1) == n1 and StateName (node2) == n2:
        edges.append (edge)
        edgeFound = True
      if StateName (node1) == n3 and StateName (node2) == n3:
        loops.append (edge)
        loopFound = True
      if edgeFound and loopFound:
        break
    if not edgeFound : raise RuntimeError \
       ('cannot find edge from node "{0}" to "{1}"'.format (n1, n2))
    if not loopFound : raise RuntimeError \
       ('cannot find edge from node "{0}" to "{1}"'.format (n1, n1))
  return edges, loops

# infinite norm between vectors
dC = lambda q1,q2: reduce (lambda x,y : x if fabs (y [0]- y [1]) < x \
                           else fabs (y [0]- y [1]), zip (q1, q2), 0)

display = False
if display:
  v = vf.createViewer ()
  pp = PathPlayer (v)
else:
  v = lambda x: None

## Initial configuration of manipulator arms
q0_r0 = [pi/6, -pi/2, pi/2, 0, 0, 0,]
q0_r1 = q0_r0 [::]

## Generate initial configurations of spheres
q0_spheres = list ()
q1_spheres = list ()
i = 0
y = -0.062
while i < nSphere:
  q0_spheres.append ([-0.45 - .1*(i/2), y, 0.025, 0, 0, 0, 1])
  q1_spheres.append ([ 0.00 - .1*(i/2), y, 0.025, (-1)**i * 0.5, -(-1)**i * 0.5,
                       0.5, 0.5])
  i+=1; y = -y

## Generate initial configurations of cylinders
q0_cylinders = list ()
i = 0
y = -0.04
while i < nCylinder:
  q0_cylinders.append ([0.45 + .1*(i/2), y, 0.025, 0, 0, 0, 1])
  i+=1; y = -y
q1_cylinders = deepcopy (q0_cylinders)
q1_cylinders [0] = [ 0.00, 0.00, 0.025, 0, 0, sqrt (2)/2, sqrt (2)/2]

q0 = q0_r0 + q0_r1 + sum (q0_spheres, []) + sum (q0_cylinders, [])
q1 = q0_r0 + q0_r1 + sum (q1_spheres, []) + sum (q1_cylinders, [])

if display:
  v (q0)

# List of nodes composing the assembly sequence
nodes = list ()
# List of rules that define all the nodes
rules = list ()
# List of grasps for each node. From any node to the next one, one grasp is
# added or removed
grasps = set ()
# list of nodes that are explored to cross each edge
exploreNodes = list ()
nodes.append (StateName (grasps))
rules.append (makeRule (grasps = grasps))
# grasp sphere0
grasps.add (('r0/gripper', 'sphere0/handle'))
nodes.append (StateName (grasps))
rules.append (makeRule (grasps = grasps))
exploreNodes.append (nodes [-2])

# grasp cylinder0
grasps.add (('r1/gripper', 'cylinder0/handle'))
nodes.append (StateName (grasps))
rules.append (makeRule (grasps = grasps))
exploreNodes.append (nodes [-2])

# assemble cylinder0 and sphere0
grasps.add (('cylinder0/magnet0', 'sphere0/magnet'))
nodes.append (StateName (grasps))
rules.append (makeRule (grasps = grasps))
exploreNodes.append (nodes [-2])

# release sphere0
grasps.remove (('r0/gripper', 'sphere0/handle'))
nodes.append (StateName (grasps))
rules.append (makeRule (grasps = grasps))
exploreNodes.append (nodes [-1])

# grasp sphere1
grasps.add (('r0/gripper', 'sphere1/handle'))
nodes.append (StateName (grasps))
rules.append (makeRule (grasps = grasps))
exploreNodes.append (nodes [-2])

# assemble sphere1
grasps.add (('cylinder0/magnet1', 'sphere1/magnet'))
nodes.append (StateName (grasps))
rules.append (makeRule (grasps = grasps))
exploreNodes.append (nodes [-2])

# release sphere1
grasps.remove (('r0/gripper', 'sphere1/handle'))
nodes.append (StateName (grasps))
rules.append (makeRule (grasps = grasps))
exploreNodes.append (nodes [-2])

# release cylinder0 : put assembly on the ground
grasps.remove (('r1/gripper', 'cylinder0/handle'))
nodes.append (StateName (grasps))
rules.append (makeRule (grasps = grasps))
exploreNodes.append (nodes [-2])

cg = ConstraintGraph (robot, 'assembly')
factory = ConstraintGraphFactory (cg)
factory.setGrippers (grippers)
#factory.environmentContacts (['table/pancake_table_table_top'])
factory.setObjects (objects, handlesPerObjects, shapesPerObject)
factory.setRules (rules)
factory.generate ()
cg.initialize ()

edges, loops = getEdges (graph = cg, nodes = nodes, exploreNodes = exploreNodes)
ps.selectPathProjector ('Progressive', .05)
ps.selectPathPlanner ('RMR*')
ps.setParameter ('kPRM*/numberOfNodes', 10)
ps.setParameter ('RMR*/nbTryRandomConfig', 400)

## Add a node to move robots in initial configurations
nodes.append (nodes [-1])
edges.append (getTransitionConnectingStates (graph = cg, s1 = nodes [-2],
                                             s2 = nodes [-1]))
loops.append (getTransitionConnectingStates (graph = cg, s1 = nodes [-1],
                                             s2 = nodes [-1]))
exploreNodes.append (nodes [-1])


#ps.addPathOptimizer ('Graph-RandomShortcut')
#ps.addPathOptimizer ('SplineGradientBased_bezier1')

solutions = list ()
# Solve 20 times the problem
import datetime as dt
totalTime = dt.timedelta (0)
totalNumberNodes = 0
q_init = q0
q_goal = q1
N = 0
for i in range (N):
  numberNodes = 0
  # Try 20 times to solve the problem and stop at first success
  t1 = dt.datetime.now ()
  for i in range (20):
    try:
      ps.clearRoadmap ()
      ps.setInitialConfig (q_init)
      ps.resetGoalConfigs ()
      ps.addGoalConfig (q_goal)
      ps.solve ()
      n = ps.numberNodes ()
      numberNodes += n
      break;
    except RuntimeError:
      n = ps.numberNodes ()
      numberNodes += n
  t2 = dt.datetime.now ()
  totalTime += t2 - t1
  print (t2-t1)
  print ("Number nodes: " + str(n))
  totalNumberNodes += numberNodes

if N != 0:
  print ("Average time: " + str ((totalTime.seconds+1e-6*totalTime.microseconds)/float (N)))
  print ("Average number nodes: " + str (totalNumberNodes/float(N)))
  cleanPaths (ps, solutions)
