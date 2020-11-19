#!/usr/bin/env python
#
#  Copyright (2017) CNRS
#
#  Author: Florent Lamiraux
#
# Start hppcorbaserver before running this script
# Note that an instance of omniNames should be running in background
#

import re, os
from argparse import ArgumentParser
from math import pi, fabs
import hpp
from hpp.corbaserver.manipulation import ConstraintGraphFactory, Rule
from setup import ConstraintGraph, Constraints, grippers, handlesPerObjects, nCylinder, nSphere, objects, ps, robot, shapesPerObject, vf
from hpp.gepetto import PathPlayer
from state_name import StateName
from visibility_prm import VisibilityPRM
import time, sys
from hpp.benchmark import PathChecker


parser = ArgumentParser()
parser.add_argument('-N', default=20, type=int)
parser.add_argument('--display', action='store_true')
parser.add_argument('--run', action='store_true')
args = parser.parse_args()

# Remove joint bound validation
ps.hppcorba.problem.clearConfigValidations()
ps.addConfigValidation("CollisionValidation")

def cleanPaths (ps, solutions) :
  offset = 0
  for i, s in enumerate (solutions):
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
if args.display:
  v = vf.createViewer ()
  pp = PathPlayer (v)
else:
  v = lambda x: None
## Initial configuration of manipulator arms
q0_r0 = [pi/6, -pi/2, pi/2, 0, 0, 0,]
q0_r1 = q0_r0 [::]
## Generate initial configurations of spheres
q0_spheres = list ()
i = 0
y = 0.04
while i < nSphere:
  q0_spheres.append ([- .1*(i/2), -.12 + y, 0.025, 0, 0, 0, 1])
  i+=1; y = -y
## Generate initial configurations of cylinders
q0_cylinders = list ()
i = 0
y = -0.04
while i < nCylinder:
  q0_cylinders.append ([0.45 + .1*(i/2), -.12 + y, 0.025, 0, 0, 0, 1])
  i+=1; y = -y
q0 = q0_r0 + q0_r1 + sum (q0_spheres, []) + sum (q0_cylinders, [])
if args.display:
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
## Add a node to move robots in initial configurations
nodes.append (nodes [-1])
edges.append (getTransitionConnectingStates (graph = cg, s1 = nodes [-2],
                                             s2 = nodes [-1]))
loops.append (getTransitionConnectingStates (graph = cg, s1 = nodes [-1],
                                             s2 = nodes [-1]))
exploreNodes.append (nodes [-1])
def generateSubGoals (q0, edges):
  '''
  Generate a list of subgoals
  Input:
    - q0: initial configuration
    - edges: list of transitions
  Return
    - a list of subgoal configurations
  The first subgoal is the initial configuration q0. The following ones are
  generated by projecting random configurations on the destination node of
  each transition of the list. Each subgoal is reachable from the previous one
  by a path of the transition.
  '''
  subgoals = list ()
  node = 'free'
  res, q_init, err = cg.applyNodeConstraints (node, q0)
  if not res:
    raise RuntimeError ('Failed to project configuration on node ' + node)
  subgoals.append (q_init [::])
  ## Starting from initial configuration, iteratively produce random
  #  configurations on each successive node in such a way that each new
  #  configuration is reachable from the previous one through the transition
  #  linking the states of the configurations.
  q_init = subgoals [0]
  for edge in edges [:-1]:
    edgeSuccess = False
    for i in range (400):
      if i == 0:
        q = q_init
      else:
        q = robot.shootRandomConfig ()
      res, q1, err = cg.generateTargetConfig (edge, q_init, q)
      if not res: continue
      res, msg = robot.isConfigValid (q1)
      if not res: continue
      v (q1)
      ps.addConfigToRoadmap (q1)
      subgoals.append (q1 [::])
      q_init = q1 [::]
      edgeSuccess = True
      break
    if not edgeSuccess:
      raise RuntimeError ('Failed to generate a subgoal through edge ' + edge)
  ## Generate last sub goal configuration to move back manipulator arms in
  #  initial configurations
  q_goal = subgoals [-1] [::]
  q_goal [0:6] = q0_r0; q_goal [6:12] = q0_r1
  subgoals.append (q_goal)
  return subgoals
## Display sequence of states in file
def displayGraph ():
  with open ('/tmp/states.dot', 'w') as f1:
    f1.write ('digraph CD  {\n  rankdir = BT\n  compound=true\n\n')
    for n in nodes:
      f1.write ('  "{0}" [shape = box]\n'.format (n))
    for n0, n1, e in zip (nodes, nodes [1:], edges):
      f1.write ('  "{0}" [shape = oval]\n'.format (e))
      f1.write ('  "{0}" -> "{1}"\n'.format (n0, e))
      f1.write ('  "{0}" -> "{1}"\n'.format (e, n1))
    f1.write ('}')
## Try to connect successive configurations using Visibility PRM planner
def solve ():
  for i, (p0, p1) in enumerate (zip (subgoals, subgoals [1:])):
    planner = VisibilityPRM (cg = cg, ps = ps, q_init = p0, q_goal = p1,
                             state = exploreNodes [i],
                             loopTransition = loops [i], logStream = None)
    planner.solve ()
#ps.addPathOptimizer ('Graph-RandomShortcut')
#ps.addPathOptimizer ('SplineGradientBased_bezier1')
solutions = list ()
# Solve 20 times the problem
import datetime as dt
totalTime = dt.timedelta (0)
totalNumberNodes = 0
for i in range (args.N):
  numberNodes = 0
  # Try 20 times to solve the problem and stop at first success
  t1 = dt.datetime.now ()
  for i in range (20):
    try:
      subgoals = generateSubGoals (q0 = q0, edges = edges)
      ps.clearRoadmap ()
      ps.setInitialConfig (subgoals [0])
      q_goal = subgoals [-1]
      ps.resetGoalConfigs ()
      ps.addGoalConfig (q_goal)
      solve ()
      ps.solve ()
      n = ps.numberNodes ()
      numberNodes += n
      solutions.append (ps.numberPaths () -1)
      break;
    except RuntimeError:
      n = ps.numberNodes ()
      numberNodes += n
  t2 = dt.datetime.now ()
  totalTime += t2 - t1
  print (t2-t1)
  print ("Number nodes: " + str(n))
  totalNumberNodes += numberNodes
if args.N != 0:
  print ("Average time: " + str ((totalTime.seconds+1e-6*totalTime.microseconds)/float (args.N)))
  print ("Average number nodes: " + str (totalNumberNodes/float(args.N)))
  cleanPaths (ps, solutions)

# Create a goal configuration with the construction set assembled.

ps.selectPathPlanner('RMR*')
ps.selectPathValidation('NoValidation', 0)
ps.clearConfigValidations()

cg.initialize()

q_goal = q0_r0 + q0_r1 + [-0.06202136144745322, -0.15, 0.025, 1, 0, 0, 0,
                           0.06202136144745322, -0.15, 0.025, 1, 0, 0, 0,
                           0, -0.15, 0.025, 0, 0, 0, 1,
                          0.5, -0.08, 0.025, 0, 0, 0, 1]
ps.setInitialConfig(q0)
ps.addGoalConfig(q_goal)
ps.setMaxIterPathPlanning(100)
