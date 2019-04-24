#
#  Copyright (2017) CNRS
#
#  Author: Florent Lamiraux
#

class StateName (object):
  """
  Handle permutations in state names

  State names are built according to the following pattern:
    'gripper name grasps handle name' separated by ':'
  A given state may therefore have a name difficult to predict since the order
  of the above sentences may vary.

  This class handles the variation.
  """
  noGrasp = 'free'
  def __init__ (self, grasps) :
    """
    Node names is defined by a set of pairs (gripper, handle)
    """
    if isinstance (grasps, set):
      self.grasps = grasps.copy ()
    elif isinstance (grasps, str):
      if grasps == self.noGrasp:
        self.grasps = set ()
      else:
        g1 = map (lambda s: s.strip (' '), grasps.split (':'))
        self.grasps = set (map (lambda s: tuple (s.split (' grasps ')), g1))
    else:
      raise TypeError ('expecting a set of pairs (gripper, handle) or a string')

  def __str__ (self) :
    if self.grasps == set ():
      return 'free'
    res = ""
    for g in self.grasps:
      res += g [0] + " grasps " + g [1] + " : "
    return res [:-3]

  def __eq__ (self, other):
    return self.grasps == other.grasps

  def __ne__ (self, other):
    return not self.__eq__ (other)
