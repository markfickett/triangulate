# Calculate survey results for a large line segment based on smaller surveyed
# triangle edges.
#
# The input is edge lengths for a triangular lattice. The first and last points
# in the survey data indicate the endpoints of target line segment (property
# boundary) that was not surveyed directly. The output is intercects between
# surveyed line segments and that target. For example, output like BC 51.22
# would mean that 51.22 units along line segment BC intercects the target.

import math
import sys


class MeasuredPoint(object):
  def __init__(self, name):
    self.name = name
    self.neighbors_and_distances = []
    self.x = None
    self.y = None

  def AddEdge(self, neighbor, distance):
    if self.name == neighbor.name:
      raise ValueError('Cannot add %r as neighbor of itself.' % self.name)
    self.neighbors_and_distances.append((neighbor, distance))

  def ComputePosition(self):
    for neighbor, _ in self.neighbors_and_distances:
      if neighbor.x is None or neighbor.y is None:
        raise RuntimeError(
            'Cannot calculate position of %r, neighbor %r has no position.'
            % (self.name, neighbor.name))
    n = len(self.neighbors_and_distances)
    if n == 0:
      self.x, self.y = (0.0, 0.0)
      return
    if n == 1:
      origin, d = self.neighbors_and_distances[0]
      theta = math.pi * 3.0 / 4.0  # arbitrary direction
      self.x = origin.x + d * math.cos(theta)
      self.y = origin.y + d * math.sin(theta)
      return
    if n != 2:
      raise RuntimeError('Two neighbors required for nontrivial cases.')

    x0, y0 = GetThirdTrianglePoint(*self.neighbors_and_distances)
    x1, y1 = GetThirdTrianglePoint(*self.neighbors_and_distances, opposite=True)
    if y0 > y1:
      self.x, self.y = x0, y0
    else:
      self.x, self.y = x1, y1

  def GetIntersections(self, segment_start, segment_end):
    """Generates a list of (neighbor_point, distance) tuples.

    This checks each of its neighbors in turn. If the line segment from this
    point to the neighbor intersects the line segment defined by the arguments,
    it calculates the distance along its segment to the intersection; and if
    that distance is in [0, length of segment to neighbor] it yields the result.
    """
    raise NotImplementedError()


def GetThirdTrianglePoint((point_a, b), (point_b, a), opposite=False):
  # Givens: A triangle ABC where we are computing the point C, and the edge
  # lengths are named for their opposing points.
  ax, ay = point_a.x, point_a.y
  bx, by = point_b.x, point_b.y
  c = math.sqrt((ax - bx)**2 + (ay - by)**2)
  print 'ax, ay =', ax, ',', ay
  print 'b =', b
  print 'bx, by =', bx, ',', by
  print 'a =', a
  print 'c =', c

  # Calculate the angle CAB using law of consines.
  alpha = math.acos((b**2 + c**2 - a**2) / (2 * b * c))
  print 'alpha =', alpha

  # Calculate the angle between AB and the x axis.
  sub_alpha = math.acos((bx - ax) / c)
  if by < ay:  # third and fourth quadrant
    sub_alpha = 2 * math.pi - sub_alpha
  print 'sub_alpha =', sub_alpha

  # The sum alpha + sub_alpha is the angle from the x axis to AC. Use the
  # definition of cosine to find C.
  full_alpha = sub_alpha - alpha if opposite else sub_alpha + alpha
  cx = ax + b * math.cos(full_alpha)
  cy = ay + b * math.sin(full_alpha)
  print 'cx, cy =', cx, ',', cy

  return cx, cy


def ParseLine(raw_line):
  """Returns a point name and a list of (neighbor name, distance) tuples."""
  line_data = raw_line.split('#')[0]
  if not line_data:
    return None
  data = line_data.strip().split()
  name = data[0]
  edges = []
  data = data[1:]
  while data:
    edges.append((data[0], float(data[1])))
    data = data[2:]
  return name, edges


if __name__ == '__main__':
  if len(sys.argv) != 2:
    print 'Usage: %s <edge file name>' % sys.argv[0]
    sys.exit(1)
  edges_path = sys.argv[1]
  points = {}
  first_point = None
  last_point = None
  with open(edges_path) as edges_file:
    for line in edges_file:
      parsed = ParseLine(line)
      if parsed is None:
        continue
      print 'parsed %r => %s' % (line, parsed)
      name, edge_measurements = parsed
      point = MeasuredPoint(name)
      if first_point is None:
        first_point = point
      else:
        last_point = point
      if point.name in points:
        raise ValueError('Point %r redeclared.' % point.name)
      points[name] = point
      for neighbor_name, distance in edge_measurements:
        neighbor = points[neighbor_name]
        point.AddEdge(neighbor, distance)
      point.ComputePosition()
      print '%r\t%f\t%f' % (point.name, point.x, point.y)
  sys.exit(0)
  for point in points.itervalues():
    if point in (first_point, last_point):
      continue
    for neighbor_point, distance in point.GetIntersections(
        first_point, last_point):
      print point.name, neighbor_point.name, distance
