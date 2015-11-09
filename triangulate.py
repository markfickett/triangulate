#!/usr/bin/env python
# Calculate survey results for a large line segment based on smaller surveyed
# triangle edges.
#
# The input is edge lengths for a triangular lattice. The first and last points
# in the survey data indicate the endpoints of target line segment (ex: a
# property boundary) that was not surveyed directly. The output is intercects
# between surveyed line segments and that target. For example, output like
# BC 51.22 would mean that 51.22 units along line segment BC intercects the
# target.

# for DrawLattice
try:
  import PIL
  import PIL.Image
  import PIL.ImageDraw
except ImportError, e:
  print 'Could not import PIL:', e
  PIL = None

import math
import os
import random
import sys

INTRODUCE_MEASUREMENT_ERROR = True
MEASUREMENT_ERROR = 6.0 / (100.0 * 12.0)  # 6 inches in 100 feet
EPSILON = 1e-5


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
    """Generates (neighbor_point, intersection_point, distance) triples.

    This checks each of its neighbors in turn. If the line segment from this
    point to the neighbor intersects the line segment defined by the arguments,
    it calculates the distance along its segment to the intersection; and if
    that distance is in [0, length of segment to neighbor] it yields the result.
    """
    for neighbor, neighbor_distance in self.neighbors_and_distances:
      #print '? %r %r' % (self.name, neighbor.name)
      intersection, distance = GetIntersectionAndDistance(
          (self.x, self.y),
          (neighbor.x, neighbor.y),
          neighbor_distance,
          (segment_start.x, segment_start.y),
          (segment_end.x, segment_end.y))
      if intersection is not None:
        yield neighbor, intersection, distance


def GetThirdTrianglePoint((point_a, b), (point_b, a), opposite=False):
  # Givens: A triangle ABC where we are computing the point C, and the edge
  # lengths are named for their opposing points.
  ax, ay = point_a.x, point_a.y
  bx, by = point_b.x, point_b.y
  c = math.sqrt((ax - bx)**2 + (ay - by)**2)
  #print 'ax, ay =', ax, ',', ay
  #print 'b =', b
  #print 'bx, by =', bx, ',', by
  #print 'a =', a
  #print 'c =', c

  # Calculate the angle CAB using law of consines.
  alpha = math.acos((b**2 + c**2 - a**2) / (2 * b * c))
  #print 'alpha =', alpha

  # Calculate the angle between AB and the x axis.
  sub_alpha = math.acos((bx - ax) / c)
  if by < ay:  # third and fourth quadrant
    sub_alpha = 2 * math.pi - sub_alpha
  #print 'sub_alpha =', sub_alpha

  # The sum alpha + sub_alpha is the angle from the x axis to AC. Use the
  # definition of cosine to find C.
  full_alpha = sub_alpha - alpha if opposite else sub_alpha + alpha
  cx = ax + b * math.cos(full_alpha)
  cy = ay + b * math.sin(full_alpha)
  #print 'cx, cy =', cx, ',', cy

  return cx, cy


def GetIntersectionAndDistance((x1, y1), (x2, y2), l, (x3, y3), (x4, y4)):
  # https://en.wikipedia.org/wiki/
  #     Line%E2%80%93line_intersection#Given_two_points_on_each_line
  #print 'x1, y1 = ', x1, ',', y1
  #print 'x2, y2 = ', x2, ',', y2
  #print 'x3, y3 = ', x3, ',', y3
  #print 'x4, y4 = ', x4, ',', y4
  denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
  #print 'denom =', denom
  if abs(denom) < EPSILON:  # parallel
    #print 'parallel'
    return None, None
  x_num = (x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)
  y_num = (x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)

  x, y = x_num / denom, y_num / denom
  #print 'x, y =', x, ',', y
  d = math.sqrt((y - y1)**2 + (x - x1)**2)
  #print 'd =', d
  if d + EPSILON > l or d < EPSILON:  # past the end of the segment
    #print 'past end'
    return None, None
  if not ((sgn(x - x1) == sgn(x2 - x1)) and (sgn(y - y1) == sgn(y2 - y1))):
    #print 'before end'
    return None, None
  return (x, y), d


def sgn(x):
  if abs(x) < EPSILON:
    return 0
  if x < 0:
    return -1
  return 1


def ParseDistance(raw):
  if "'" in raw:
    feet, inches = raw.split("'")
    if '"' not in inches:
      raise ValueError('Bad feet-and-inches: %r' % raw)
    return 12 * int(feet) + float(inches.strip('"'))
  else:
    return float(raw)


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
    distance = ParseDistance(data[1])
    if INTRODUCE_MEASUREMENT_ERROR:
      distance += random.uniform(-MEASUREMENT_ERROR, MEASUREMENT_ERROR)
    edges.append((data[0], distance))
    data = data[2:]
  return name, edges


def FormatFeetAndInches(inches_value):
  feet = int(inches_value) / 12
  inches = inches_value - (feet * 12)
  return '%d\'%.1f"' % (feet, inches)


def DrawLattice(ordered_points):
  if PIL is None:
    print 'PIL unavailable, not drawing lattice.'
    return
  min_x = min_y = float('Inf')
  max_x = max_y = float('-Inf')
  for point in ordered_points:
    min_x = min(min_x, point.x)
    max_x = max(max_x, point.x)
    min_y = min(min_y, point.y)
    max_y = max(max_y, point.y)

  offset = 50
  offset_x = offset + (0 if min_x > 0 else -min_x)
  offset_y = offset + (0 if min_y > 0 else -min_y)
  image = PIL.Image.new(
      'RGBA',
      (int(max_x - min_x + 2 * offset), int(max_y - min_y + 2 * offset)),
      (255, 255, 255, 0))
  draw = PIL.ImageDraw.Draw(image)

  def TransformPoint(in_x, in_y):
    return int(in_x + offset_x), int(max_y - in_y + offset_y)

  for point in ordered_points:
    x, y = TransformPoint(point.x, point.y)
    for neighbor, _ in point.neighbors_and_distances:
      nx, ny = TransformPoint(neighbor.x, neighbor.y)
      draw.line((x, y, nx, ny), 'red')
    draw.text((x, y), point.name, 'black')

  first = ordered_points[0]
  last = ordered_points[-1]
  fx, fy = TransformPoint(first.x, first.y)
  lx, ly = TransformPoint(last.x, last.y)
  draw.line((fx, fy, lx, ly), 'black')

  image.show()
  image.save(os.path.expanduser('~/Desktop/lattice.png'))


if __name__ == '__main__':
  if len(sys.argv) != 2:
    print 'Usage: %s <edge file name>' % sys.argv[0]
    sys.exit(1)
  edges_path = sys.argv[1]

  # Load the lattice.
  points = {}
  ordered_points = []
  with open(edges_path) as edges_file:
    print '%s\t%s' % ('point', 'location')
    for line in edges_file:
      parsed = ParseLine(line)
      if parsed is None:
        continue
      name, edge_measurements = parsed
      point = MeasuredPoint(name)
      if point.name in points:
        raise ValueError('Point %r redeclared.' % point.name)
      points[name] = point
      ordered_points.append(point)
      for neighbor_name, distance in edge_measurements:
        neighbor = points[neighbor_name]
        point.AddEdge(neighbor, distance)

  for point in ordered_points:
    point.ComputePosition()
  first, last = ordered_points[0], ordered_points[-1]

  for point in ordered_points:
    print '%s\t%f, %f' % (point.name, point.x, point.y)

  # Calculate intersections.
  print '%s\t%s\t%s' % ('segment', 'intersection location', 'distance')
  for point in ordered_points[2:-1]:
    for neighbor_point, (x, y), distance in point.GetIntersections(first, last):
      print '%s %s\t%s\t%f, %f' % (
          point.name, neighbor_point.name, FormatFeetAndInches(distance), x, y)

  DrawLattice(ordered_points)
