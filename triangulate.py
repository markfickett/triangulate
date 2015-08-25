# Calculate survey results for a large line segment based on smaller surveyed
# triangle edges.
#
# The input is edge lengths for a triangular lattice. The first and last points
# in the survey data indicate the endpoints of target line segment (property
# boundary) that was not surveyed directly. The output is intercects between
# surveyed line segments and that target. For example, output like BC 51.22
# would mean that 51.22 units along line segment BC intercects the target.

import sys


class MeasuredPoint(object):
  def __init__(self, name):
    self.name = name
    self.edges = []

  def AddEdge(self, neighbor, distance):
    self.edges.append((neighbor, distance))

  def ComputePosition(self):
    raise NotImplementedError()

  def GetIntersections(self, segment_start, segment_end):
    """Generates a list of (neighbor_point, distance) tuples.

    This checks each of its neighbors in turn. If the line segment from this
    point to the neighbor intersects the line segment defined by the arguments,
    it calculates the distance along its segment to the intersection; and if
    that distance is in [0, length of segment to neighbor] it yields the result.
    """
    raise NotImplementedError()


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
      print 'parsed %r => %s' % (line, parsed)
      if parsed is None:
        continue
      name, edge_measurements = parsed
      point = MeasuredPoint(name)
      if first_point is None:
        first_point = point
      else:
        last_point = point
      points[name] = point
      for neighbor_name, distance in edge_measurements:
        neighbor = points[neighbor_name]
        point.AddEdge(neighbor, distance)
      #point.ComputePosition()
  sys.exit(0)
  for point in points.itervalues():
    if point in (first_point, last_point):
      continue
    for neighbor_point, distance in point.GetIntersections(
        first_point, last_point):
      print point.name, neighbor_point.name, distance
