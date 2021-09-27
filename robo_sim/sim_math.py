from ._requirements import *

atan2_range = math.pi  # atan2 range is from [-pi, pi]


def compute_vec2vec_projection(self, u, v):
  """Computes projection of vector {u} onto vector {v}."""
  return (np.dot(u, v) / np.linalg.norm(v)) * v


def compute_vec2plane_projection(self, vec, plane_norm):
  """Computes projection of vector onto plane (defined by a normal vector)."""
  vec, norm = self.convert_lists2np(vec, plane_norm)
  vec2vec_proj = self.compute_vec2vec_projection(vec, norm)
  return vec - vec2vec_proj


def compute_vec2plane_intersection(self, vec, vec_point, plane_norm,
                                   plane_point):
  """Computes intersection of a vector with a plane.
  Args:
    vec: 3D vector
    vec_point: 3D point on vector
    plane_norm: Normal vector of plane
    plane_point: 3D point on plane
  Returns None if parallel or collinear, otherwise point of intersection.
  """
  # Sanitize input and convert everything to np.ndarray
  vec, vec_point, plane_norm, plane_point = self.convert_lists2np(
      vec, vec_point, plane_norm, plane_point)
  if (np.dot(vec, plane_norm) == 0):
    # Vector is perp with plane normal, i.e. parallel with plane (no POI)
    return None
  numerator = np.dot((plane_point - vec_point), plane_norm)
  denom = np.dot(vec, plane_norm)
  d = numerator / denom

  # Return point of intersection
  return vec_point + d * vec


def convert_lists2np(self, *args):
  """Helper function to convert variable number of arguments to np.ndarray.
  Returns: same args, just in numpy format
  """
  converted_args = []
  for arg in args:
    converted_args.append(np.array(arg))
  if len(converted_args) == 1:
    # If single arg, return that element instead of nested list
    return converted_args[0]
  else:
    return converted_args


def normalize_vector(self, vec):
  """Normalizes vector.
  Returns: u/|u|
  """
  vec = np.array(vec)
  return vec / np.linalg.norm(vec)


def compute_vec2vec_angle(self, vec1, vec2):
  """Computes the angle (in radians) between vec1 and vec2."""
  cos_theta = np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2))
  return math.acos(cos_theta)


def compute_vec_angle(self, vec):
  """Projects vector onto ground plane and computes angle from the z axis."""
  # Return modded result so angle is in range [0, math.pi]
  return (math.atan2(vec[0], vec[2])) % atan2_range
