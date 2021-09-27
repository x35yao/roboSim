from ._requirements import *
from PIL import Image
from pathlib import Path


def capture_image(self) -> None:
  """Captures and saves image of scene from internal simulation camera."""
  self.camera_state = self.bt.getDebugVisualizerCamera()
  width, height, view_matrix, proj_matrix, *rest = self.camera_state
  self.cur_image = self.bt.getCameraImage(width=width,
                                          height=height,
                                          viewMatrix=view_matrix,
                                          projectionMatrix=proj_matrix)
  self._save_image(self.cur_image)


def _save_image(self, img_raw) -> None:
  """Saves image to local storage. 
  Saved image name is suffixed with unique, monotonic ID.
  Args:
    img_raw: PyBullet image obtained from getCameraImage() function.
  """
  width, height, rgba, *rest = img_raw

  # Convert pyBullet image to PIL format
  rgba = bytes(rgba)
  img = Image.frombytes('RGBA', (width, height), rgba)

  self._verify_dir_exists(self.out_data_dir)

  # Save image as PNG file at original resolution
  filename = self.out_data_dir + "/capture_" + str(self.saved_img_id) + ".png"
  print("Saving image: ", filename)
  img.save(filename)

  # Increment unique ID
  self.saved_img_id += 1


def _verify_dir_exists(self, dir_path: str) -> None:
  """Verifies that the directory exists. If not, recursively creates directory.
  Args:
    dir_path: Path to directory to be created
  """
  if not self.out_data_dir_created:
    try:
      Path(dir_path).mkdir(parents=True, exist_ok=False)
    except:
      raise
    print("Creating directory: ", dir_path)
    self.out_data_dir_created = True