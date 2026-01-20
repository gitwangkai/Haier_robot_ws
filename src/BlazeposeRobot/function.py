import numpy as np


# Angle Calculation Function
def calculate_angle(a, b, c):
    a = np.array(a)  # First
    b = np.array(b)  # Mid
    c = np.array(c)  # End

    radians = np.arctan2(c[1] - b[1], c[0] - b[0]) - np.arctan2(a[1] - b[1], a[0] - b[0])
    angle = np.abs(radians * 180.0 / np.pi)

    if angle > 180.0:
        angle = 360 - angle

    return angle


def check_element(arr, element):
    """
  Checks if an element exists in an array and returns a boolean.

  Args:
    arr: The array to check.
    element: The element to search for.

  Returns:
    True if the element exists, otherwise False.
  """
    try:
        return element in arr
    except Exception:
        return False

def estimate_distance(bbox):
    avg_person_height = 1.7  
    focal_length_pixel = 1000  
    bbox_height = bbox[3] - bbox[1]  
    distance = (avg_person_height * focal_length_pixel) / (bbox_height * 2)
    return distance
