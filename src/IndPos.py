# This file contains the class with which the user interacts.
# - It loads the scan file specified by the user, parse it.
# - Allow the user to define the reference frame of his scene.
#     make a new list from the original for which we calculate all the coords
#     in the unified reference frame defined by the user.
# - Now we accept a dictionary of markers from the user (comes from aruco_interface.py)
#     Based on those values we calculate the position of the camera with reference to the user
#     reference frame
# - Return position and orientation