# In this file we manage the scanning of the scene, this works by getting the 
# markers dictionary from aruco_interface (or others in the future) and storing
# the relations between the visible markers over the scanning time.
# We record a certain number of iterations for each relation between the markers
# and produce a list containing all the relations with all their iterations. 
# When the scanning is done, the list is simplified to contain the averages of the iterations.
# The list is then stored. Here we note that the list does not have a unified reference.
# That is to be selected by the user in IndPose.py