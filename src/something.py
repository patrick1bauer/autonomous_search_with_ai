#parameters
drones = 5 # number of drones
searchArea = [-38, -38, 102, 38] #[startX, startY, endX, endY]
droneIndex = 0 # Drone ID in the swarm, 0-delineated
step = 5 # step size for the search grid

#operations
offset = 1/drones * (searchArea[2] - searchArea[0])
droneSearchArea = [droneIndex * offset + startX, startY, (droneindex + 1) * offset + startX, endY]

XOffset = droneSearchArea[0]
YOffset = droneSearchArea[1]
XBound = droneSearchArea[2]
YBound = droneSearchArea[3]
for x in range(0, XBounds, step)
  for y in range(0, yBounds, step)
    waypointXYZ = [x + XOffset, (y if x%2 == 0 else (YBound - y)) + YOffset,0]
    waypointQuaternion = tf.transformations.quaternion_from_euler(0, 0 if x%2 == 0 else math.pi, 0)