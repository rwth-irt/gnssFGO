
# Original transformations
Currently, z-aix of imu and lidar towards top

## IMUToLiDAR:
[0., -1., 0., 0., \
1., 0., 0., 0., \
0., 0., 1., 7.62e-02]

## UbloxToLiDAR:
[4.318e-01, 5.08e-01, 5.08e-01]

## IMUToUblox
[-5.08e-01, 4.318e-01, -5.08e-01]

## SPANToLiDAR:
The x-aix  of SPAN directs to backwards of the driving direction, z-aix down to ground
[0., -1., 0., -5.245e-01,\
-1., 0., 0., 1.06045,\
0., 0., -1., 7.98576e-01]

## IMUToSPAN:
[1., 0., 0., 5.245e-01,\
0., -1., 0., 1.06045,\
0., 0., -1., 8.748e-01]

## IMUToSPANAnt:
[0, -4.572e-01, -0.3048]


# OnlineFGO transformations
In OnlineFGO, the z-aix of the imu towards down, the x-aix towards the driving directions
## IMUToLiDAR:
[0., 1., 0., 0.,\
1., 0., 0., 0.,\
0., 0., -1., -7.62e-02]

[1., 0., 0., 0.,\
0., -1., 0., 0.,\
0., 0., -1., -7.62e-02]

## IMUToUblox
[-5.08e-01, -4.318e-01, 5.08e-01]

## IMUToSPAN
[-1., 0., 0.,  1.0604\
  0., -1. 0.,  -0.5245\
  0., 0., 1.0, 0.8748]

## IMUToSPANAnt:
[0, 4.572e-01, 0.3048]















