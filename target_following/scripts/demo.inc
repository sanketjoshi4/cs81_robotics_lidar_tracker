define lidar ranger
(
  sensor
  (
    range [0 2]
    fov 360.0
    samples 1442
  )
  # generic model properties
  color "black"
)

define rosbot position
(
  pose [ 0.0 0.0 0.0 0.0 ]
  localization "gps"
  #localization "odom"
  #odom_error [0.01 0.01 999999 999999 999999 0.01]


  size [ 0.2 0.2 0.2 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "diff"
  color "gray"

  lidar(pose [ 0.0 0.0 -0.1 0.0 ])
)

define rosbot2 position
(
  pose [ 0.0 0.0 0.0 0.0 ]
  localization "gps"
  #localization "odom"
  #odom_error [0.01 0.01 999999 999999 999999 0.01]


  size [ 0.2 0.2 0.2 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "diff"
  color "gray"

  lidar(pose [ 0.0 0.0 -0.1 0.0 ])
)

define obs_rosbot position
(
  pose [ 0.0 0.0 0.0 0.0 ]
  localization "gps"
  #localization "odom"
  #odom_error [0.01 0.01 999999 999999 999999 0.01]


  size [ 0.4 0.4 0.4 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "diff"
  color "gray"

  #lidar(pose [ 0.0 0.0 -0.1 0.0 ])
)
