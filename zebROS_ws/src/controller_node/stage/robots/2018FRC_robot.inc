define laser ranger
(
  sensor
  (
    range_max 7.5
    fov 360.0
    samples 360
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]
)

define omnidir_robot position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  odom_error [0.03 0.03 999999 999999 999999 0.02]

  size [ 0.25 0.25 0.40 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  color "red"

  # kinematics
  drive "omni"

  # spawn sensors
  laser(pose [ -0.1 0.0 -0.11 0.0 ])
)
