define topurg ranger
(
  sensor
  (
    range_max 6.0
    fov 180.0
    samples 683
  )
  # generic model properties
  color "black"
  size [ 0.05 0.05 0.1 ]
)

define korus position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  odom_error [0.03 0.03 999999 999999 999999 0.03]

  size [ 0.55 0.55 0.325 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "diff"
  color "gray"

  topurg(pose [ 0.25 0.0 0.325 0.0 ])
)
