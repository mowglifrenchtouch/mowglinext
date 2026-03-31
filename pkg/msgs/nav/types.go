package nav

import "github.com/cedbossneo/openmower-gui/pkg/msgs/geometry"

type Path struct {
	Header geometry.Header
	Poses  []geometry.PoseStamped
}

type Odometry struct {
	Header       geometry.Header
	ChildFrameId string
	Pose         geometry.PoseWithCovariance
	Twist        geometry.TwistWithCovariance
}
