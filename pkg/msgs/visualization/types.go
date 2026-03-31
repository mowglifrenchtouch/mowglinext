package visualization

import "github.com/cedbossneo/openmower-gui/pkg/msgs/geometry"

type Marker struct {
	Header   geometry.Header
	Ns       string
	Id       int32
	Type     int32
	Action   int32
	Pose     geometry.Pose
	Scale    geometry.Vector3
	Color    ColorRGBA
	Lifetime Duration
	Points   []geometry.Point
}

type ColorRGBA struct {
	R float32
	G float32
	B float32
	A float32
}

type Duration struct {
	Sec     int32
	Nanosec uint32
}

type MarkerArray struct {
	Markers []Marker
}
