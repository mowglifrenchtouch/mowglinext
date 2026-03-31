package geometry

type Point struct {
	X float64
	Y float64
	Z float64
}

type Point32 struct {
	X float32
	Y float32
	Z float32
}

type Vector3 struct {
	X float64
	Y float64
	Z float64
}

type Quaternion struct {
	X float64
	Y float64
	Z float64
	W float64
}

type Pose struct {
	Position    Point
	Orientation Quaternion
}

type PoseWithCovariance struct {
	Pose       Pose
	Covariance [36]float64
}

type PoseStamped struct {
	Header Header
	Pose   Pose
}

type Polygon struct {
	Points []Point32
}

type Twist struct {
	Linear  Vector3
	Angular Vector3
}

type TwistWithCovariance struct {
	Twist      Twist
	Covariance [36]float64
}

// Header is std_msgs/Header
type Header struct {
	Stamp   Stamp
	FrameId string
}

type Stamp struct {
	Sec     uint32
	Nanosec uint32
}
