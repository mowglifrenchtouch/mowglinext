package sensor

import "github.com/cedbossneo/openmower-gui/pkg/msgs/geometry"

type Imu struct {
	Header                       geometry.Header
	Orientation                  geometry.Quaternion
	OrientationCovariance        [9]float64
	AngularVelocity              geometry.Vector3
	AngularVelocityCovariance    [9]float64
	LinearAcceleration           geometry.Vector3
	LinearAccelerationCovariance [9]float64
}

type NavSatFix struct {
	Header                 geometry.Header
	Status                 NavSatStatus
	Latitude               float64
	Longitude              float64
	Altitude               float64
	PositionCovariance     [9]float64
	PositionCovarianceType uint8
}

type NavSatStatus struct {
	Status  int8
	Service uint16
}

type LaserScan struct {
	Header         geometry.Header
	AngleMin       float32
	AngleMax       float32
	AngleIncrement float32
	TimeIncrement  float32
	ScanTime       float32
	RangeMin       float32
	RangeMax       float32
	Ranges         []float32
	Intensities    []float32
}
