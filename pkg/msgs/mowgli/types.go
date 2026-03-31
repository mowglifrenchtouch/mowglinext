package mowgli

import "github.com/cedbossneo/openmower-gui/pkg/msgs/geometry"

// Status matches mowgli_interfaces/Status
type Status struct {
	Stamp                 geometry.Stamp
	MowerStatus           uint8
	RaspberryPiPower      bool
	IsCharging            bool
	EscPower              bool
	RainDetected          bool
	SoundModuleAvailable  bool
	SoundModuleBusy       bool
	UiBoardAvailable      bool
	MowEnabled            bool
	MowerEscStatus        uint8
	MowerEscTemperature   float32
	MowerEscCurrent       float32
	MowerMotorTemperature float32
	MowerMotorRpm         float32
}

// HighLevelStatus matches mowgli_interfaces/HighLevelStatus
type HighLevelStatus struct {
	State             uint8
	StateName         string
	SubStateName      string
	CurrentArea       int16
	CurrentPath       int16
	CurrentPathIndex  int16
	GpsQualityPercent float32
	BatteryPercent    float32
	IsCharging        bool
	Emergency         bool
}

// Power matches mowgli_interfaces/Power
type Power struct {
	Stamp          geometry.Stamp
	VCharge        float32
	VBattery       float32
	ChargeCurrent  float32
	ChargerEnabled bool
	ChargerStatus  string
}

// Emergency matches mowgli_interfaces/Emergency
type Emergency struct {
	Stamp            geometry.Stamp
	ActiveEmergency  bool
	LatchedEmergency bool
	Reason           string
}

// AbsolutePose matches mowgli_interfaces/AbsolutePose
type AbsolutePose struct {
	Header              geometry.Header
	SensorStamp         uint32
	ReceivedStamp       uint32
	Source              uint8
	Flags               uint16
	OrientationValid    uint8
	MotionVectorValid   uint8
	PositionAccuracy    float32
	OrientationAccuracy float32
	Pose                geometry.PoseWithCovariance
	MotionVector        geometry.Vector3
	VehicleHeading      float64
	MotionHeading       float64
}

// WheelTick matches mowgli_interfaces/WheelTick
type WheelTick struct {
	Stamp            geometry.Stamp
	WheelTickFactor  uint32
	ValidWheels      uint8
	WheelDirectionFl uint8
	WheelTicksFl     uint32
	WheelDirectionFr uint8
	WheelTicksFr     uint32
	WheelDirectionRl uint8
	WheelTicksRl     uint32
	WheelDirectionRr uint8
	WheelTicksRr     uint32
}

// Map is the internal map structure sent to the frontend.
// Matches the old xbot_msgs.Map JSON format.
type Map struct {
	MapWidth        float64
	MapHeight       float64
	MapCenterX      float64
	MapCenterY      float64
	NavigationAreas []MapArea
	WorkingArea     []MapArea
	DockX           float64
	DockY           float64
	DockHeading     float64
}

// MapArea matches the old xbot_msgs.MapArea JSON format.
type MapArea struct {
	Name      string
	Area      geometry.Polygon
	Obstacles []geometry.Polygon
}

// DockingSensor - placeholder, may not exist in ROS2 mowgli
type DockingSensor struct {
	DockPresent  bool
	DockDistance float32
}
