package nad_nav

import "fmt"

// AnchorObservation is a single camera-derived reading in normalized image coordinates.
//
// Conventions:
//   - cx, cy in [-1, +1] with (0, 0) at the image center.
//   - size in [0, 1] where larger implies closer.
type AnchorObservation struct {
	T          float64
	Detected   bool
	Confidence float64
	CX         float64
	CY         float64
	Size       float64
}

// AnchorState is the filtered observation used by the controller.
//
// It adds smoothed values, velocities, and target age for dropout handling.
type AnchorState struct {
	T          float64
	Valid      bool
	Confidence float64
	CX         float64
	CY         float64
	VX         float64
	VY         float64
	Size       float64
	VSize      float64
	Age        float64
}

// Mode selects which controller policy produces outputs.
type Mode int

const (
	ModeSearch Mode = iota + 1
	ModeTrack
	ModeApproach
	ModeCapture
	ModeFlyStraight
	ModeLateralOnly
)

func (m Mode) String() string {
	switch m {
	case ModeSearch:
		return "SEARCH"
	case ModeTrack:
		return "TRACK"
	case ModeApproach:
		return "APPROACH"
	case ModeCapture:
		return "CAPTURE"
	case ModeFlyStraight:
		return "FLY_STRAIGHT"
	case ModeLateralOnly:
		return "LATERAL_ONLY"
	default:
		return fmt.Sprintf("Mode(%d)", int(m))
	}
}

// BodyCommand is the abstract controller output sent to downstream actuators.
type BodyCommand struct {
	T        float64
	Mode     Mode
	Yaw      float64 // [-1, 1]
	Vertical float64 // [-1, 1]
	Forward  float64 // [0, 1]
}
