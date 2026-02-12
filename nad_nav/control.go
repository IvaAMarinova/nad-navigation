package nad_nav

import "math"

// ModeCommandConfig defines constant offsets applied per mode.
type ModeCommandConfig struct {
	Yaw      float64 `json:"yaw"`
	Vertical float64 `json:"vertical"`
	Forward  float64 `json:"forward"`
}

// ControllerConfig bundles controller gains and mode policy.
type ControllerConfig struct {
	ConfMin float64 `json:"conf_min"`

	BaseSearch   ModeCommandConfig `json:"base_search"`
	BaseTrack    ModeCommandConfig `json:"base_track"`
	BaseApproach ModeCommandConfig `json:"base_approach"`
	BaseCapture  ModeCommandConfig `json:"base_capture"`

	XTol               float64 `json:"x_tol"`
	YTol               float64 `json:"y_tol"`
	CenteredHoldFrames int     `json:"centered_hold_frames"`

	SizeCapture float64 `json:"size_capture"`

	KpX float64 `json:"kp_x"`
	KdX float64 `json:"kd_x"`
	KpY float64 `json:"kp_y"`
	KdY float64 `json:"kd_y"`

	BaseForward float64 `json:"base_forward"`
	ForwardMin  float64 `json:"forward_min"`
	MaxForward  float64 `json:"max_forward"`
	XGate       float64 `json:"x_gate"`
	YGate       float64 `json:"y_gate"`

	TLead float64 `json:"t_lead"`

	AllowedModes        []Mode  `json:"allowed_modes"`
	DefaultMode         Mode    `json:"default_mode"`
	ModeOverride        *Mode   `json:"mode_override"`
	FlyStraightSeconds  float64 `json:"fly_straight_seconds"`
	FlyStraightForward  float64 `json:"fly_straight_forward"`
	FlyStraightYaw      float64 `json:"fly_straight_yaw"`
	FlyStraightVertical float64 `json:"fly_straight_vertical"`
	FlyStraightAfter    Mode    `json:"fly_straight_after_mode"`
}

// DroneController implements the state machine and PD control.
type DroneController struct {
	Cfg           ControllerConfig
	mode          Mode
	centeredCount int
	searchPhase   float64
	flyStartT     *float64
	lateralStartT *float64
	lastCmd       BodyCommand
	hasLastCmd    bool
}

// NewDroneController constructs a controller with the given configuration.
func NewDroneController(cfg ControllerConfig) *DroneController {
	return &DroneController{Cfg: cfg, mode: cfg.DefaultMode}
}

// Step computes the next command for the current time step.
func (dc *DroneController) Step(st AnchorState, dt float64) BodyCommand {
	cmd := dc.step(st, dt)
	dc.lastCmd = cmd
	dc.hasLastCmd = true
	return cmd
}

func (dc *DroneController) step(st AnchorState, dt float64) BodyCommand {
	if dc.Cfg.ModeOverride != nil {
		return dc.stepOverride(st, dt)
	}

	desired := dc.selectMode(st)
	actual := dc.applyModePolicy(desired)
	if actual != dc.mode {
		if actual != ModeFlyStraight {
			dc.flyStartT = nil
		}
		if actual != ModeLateralOnly {
			dc.lateralStartT = nil
		}
	}
	dc.mode = actual
	return dc.commandForMode(actual, st, dt)
}

// stepOverride forces a specific mode when configured.
func (dc *DroneController) stepOverride(st AnchorState, dt float64) BodyCommand {
	override := *dc.Cfg.ModeOverride
	if override == ModeFlyStraight {
		if dc.flyStartT == nil {
			t := st.T
			dc.flyStartT = &t
		}
		elapsed := st.T - *dc.flyStartT
		if elapsed <= dc.Cfg.FlyStraightSeconds {
			return dc.commandFlyStraight(st)
		}
		actual := dc.applyModePolicy(dc.Cfg.FlyStraightAfter)
		dc.mode = actual
		return dc.commandForMode(actual, st, dt)
	}
	actual := dc.applyModePolicy(override)
	dc.mode = actual
	return dc.commandForMode(actual, st, dt)
}

// selectMode chooses a desired mode based on tracking state.
func (dc *DroneController) selectMode(st AnchorState) Mode {
	recentlySeen := st.Age <= 0.35
	if !st.Valid && recentlySeen {
		return ModeTrack
	}

	if !st.Valid {
		dc.centeredCount = 0
		return ModeSearch
	}

	centered := math.Abs(st.CX) < dc.Cfg.XTol && math.Abs(st.CY) < dc.Cfg.YTol
	if centered {
		dc.centeredCount++
	} else {
		dc.centeredCount = 0
	}

	if st.Size >= dc.Cfg.SizeCapture && dc.centeredCount >= dc.Cfg.CenteredHoldFrames {
		return ModeCapture
	}
	if dc.centeredCount >= dc.Cfg.CenteredHoldFrames {
		return ModeApproach
	}
	return ModeTrack
}

// applyModePolicy clamps the desired mode to the allowed set.
func (dc *DroneController) applyModePolicy(desired Mode) Mode {
	if len(dc.Cfg.AllowedModes) == 0 {
		return desired
	}
	for _, allowed := range dc.Cfg.AllowedModes {
		if allowed == desired {
			return desired
		}
	}
	for _, allowed := range dc.Cfg.AllowedModes {
		if allowed == dc.Cfg.DefaultMode {
			return dc.Cfg.DefaultMode
		}
	}
	return dc.Cfg.AllowedModes[0]
}

// commandForMode computes the command for a specific mode.
func (dc *DroneController) commandForMode(mode Mode, st AnchorState, dt float64) BodyCommand {
	switch mode {
	case ModeStop:
		return dc.commandStop(st)
	case ModeFlyStraight:
		return dc.commandFlyStraight(st)
	case ModeLateralOnly:
		return dc.commandLateralOnly(st)
	case ModeSearch:
		dc.searchPhase += dt
		yaw := dc.Cfg.BaseSearch.Yaw + 0.35*math.Sin(0.7*dc.searchPhase)
		return BodyCommand{T: st.T, Mode: mode, Yaw: yaw, Vertical: dc.Cfg.BaseSearch.Vertical, Forward: dc.Cfg.BaseSearch.Forward}
	case ModeTrack:
		return dc.commandTrackLike(mode, dc.Cfg.BaseTrack, st)
	case ModeApproach:
		return dc.commandTrackLike(mode, dc.Cfg.BaseApproach, st)
	default:
		return BodyCommand{T: st.T, Mode: mode, Yaw: dc.Cfg.BaseCapture.Yaw, Vertical: dc.Cfg.BaseCapture.Vertical, Forward: dc.Cfg.BaseCapture.Forward}
	}
}

// commandTrackLike computes PD steering for TRACK/APPROACH behaviors.
func (dc *DroneController) commandTrackLike(mode Mode, base ModeCommandConfig, st AnchorState) BodyCommand {
	cx := st.CX + st.VX*dc.Cfg.TLead
	cy := st.CY + st.VY*dc.Cfg.TLead
	ex := -cx
	ey := -cy

	yaw := base.Yaw + dc.Cfg.KpX*ex + dc.Cfg.KdX*(-st.VX)
	vertical := base.Vertical + dc.Cfg.KpY*ey + dc.Cfg.KdY*(-st.VY)

	yaw = clamp(yaw, -1, 1)
	vertical = clamp(vertical, -1, 1)

	centeredNow := math.Abs(st.CX) < dc.Cfg.XTol && math.Abs(st.CY) < dc.Cfg.YTol
	var forward float64
	if centeredNow {
		forward = dc.Cfg.MaxForward
	} else {
		gate := math.Max(0, 1-math.Abs(st.CX)/dc.Cfg.XGate) * math.Max(0, 1-math.Abs(st.CY)/dc.Cfg.YGate)
		if mode == ModeTrack {
			forward = 0.10 * gate
		} else {
			forward = math.Min(dc.Cfg.MaxForward, dc.Cfg.BaseForward*gate+0.25*gate)
		}
	}

	forward = clamp(math.Max(base.Forward, forward), 0, 1)
	forward = math.Max(forward, dc.Cfg.ForwardMin)

	return BodyCommand{T: st.T, Mode: mode, Yaw: yaw, Vertical: vertical, Forward: forward}
}

// commandLateralOnly outputs yaw corrections without forward movement.
func (dc *DroneController) commandLateralOnly(st AnchorState) BodyCommand {
	if dc.lateralStartT == nil {
		t := st.T
		dc.lateralStartT = &t
	}
	cx := st.CX + st.VX*dc.Cfg.TLead
	ex := -cx
	yaw := clamp(dc.Cfg.KpX*ex+dc.Cfg.KdX*(-st.VX), -1, 1)
	forward := 0.0
	modeOut := ModeStop
	if dc.Cfg.FlyStraightSeconds > 0 && dc.lateralStartT != nil {
		elapsed := st.T - *dc.lateralStartT
		half := dc.Cfg.FlyStraightSeconds / 2
		if half > 0 && elapsed <= dc.Cfg.FlyStraightSeconds {
			var scale float64
			if elapsed <= half {
				scale = elapsed / half
			} else {
				scale = (dc.Cfg.FlyStraightSeconds - elapsed) / half
			}
			scale = clamp(scale, 0, 1)
			forward = dc.Cfg.FlyStraightForward * scale
			modeOut = ModeFlyStraight
		}
	}
	return BodyCommand{T: st.T, Mode: modeOut, Yaw: yaw, Vertical: 0, Forward: forward}
}

// commandFlyStraight outputs a constant forward command for testing.
func (dc *DroneController) commandFlyStraight(st AnchorState) BodyCommand {
	forward := dc.Cfg.FlyStraightForward
	if dc.flyStartT != nil && dc.Cfg.FlyStraightSeconds > 0 {
		elapsed := st.T - *dc.flyStartT
		half := dc.Cfg.FlyStraightSeconds / 2
		if half > 0 {
			var scale float64
			if elapsed <= half {
				scale = elapsed / half
			} else {
				scale = (dc.Cfg.FlyStraightSeconds - elapsed) / half
			}
			scale = clamp(scale, 0, 1)
			forward = dc.Cfg.FlyStraightForward * scale
		}
	}
	return BodyCommand{
		T:        st.T,
		Mode:     ModeFlyStraight,
		Yaw:      dc.Cfg.FlyStraightYaw,
		Vertical: dc.Cfg.FlyStraightVertical,
		Forward:  forward,
	}
}

func (dc *DroneController) commandStop(st AnchorState) BodyCommand {
	if !dc.hasLastCmd {
		return BodyCommand{T: st.T, Mode: ModeStop, Yaw: 0, Vertical: 0, Forward: 0}
	}
	last := dc.lastCmd
	return BodyCommand{T: st.T, Mode: ModeStop, Yaw: last.Yaw, Vertical: last.Vertical, Forward: last.Forward}
}

// clamp keeps value inside [lo, hi].
func clamp(value, lo, hi float64) float64 {
	if value < lo {
		return lo
	}
	if value > hi {
		return hi
	}
	return value
}
