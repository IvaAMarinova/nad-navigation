package nad_nav

import "math"

// TrackerConfig controls smoothing and dropout handling for observations.
type TrackerConfig struct {
	Alpha            float64 `json:"alpha"`
	HoldSeconds      float64 `json:"hold_seconds"`
	Decay            float64 `json:"decay"`
	ReacquireConfMin float64 `json:"reacquire_conf_min"`
}

// AnchorTracker tracks the anchor position with smoothing and velocity estimates.
type AnchorTracker struct {
	cfg TrackerConfig

	lastT       *float64
	cx, cy      float64
	size        float64
	vx, vy      float64
	vsize       float64
	lastValidT  *float64
	lastRawCX   *float64
	lastRawCY   *float64
	lastRawSize *float64
}

// NewAnchorTracker constructs a new tracker with the provided configuration.
func NewAnchorTracker(cfg TrackerConfig) *AnchorTracker {
	return &AnchorTracker{cfg: cfg}
}

// Update ingests the latest observation and returns a filtered AnchorState.
func (tr *AnchorTracker) Update(obs AnchorObservation, confMin float64) AnchorState {
	t := obs.T

	if tr.lastT == nil {
		tr.lastT = &t
		if obs.Detected && obs.Confidence >= confMin {
			tr.lastValidT = &t
		}
	}

	lastT := t
	if tr.lastT != nil {
		lastT = *tr.lastT
	}
	dt := math.Max(1e-3, t-lastT)
	*tr.lastT = t

	minConf := confMin
	if tr.lastValidT == nil || t-*tr.lastValidT > tr.cfg.HoldSeconds {
		if tr.cfg.ReacquireConfMin > 0 {
			minConf = tr.cfg.ReacquireConfMin
		}
	}

	good := obs.Detected && obs.Confidence >= minConf

	var valid bool
	var age float64

	if good {
		if tr.lastRawCX != nil {
			tr.vx = (obs.CX - *tr.lastRawCX) / dt
			tr.vy = (obs.CY - *tr.lastRawCY) / dt
			tr.vsize = (obs.Size - *tr.lastRawSize) / dt
		}

		tr.lastRawCX = &obs.CX
		tr.lastRawCY = &obs.CY
		tr.lastRawSize = &obs.Size

		a := tr.cfg.Alpha
		tr.cx = a*tr.cx + (1-a)*obs.CX
		tr.cy = a*tr.cy + (1-a)*obs.CY
		tr.size = a*tr.size + (1-a)*obs.Size

		tr.lastValidT = &t
		valid = true
		age = 0
	} else {
		if tr.lastValidT != nil {
			age = t - *tr.lastValidT
		} else {
			age = 999
		}
		valid = age <= tr.cfg.HoldSeconds
		if !valid {
			tr.vx *= tr.cfg.Decay
			tr.vy *= tr.cfg.Decay
			tr.vsize *= tr.cfg.Decay
			tr.size *= 0.95
		}
	}

	conf := 0.0
	if obs.Detected {
		conf = obs.Confidence
	}

	return AnchorState{
		T: t, Valid: valid, Confidence: conf,
		CX: tr.cx, CY: tr.cy, VX: tr.vx, VY: tr.vy,
		Size: tr.size, VSize: tr.vsize, Age: age,
	}
}
