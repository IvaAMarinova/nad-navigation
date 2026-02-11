package nad_nav

import (
	"errors"
	"fmt"
	"net"
	"strconv"
	"strings"
	"sync"
	"time"
)

// LiveRunConfig wraps configuration for the live controller.
type LiveRunConfig struct {
	App AppConfig
}

// RunLive starts the UDP-to-UDP control loop.
func RunLive(cfg AppConfig) error {
	if cfg.Hz <= 0 {
		return fmt.Errorf("hz must be > 0")
	}
	if cfg.Live.UDPAddr == "" {
		return fmt.Errorf("live.udp_addr must be set")
	}

	store := &liveStore{}
	if err := startUDPListener(cfg.Live, store); err != nil {
		return err
	}

	tracker := NewAnchorTracker(cfg.Tracker)
	controller := NewDroneController(cfg.Controller)
	sender, err := NewOutputSender(cfg.Output.UDPAddr)
	if err != nil {
		return err
	}
	viz, err := StartViz(cfg.Viz)
	if err != nil {
		return err
	}
	defer func() {
		_ = sender.Close()
	}()

	dtTarget := 1.0 / cfg.Hz
	t0 := time.Now()
	lastWall := time.Now()
	var lastSeq uint64
	var lastObs AnchorObservation

	for {
		now := time.Now()
		simT := now.Sub(t0).Seconds()

		obs, hasT, seq := store.Snapshot()
		if seq != lastSeq {
			lastSeq = seq
			lastObs = obs
			if !hasT {
				lastObs.T = simT
			}
		} else {
			lastObs = AnchorObservation{T: simT, Detected: false, Confidence: 0}
		}
		if viz != nil {
			viz.UpdateInput(lastObs)
		}

		st := tracker.Update(lastObs, controller.Cfg.ConfMin)

		dtReal := mathMax(1e-3, now.Sub(lastWall).Seconds())
		lastWall = now

		cmd := controller.Step(st, dtReal)
		sender.Send(cmd)
		if viz != nil {
			viz.UpdateOutput(cmd)
		}

		if cfg.Log.Enabled {
			fmt.Printf(
				"%8.3f mode=%-14s obs(cx=%+.3f cy=%+.3f size=%.3f det=%t conf=%.2f) "+
					"state(cx=%+.3f cy=%+.3f age=%.2f valid=%t) "+
					"cmd(yaw=%+.3f vert=%+.3f fwd=%+.3f)\n",
				cmd.T,
				cmd.Mode.String(),
				lastObs.CX,
				lastObs.CY,
				lastObs.Size,
				lastObs.Detected,
				lastObs.Confidence,
				st.CX,
				st.CY,
				st.Age,
				st.Valid,
				cmd.Yaw,
				cmd.Vertical,
				cmd.Forward,
			)
		}

		elapsed := time.Since(now).Seconds()
		sleep := mathMax(0, dtTarget-elapsed)
		time.Sleep(time.Duration(sleep * float64(time.Second)))
	}
}

type liveStore struct {
	mu       sync.RWMutex
	last     AnchorObservation
	lastHasT bool
	seq      uint64
}

// Update stores the latest observation and advances the sequence counter.
func (s *liveStore) Update(obs AnchorObservation, hasT bool) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.last = obs
	s.lastHasT = hasT
	s.seq++
}

// Snapshot returns the most recent observation and metadata.
func (s *liveStore) Snapshot() (AnchorObservation, bool, uint64) {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.last, s.lastHasT, s.seq
}

// startUDPListener spawns a goroutine that listens for observation packets.
func startUDPListener(cfg LiveConfig, store *liveStore) error {
	addr, err := net.ResolveUDPAddr("udp", cfg.UDPAddr)
	if err != nil {
		return err
	}
	conn, err := net.ListenUDP("udp", addr)
	if err != nil {
		return err
	}

	bufSize := cfg.ReadBuffer
	if bufSize <= 0 {
		bufSize = 2048
	}

	go func() {
		buf := make([]byte, bufSize)
		for {
			n, _, err := conn.ReadFromUDP(buf)
			if err != nil {
				continue
			}
			obs, hasT, err := parseLiveObservation(buf[:n])
			if err != nil {
				continue
			}
			store.Update(obs, hasT)
		}
	}()

	return nil
}

// parseLiveObservation parses CSV payloads into AnchorObservation objects.
func parseLiveObservation(b []byte) (AnchorObservation, bool, error) {
	s := strings.TrimSpace(string(b))
	if s == "" {
		return AnchorObservation{}, false, errors.New("empty payload")
	}

	parts := strings.Split(s, ",")
	if len(parts) != 5 && len(parts) != 6 {
		return AnchorObservation{}, false, fmt.Errorf("expected 5 or 6 fields, got %d", len(parts))
	}

	var idx int
	var t float64
	var err error
	if len(parts) == 6 {
		t, err = parseF64(parts[0])
		if err != nil {
			return AnchorObservation{}, false, err
		}
		idx = 1
	}

	detected, err := parseBoolLoose(parts[idx])
	if err != nil {
		return AnchorObservation{}, false, err
	}
	conf, err := parseF64(parts[idx+1])
	if err != nil {
		return AnchorObservation{}, false, err
	}
	cx, err := parseF64(parts[idx+2])
	if err != nil {
		return AnchorObservation{}, false, err
	}
	cy, err := parseF64(parts[idx+3])
	if err != nil {
		return AnchorObservation{}, false, err
	}
	size, err := parseF64(parts[idx+4])
	if err != nil {
		return AnchorObservation{}, false, err
	}

	obs := AnchorObservation{T: t, Detected: detected, Confidence: conf, CX: cx, CY: cy, Size: size}
	return obs, len(parts) == 6, nil
}

// parseF64 parses a float from a CSV field.
func parseF64(value string) (float64, error) {
	return strconv.ParseFloat(strings.TrimSpace(value), 64)
}

// parseBoolLoose parses booleans from common telemetry encodings.
func parseBoolLoose(value string) (bool, error) {
	norm := strings.ToLower(strings.TrimSpace(value))
	switch norm {
	case "1", "true", "yes", "y", "t":
		return true, nil
	case "0", "false", "no", "n", "f":
		return false, nil
	default:
		f, err := strconv.ParseFloat(norm, 64)
		if err != nil {
			return false, err
		}
		return f != 0, nil
	}
}

// mathMax returns the larger of a or b.
func mathMax(a, b float64) float64 {
	if a > b {
		return a
	}
	return b
}
