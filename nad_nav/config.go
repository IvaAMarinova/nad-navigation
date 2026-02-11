package nad_nav

import (
	"encoding/json"
	"fmt"
	"os"
	"strings"
)

// LiveConfig controls UDP input settings for camera observations.
type LiveConfig struct {
	UDPAddr    string `json:"udp_addr"`
	ReadBuffer int    `json:"read_buffer"`
}

// OutputConfig controls UDP output settings for controller commands.
type OutputConfig struct {
	UDPAddr string `json:"udp_addr"`
}

// LogConfig controls console logging.
type LogConfig struct {
	Enabled bool `json:"enabled"`
}

// AppConfig aggregates all configuration sections.
type AppConfig struct {
	Hz         float64          `json:"hz"`
	Tracker    TrackerConfig    `json:"tracker"`
	Controller ControllerConfig `json:"controller"`
	Live       LiveConfig       `json:"live"`
	Output     OutputConfig     `json:"output"`
	Viz        VizConfig        `json:"viz"`
	Log        LogConfig        `json:"log"`
}

// LoadConfig reads the JSON config from disk.
func LoadConfig(path string) (AppConfig, error) {
	var cfg AppConfig
	data, err := os.ReadFile(path)
	if err != nil {
		return cfg, err
	}
	if err := json.Unmarshal(data, &cfg); err != nil {
		return cfg, err
	}
	return cfg, nil
}

// ParseMode converts a mode name into a Mode enum.
func ParseMode(value string) (Mode, error) {
	normalized := strings.ToUpper(strings.TrimSpace(value))
	switch normalized {
	case "SEARCH":
		return ModeSearch, nil
	case "TRACK":
		return ModeTrack, nil
	case "APPROACH":
		return ModeApproach, nil
	case "CAPTURE":
		return ModeCapture, nil
	case "FLY_STRAIGHT":
		return ModeFlyStraight, nil
	case "LATERAL_ONLY":
		return ModeLateralOnly, nil
	default:
		return ModeSearch, fmt.Errorf("unknown mode %q", value)
	}
}

// UnmarshalJSON allows modes to be loaded from JSON strings.
func (m *Mode) UnmarshalJSON(b []byte) error {
	var raw *string
	if err := json.Unmarshal(b, &raw); err == nil {
		if raw == nil {
			return nil
		}
		parsed, err := ParseMode(*raw)
		if err != nil {
			return err
		}
		*m = parsed
		return nil
	}

	var fallback string
	if err := json.Unmarshal(b, &fallback); err != nil {
		return err
	}
	parsed, err := ParseMode(fallback)
	if err != nil {
		return err
	}
	*m = parsed
	return nil
}
