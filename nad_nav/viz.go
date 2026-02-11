package nad_nav

import (
	"expvar"
	"log"
	"net/http"
)

// VizConfig controls the optional expvar endpoint used by jplot.
type VizConfig struct {
	Enabled bool   `json:"enabled"`
	Addr    string `json:"addr"`
}

// VizMetrics exposes live input/output values via expvar.
type VizMetrics struct {
	input  *expvar.Map
	output *expvar.Map
	flat   map[string]*expvar.Float
}

// StartViz starts an HTTP server exposing /debug/vars for plotting.
func StartViz(cfg VizConfig) (*VizMetrics, error) {
	if !cfg.Enabled {
		return nil, nil
	}
	if cfg.Addr == "" {
		cfg.Addr = "127.0.0.1:7070"
	}

	metrics := &VizMetrics{
		input:  expvar.NewMap("input"),
		output: expvar.NewMap("output"),
		flat:   map[string]*expvar.Float{},
	}
	metrics.input.Set("cx", new(expvar.Float))
	metrics.input.Set("cy", new(expvar.Float))
	metrics.input.Set("size", new(expvar.Float))
	metrics.output.Set("yaw", new(expvar.Float))
	metrics.output.Set("vertical", new(expvar.Float))
	metrics.output.Set("forward", new(expvar.Float))
	metrics.output.Set("mode", new(expvar.Float))
	metrics.flat["input_cx"] = expvar.NewFloat("input_cx")
	metrics.flat["input_cy"] = expvar.NewFloat("input_cy")
	metrics.flat["input_size"] = expvar.NewFloat("input_size")
	metrics.flat["output_yaw"] = expvar.NewFloat("output_yaw")
	metrics.flat["output_vertical"] = expvar.NewFloat("output_vertical")
	metrics.flat["output_forward"] = expvar.NewFloat("output_forward")
	metrics.flat["output_mode"] = expvar.NewFloat("output_mode")

	server := &http.Server{Addr: cfg.Addr, Handler: http.DefaultServeMux}
	go func() {
		if err := server.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			log.Printf("viz server error: %v", err)
		}
	}()

	return metrics, nil
}

// UpdateInput publishes the latest camera input values.
func (v *VizMetrics) UpdateInput(obs AnchorObservation) {
	if v == nil {
		return
	}
	setFloat(v.input, "cx", obs.CX)
	setFloat(v.input, "cy", obs.CY)
	setFloat(v.input, "size", obs.Size)
	setFlat(v.flat, "input_cx", obs.CX)
	setFlat(v.flat, "input_cy", obs.CY)
	setFlat(v.flat, "input_size", obs.Size)
}

// UpdateOutput publishes the latest controller output values.
func (v *VizMetrics) UpdateOutput(cmd BodyCommand) {
	if v == nil {
		return
	}
	setFloat(v.output, "yaw", cmd.Yaw)
	setFloat(v.output, "vertical", cmd.Vertical)
	setFloat(v.output, "forward", cmd.Forward)
	setFloat(v.output, "mode", float64(cmd.Mode))
	setFlat(v.flat, "output_yaw", cmd.Yaw)
	setFlat(v.flat, "output_vertical", cmd.Vertical)
	setFlat(v.flat, "output_forward", cmd.Forward)
	setFlat(v.flat, "output_mode", float64(cmd.Mode))
}

// setFloat updates an expvar.Float stored inside a map.
func setFloat(m *expvar.Map, key string, value float64) {
	if v := m.Get(key); v != nil {
		if f, ok := v.(*expvar.Float); ok {
			f.Set(value)
			return
		}
	}
	f := new(expvar.Float)
	f.Set(value)
	m.Set(key, f)
}

func setFlat(vars map[string]*expvar.Float, key string, value float64) {
	if v, ok := vars[key]; ok {
		v.Set(value)
	}
}
