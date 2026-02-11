package main

import (
	"flag"
	"log"

	"nad-navigation/nad_nav"
)

func main() {
	var configPath string
	var liveAddr string
	var outputAddr string
	var modeOverride string
	flag.StringVar(&configPath, "config", "config.testing.json", "Path to JSON config.")
	flag.StringVar(&liveAddr, "live-addr", "", "Override live UDP listen addr (host:port).")
	flag.StringVar(&outputAddr, "output-addr", "", "Override output UDP addr (host:port).")
	flag.StringVar(&modeOverride, "mode-override", "", "Force controller mode (e.g., FLY_STRAIGHT).")
	flag.Parse()

	cfg, err := nad_nav.LoadConfig(configPath)
	if err != nil {
		log.Fatalf("load config %q: %v", configPath, err)
	}

	if liveAddr != "" {
		cfg.Live.UDPAddr = liveAddr
	}
	if outputAddr != "" {
		cfg.Output.UDPAddr = outputAddr
	}
	if modeOverride != "" {
		mode, err := nad_nav.ParseMode(modeOverride)
		if err != nil {
			log.Fatalf("invalid mode override %q: %v", modeOverride, err)
		}
		cfg.Controller.ModeOverride = &mode
	}

	if err := nad_nav.RunLive(cfg); err != nil {
		log.Fatal(err)
	}
}
