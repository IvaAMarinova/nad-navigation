.PHONY: run run-official

run:
	go run ./cmd/nad --config config.testing.json

run-official:
	go run ./cmd/nad --config config.official.json
