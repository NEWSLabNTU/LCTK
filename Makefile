.PHONY: default build clean

default: build

build:
	cargo build --release --all-targets

clean:
	cargo clean
