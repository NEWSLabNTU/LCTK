#!/bin/sh
root_dir="$(git rev-parse --show-toplevel)"
output_dir="${root_dir}/recording/pcap"
mkdir -p "$output_dir"
tshark -i enp7s0 -w "${output_dir}/$(date --iso-8601=ns).pcap" udp
