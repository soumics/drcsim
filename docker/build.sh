#!/bin/bash
# Build the drcsim image (NVIDIA + CycloneDDS). Usage: docker/build.sh [tag]
set -e
cd "$(dirname "$0")/.."
docker build -f docker/Dockerfile -t "${1:-drcsim:jazzy}" .
