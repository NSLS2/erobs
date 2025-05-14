#!/bin/bash
set -e

whoami
echo "Running in $(pwd)"
echo "Running in $(dirname $0)"
echo "Running in $(realpath $0)"
sudo chown -R ros:ros src
ls -la
./setup.sh
./build.sh
# ./test.sh
