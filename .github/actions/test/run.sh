#!/bin/bash
set -e

whoami
echo "Running in $(pwd)"
echo "Running in $(dirname $0)"
echo "Running in $(realpath $0)"
ls -la
./setup.sh
./build.sh
# ./test.sh
