#!/usr/bin/env bash

# Docker image: https://github.com/dldl/sphinx-server
ROOT=$(dirname "$(readlink -f "$0")")/..

sudo docker run --rm -it -v "$ROOT/docs/source":/web -p 8000:8000 --name sphinx-server dldl/sphinx-server
