#!/usr/bin/env bash

# Docker image: https://github.com/dldl/sphinx-server
# To stop the server, use docker stop sphinx-server
# To start the server, use docker start sphinx-server
# To remove the server, use docker rm -v sphinx-server

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
docker run --rm -it -v $DIR"/source":/web -p 8000:8000 --name sphinx-server dldl/sphinx-server
