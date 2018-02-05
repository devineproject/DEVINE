# DEVINE

First clone this repository and its submodules

```
git clone --recursive git@github.com:FelixMartel/DEVINE.git
```

## Play as oracle 

Tip: Go to [http://cocodataset.org/#explore](http://cocodataset.org/#explore) and search for your image name

This will require just under 3.5 Go of RAM

### From cli

```
./install_pretrained.sh
./play_oracle_pretrained.sh
```

### From cli inside docker (WIP)

Install, docker, nivdia-docker2 and the latest docker-compose (1.19.0-rc0 and up)

```
sudo docker-compose up
```
