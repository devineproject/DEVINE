# DEVINE

First clone this repository and its submodules

```
git clone --recursive git@github.com:FelixMartel/DEVINE.git
```

## Play as oracle 

Tip: Go to [http://cocodataset.org/#explore](http://cocodataset.org/#explore) and search for your image name

This will require just under 3.5 Go of RAM

### In your environment

```
./install_pretrained.sh { tensorflow | tensorflow-gpu }
./play_oracle_pretrained.sh
```

### Inside docker

Install docker and docker-compose

```
sudo docker-compose run --rm game
```

### Inside gpu backed docker (WIP)

Install docker, nivdia-docker2 and the latest docker-compose (1.19.0 and up)

```
sudo docker-compose -f docker-compose.gpu.yml run --rm game
```
