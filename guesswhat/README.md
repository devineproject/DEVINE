# GuessWhat engine

## ROS node

### Installation

1. Run the install script `. install_package.bash`
2. Build the module using catkin\_make

```bash
roscd
cd ..
catkin_make
```

### Usage

```bash
rosrun devine_guesswhat guesswhat_node.py
```

## Play the game from command line

First clone this repository and its submodules

```
git clone --recursive git@github.com:FelixMartel/DEVINE.git
```

### Play as oracle 

Tip: Once the game is started go to [http://cocodataset.org/#explore](http://cocodataset.org/#explore) and search for your image name

This will require just under 3.5 Go of RAM

#### In your environment

```
./install_pretrained.sh { tensorflow | tensorflow-gpu }
./play_oracle_pretrained.sh
```

#### Inside docker

Install docker and docker-compose

```
sudo docker-compose run --rm game
```

#### Inside gpu backed docker (WIP)

Install docker, nivdia-docker2 and the latest docker-compose (1.19.0 and up)

```
sudo docker-compose -f docker-compose.gpu.yml run --rm game
```
