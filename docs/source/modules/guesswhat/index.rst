GuessWhat
#########

Description
===========

GuessWhat has been developed by `IGLU`_.

TODO: More in depth information.

ROS Installation
================

1. Run the install script `./install_package.bash`
2. Build the module using `catkin_make`:

.. code-block:: bash

    roscd
    cd ..
    catkin_make

Usage
=====

.. code-block:: bash

    rosrun devine_guesswhat guesswhat_node.py

Play The Game!
==============

1. Clone this repository and its submodules

.. code-block:: bash

    git clone --recursive git@github.com:FelixMartel/DEVINE.git

2. Install `Docker`_ and `Docker Compose`_


Play As Oracle
^^^^^^^^^^^^^^

This should require just under 3.5 Go of RAM

Tip: Once the game is started go to `COCO Dataset`_  and search for your image name.

In your environment

.. code-block:: bash

    ./install_pretrained.sh { tensorflow | tensorflow-gpu }
    ./play_oracle_pretrained.sh
    sudo docker-compose run --rm game

Inside gpu backed docker (WIP)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Install docker, nivdia-docker2 and the latest docker-compose (1.19.0 and up)

.. code-block:: bash

    sudo docker-compose -f docker-compose.gpu.yml run --rm game

.. _Docker: https://docs.docker.com/install/linux/docker-ce/ubuntu/
.. _Docker Compose: https://docs.docker.com/compose/install/
.. _COCO Dataset: http://cocodataset.org/#explore
.. _IGLU: https://iglu-chistera.github.io/
