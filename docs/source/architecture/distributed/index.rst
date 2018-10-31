Distributed Computing
#####################

As part of this project we experimented with running ROS nodes on multiple machines.

This solution was developed to suit our project's needs by allowing it to run on a remote server with its dependencies inside a container.

Initial network configuration
=============================

First, create a docker network. In this tutorial we will use subnet ``172.20.0.0/16``, but you may need to change subnet so it does not conflict with existing networks. On each machine run:

.. code-block:: bash

  docker network create --subnet 172.20.0.0/16 --gateway 172.20.0.1 devine

This will create a bridge interface named ``br-${networkId}``. The network id can be recovered using ``docker network ls``.

Bringing up the nodes
=====================

When bringing up the containers, assign them an ip (within the subnet) and a hostname. ROS nodes also need to be able to reach the rosmaster specified by the environment variable ROS_MASTER_URI.

For example, run on the first machine:

.. code-block:: bash

  docker run -ti --rm --runtime=nvidia --network devine \
  --hostname machine1 --ip 172.20.0.16 \
  --add-host machine2:172.20.0.15 \
  -e ROS_MASTER_URI=http://172.20.0.16:11311 \
  devine bash

On the second machine run:

.. code-block:: bash

  docker run -ti --rm --runtime=nvidia --network devine \
  --hostname machine2 --ip 172.20.0.15 \
  --add-host machine1:172.20.0.16 \
  -e ROS_MASTER_URI=http://172.20.0.16:11311 \
  devine bash

Tunneling
=========

To link the containers we use ssh tunneling.

From machine1 run:

.. code-block:: bash

  ssh -o Tunnel=ethernet -w 123:123 root@machine2

This will create a tap interface named tap123 on each side.

We connect these taps to the bridge. On each machine run:

.. code-block:: bash

  ip link set tap123 master br-$(docker network ls -f name=devine | grep devine | awk '{print $1}')
  ip link set tap123 up

