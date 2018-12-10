Getting Started
###############

DEVINE is a project with **many** dependencies such as `ROS`_. 

In this section, you can find links to different installation types that we support.

That being said, we **highly** recommand going with the :ref:`docker_install` way.

Installation
============

.. toctree::
    :glob:
    :maxdepth: 1
   
    */index

.. _ROS: http://www.ros.org/


Launching the project
=====================

The project uses a `devine.launch` file which can start all the required ROS nodes.

.. code-block:: bash

    $ roslaunch devine devine.launch

By default, this will launch all the nodes.
You can also specify which nodes to launch, like so:

.. code-block:: bash

    $ roslaunch devine devine.launch launch_all:=false dashboard:=true

Also by default, the launch file is made to run on a real robot.
To run in **simulation** only, you can change the `sim` argument:

.. code-block:: bash

    $ roslaunch devine devine.launch sim:=true

