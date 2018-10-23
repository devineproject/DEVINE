Installation
############

DEVINE is a project with **many** dependencies such as ``ROS``. 

We recommend you to install it on a fresh copy of Ubuntu 16.04 LTS. 

Fresh Ubuntu 16.04 LTS
======================

* Create a ``catkin workspace`` directory like explained in the `ROS tutorial`_
* Create ``src`` directory under it
* Clone the `DEVINE repository`_ in ``src/`` **Make sure not to rename the repository**
* Navigate to ``DEVINE/scripts``
* Run the following command

.. code-block:: bash

    ./install.sh {path/to/catkin/src} {path/to/devine/root}


Virtual Box
===========

The DEVINE project can be installed in a virtual machine.

To do so, make sure you have a VM with Ubuntu 16.04 installed, and follow the steps of installing `Fresh Ubuntu 16.04 LTS`_.

Note about running the project in Virtual Box
---------------------------------------------

To allow the Xbox Kinect connected physically to the host to communicate with the VM, you must link your USB devices from the host to the client:

.. image:: link-kinect-usb.png

There should be three devices to select for the Kinect:

* Microsoft Xbox NUI Motor
* Microsoft Xbox NUI Camera
* Microsoft Xbox NUI Audio

If you get an error while linking the devices, it may be possible that the device is busy by another process. The simplest way to solve that is to restart the client and restart the host.

You may also need to install `Oracle VM VirtualBox Extension Pack <https://www.virtualbox.org/wiki/Downloads>`_ in order to allow the use of **USB 2.0** in the settings of your VM.


Docker
======

Docker is an application which runs a program in an isolated environment with its dependencies, akin to a virtual machine. Docker is portable, lightweight and allows for compatibility.

How to get started
------------------

Navigate to the docker folder. Run ``build.sh``.
This will get the devine-base image and build the devine image.
The devine-base image contains all of the projects dependencies and can be rebuilt if necessary using ``./base/build-base.sh``.
The devine image contains the actual code.
Separating the dependencies from the code speed up further DEVINE builds.

Once the build is complete, you can validate by running ``sudo docker images``. One docker should be named devine.

With an image in hand, run the command ``./run.sh``.
This will launch an instance of your docker image.
You will arive in a ubuntu like terminal which has the same layout as the code base.
To exit, use ctrl+d. 

Useful commands
---------------

* ``sudo docker container ls``: Lists all containers currently running
* ``sudo docker exec -it {containerId} bash``: starts another bash in a given docker container
* ``docker cp {path/to/filename} {containerId}:{Destination/Path/}`` copy a file into a specific docker image

.. _DEVINE repository: https://github.com/FelixMartel/DEVINE
.. _ROS tutorial: https://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
