.. _fresh_ubuntu:

Ubuntu 16.04 LTS
################

We recommend you to install it on a fresh copy of Ubuntu 16.04 LTS. 

The following steps will install all the dependencies for the DEVINE project.

1. Create a ``catkin workspace`` directory like explained in the `ROS tutorial`_.
2. Create ``src`` directory under it.
3. Clone the `DEVINE repository`_ in ``src/``. **Make sure not to rename the repository**
4. Navigate to ``DEVINE/scripts``.
5. Run the following command:

.. code-block:: bash

    $ ./install.sh {path/to/catkin/src} {path/to/devine/root}

GPU Usage - Optional
====================

If you want to use your GPU instead of your CPU for the computation, follow the GPU setup bellow.

.. toctree::
    :glob:
    :maxdepth: 1
   
    gpu/index

.. _DEVINE repository: https://github.com/devineproject/DEVINE
.. _ROS tutorial: https://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
