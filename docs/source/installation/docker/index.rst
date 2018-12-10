.. _docker_install:

Docker
######

Docker is an application which runs a program in an isolated environment with its dependencies, akin to a virtual machine. Docker is portable, lightweight and allows for compatibility.

How to get started
==================

First, navigate to the docker folder.

Build the docker image for CPU use:

.. code-block:: bash

    $ ./build.sh

Or build the docker image for GPU use:

.. code-block:: bash

    $ ./build.sh --gpu

Theses commands will get the devine-base image and build the devine image.

Once the build is complete, you can validate by running ``sudo docker images``. One docker should be named devine.
With an image in hand, simply run the command to launch an instance of your docker image:

.. code-block:: bash

    $ ./run.sh

You will arive in a ubuntu like terminal which has the same layout as the code base.
To exit, use ctrl+d. 

*Note*: both ``run.sh`` and ``build.sh`` have some arguments that can be set depending on your usage.
Use the argument ``--help`` for more information.


Information about the DEVINE docker images
==========================================

The DEVINE project uses two docker images:

- *devine-base*: contains all of the projects dependencies and can be rebuilt if necessary using ``./base/build-base.sh``.
- *devine*: contains the actual code.

Separating the dependencies from the code speed up further DEVINE builds.


Useful commands
===============

.. code-block:: bash

    sudo docker container ls                                       # Lists all containers currently running
    sudo docker exec -it {containerId} bash                        # starts another bash in a given docker container
    docker cp {path/to/filename} {containerId}:{Destination/Path/} # copy a file into a specific docker image
