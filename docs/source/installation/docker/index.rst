.. _docker_install:

Docker
######

Docker is an application which runs a program in an isolated environment with its dependencies, akin to a virtual machine. Docker is portable, lightweight and allows for compatibility.

How to get started
==================

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
===============

.. code-block:: bash

    sudo docker container ls                                       # Lists all containers currently running
    sudo docker exec -it {containerId} bash                        # starts another bash in a given docker container
    docker cp {path/to/filename} {containerId}:{Destination/Path/} # copy a file into a specific docker image
