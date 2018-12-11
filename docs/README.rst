How to generate the documentation
#################################

Using Docker
============

We use a `Sphinx Server`_ as the container.

The documentation will automatically refresh if you change the docs!

* Run ``$ ./serve_doc.sh``
* In your browser, navigate to ``http://localhost:8000/``
* Enjoy!


Local Env
=========

Environment Setup
-----------------

.. code-block:: bash

    $ pip install -r requirements.txt --user


Build
-----

* Navigate to ``/docs``;
* ``$ make html``
* Verify build succeeded;
* Open ``/build/html/index.html`` in your favorite browser;
* Enjoy!


Docker
------

If you already have ``Docker`` installed, simply navigate to ``/docs`` and run the following command to start a sphinx server at port :8000.

.. code-block:: bash
    
    $ ./serve_doc.sh


.. _Sphinx Server: https://github.com/dldl/sphinx-server
