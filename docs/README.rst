How to generate the documentation
#################################

Using Docker
============

We use a `Sphinx Server`_ as the container.

The documentation will automatically refresh if you change the docs!

* Run ``./serve_doc.sh``
* In your browser, navigate to ``http://localhost:8000/``
* Enjoy!


Local Env
=========

Environment Setup
-----------------

.. code-block:: bash

    pip install sphinx
    pip install sphinx-rtd-theme
    pip install sphinxcontrib-plantuml
    pip install graphviz

Build
-----

* Navigate to ``/docs``;
* `make html`
* Verify build succeeded;
* Open `/build/html/index.html` in your favorite browser;
* Enjoy!


.. _Sphinx Server: https://github.com/dldl/sphinx-server
