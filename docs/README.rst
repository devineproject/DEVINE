How to generate the documentation
#################################

Using Docker
============

We use a `Sphinx Server`_ as the container.

The documentation will automatically refresh if you change the docs!

* navigate to ``/docs``
* run ``./serve_doc.sh``
* In your browser, navigate to ``http://localhost:8000/``
* profit!


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

* navigate to ``/docs``;
* `make html`
* verify build succeeded;
* open `/build/html/index.html` in your favorite browser;
* profit!


.. _Sphinx Server: https://github.com/dldl/sphinx-server