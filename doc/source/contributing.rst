Contributing
********************************************************************************

General instructions

Regardless of the type of contribution the user has to

Improving documentation

Creating new examples

Creating new tire and vehicle models


Push from docs to gh-pages branch

    git subtree push --prefix docs origin gh-pages


Instructions
================================================================================

There are several ways to contribute to open source projects (`Contributing to open source <https://guides.github.com/activities/contributing-to-open-source/>`_).

To push your contribution see the following steps:

* Add and/or improve the .m files (package or examples) with codes and publishable comments.
* Add the publish command of the new files to DocFiles/makeDoc.m.
* Create the apropriate links between the documentation pages. Ex: "See Also", "examples", ...
* Update main.m and api.m.
* Run makeDoc.m.
* Commit and push.


Instructions documentation contribution for windows with anaconda
================================================================================

To prepare the system open up a terminal and install:

* cloud_sptheme

.. code-block::  none

    conda install -c conda-forge cloud_sptheme 
 
* sphinxcontrib-bibtex

.. code-block:: none 

    conda install -c conda-forge sphinxcontrib-bibtex 

* pybtex (optional)

.. code-block:: none 

    conda install -c omnia pybtex 

The workflow for updating and pushing your contribution see the following steps:

  * I had to navigate to the doc folder (in master branch) and execute "make html". this should update the html docs in the folder doc/build. This folder is ignored so you should not need to push anything from this folder.
  * checkout the *gh-pages* branch folder
  * copy the <openvd-root>/doc/build folder into <openvd-root>/build
  * commit and push changes to the openvd repo (gh-pages). After a few minutes the documentation changes should be online and visible




To Do List
================================================================================

.. todolist::
