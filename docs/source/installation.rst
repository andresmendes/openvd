Installation
********************************************************************************

Octave users
================================================================================

* Download the OpenVD package by clicking `here <https://github.com/andresmendes/openvd/archive/master.zip>`_.

* Save the zip file in your package *path*.

* Open Octave, go to your package *path* and run (For more details see `pkg function <https://octave.sourceforge.io/octave/function/pkg.html>`_.)::

    pkg install openvd-master.zip

* Load the OpenVD package with::

    pkg load openvd

* To check the installation, change the current path with::

    cd openvd-0.0.0/docs/examples/TemplateSimple/

* and run the :ref:`template-simple` example with::

    TemplateSimple

Matlab users
================================================================================

* Download the OpenVD package by clicking `here <https://github.com/andresmendes/openvd/archive/master.zip>`_.

* Unzip the openvd-master.zip file in your current directory.

* Open Matlab and add the *inst* directory to the *paths list* with (For more details see `help path <http://www.mathworks.com/help/matlab/ref/path.html>`_.)::

    addpath('openvd-master/inst')

* To check the installation, change the current path with::

    cd openvd-master/docs/examples/TemplateSimple/

* and run the :ref:`template-simple` example with::

    TemplateSimple

Dependencies
================================================================================

To run the functions of the package Octave > 4.0 or Matlab > v8.5 is required.
