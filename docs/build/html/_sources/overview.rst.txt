Overview
************************

The general structure of the package is illustrated in :numref:`generalStructure`. All the classes of the package are categorized into :ref:`tire-model`, :ref:`vehicle-model` and :ref:`graphics-feat`. One Vehicle model and one Tire model are combined to form the System. The integration of the System, with the appropriate parameters and initial conditions, is performed through the standard `ode45 <https://octave.sourceforge.io/octave/function/ode45.html>`_ function of Octave/Matlab. The resulting data can be plotted as Frame and Animation with the Graphics features.

This package uses an object-oriented programming architecture. For more details see `Object-Oriented Programming in OCTAVE <https://www.gnu.org/software/octave/doc/interpreter/Object-Oriented-Programming.html>`_ or `Object-Oriented Programming in MATLAB <https://www.mathworks.com/discovery/object-oriented-programming.html>`_

.. _generalStructure:
.. figure:: ../illustrations/misc/flowchart.svg
    :align:   center

    General structure of the package.

The links to the description page of the available models and graphics are listed below.

.. _tire-model:

Tire model
========================

The tire models and the respective theoretical foundation and class/function are listed below.

======================================= ======================================= =======================================
Name                                    Theory                                  Class/Function
======================================= ======================================= =======================================
Tire Linear                             :ref:`tire-linear`                      :mod:`TireLinear`
Tire Pacejka                            :ref:`tire-pacejka`                     :mod:`TirePacejka`
Tire Polynomial                         :ref:`tire-polynomial`                  :mod:`TirePolynomial`
======================================= ======================================= =======================================

.. _vehicle-model:

Vehicle model
========================

The vehicle models and the respective theoretical foundation and class/function are listed below.

======================================= ======================================= =======================================
Name                                    Theory                                  Class/Function
======================================= ======================================= =======================================
Vehicle Articulated Linear              :ref:`vehicle-articulated-4dof`         :mod:`VehicleArticulatedLinear`
Vehicle Articulated Nonlinear           :ref:`vehicle-articulated-4dof`         :mod:`VehicleArticulatedNonlinear`
Vehicle Simple Linear                   :ref:`vehicle-simple-3dof`              :mod:`VehicleSimpleLinear`
Vehicle Simple Nonlinear                :ref:`vehicle-simple-3dof`              :mod:`VehicleSimpleNonlinear`
Vehicle Simple Nonlinear 4DOF           :ref:`vehicle-simple-4dof`              :mod:`VehicleSimpleNonlinear4DOF`
======================================= ======================================= =======================================

.. _graphics-feat:

Graphics
========================

:mod:`Graphics`
