Overview
************************

The general structure of the package is illustrated below. All the classes of the package are categorized into Vehicle model, Tire model and Graphics. One Vehicle model and one Tire model are combined to form the System. The integration of the System, with the apropriate parameters and initial conditions, is performed through the standard `ode45 <https://octave.sourceforge.io/octave/function/ode45.html>`_ function of Octave/Matlab. The resulting data can be plotted as Frame and Animation with the Graphics features.

This package uses an object-oriented programming architecture. For more details see `Object-Oriented Programming in OCTAVE <https://www.gnu.org/software/octave/doc/interpreter/Object-Oriented-Programming.html>`_ or `Object-Oriented Programming in MATLAB <https://www.mathworks.com/discovery/object-oriented-programming.html>`_

.. figure:: https://andresmendes.github.io/openvd/illustrations/fluxograma.svg

The links to the description page of the available models and graphics are listed below.

Tire model
========================

The tire models are listed below.

* `Tire linear <html/DocTireLinear.html>`_
* `Tire Pacejka <html/DocTirePacejka.html>`_
* `Tire polynomial <html/DocTirePolynomial.html>`_

Vehicle model
========================

The vehicle models are listed below.

The theoretical foundation of vehicle models can be found in: `TheoryVehicleSimple <theory/vehicleSimple.pdf>`_, `TheoryVehicleSimple4DOF <theory/vehicleSimple4DOF.pdf>`_ and `TheoryVehicleArticulated <theory/vehicleArticulated.pdf>`_.

* `Vehicle Articulated Linear <html/DocVehicleArticulatedLinear.html>`_
* `Vehicle Articulated Nonlinear <html/DocVehicleArticulatedNonlinear.html>`_
* `Vehicle Simple Linear <html/DocVehicleSimpleLinear.html>`_
* `Vehicle Simple Nonlinear <html/DocVehicleSimpleNonlinear.html>`_
* `Vehicle Simple Nonlinear 4DOF <html/DocVehicleSimpleNonlinear4DOF.html>`_

Graphics
========================

* `Graphics <html/DocGraphics.html>`_
