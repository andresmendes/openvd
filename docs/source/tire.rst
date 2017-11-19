Tire models
********************************************************************************

The tire models describe the forces in the road-tire interactions according to the dynamic condition of the vehicle.

The typical relation between the lateral force and the slip angle can be observed in :numref:`lateralForce`. Besides, it is possible to verify the definition of slip angle.

.. _lateralForce:
.. figure:: ../illustrations/misc/lateralForce.png
    :align:   center

    Tire lateral force and slip angle (Adapted from :cite:`Gillespie1992fundamentals`).

.. _tire-linear:

Linear
================================================================================

The linear tire model describes a linear relationship between tire lateral force and slip angle.

The lateral force of the tire can be calculated as

.. math:: F_y = K \alpha

where :math:`F_y` is the lateral force, :math:`K` is the cornering stiffness and :math:`\alpha` is the tire slip angle.

This model is only valid for small values of slip angle

.. _tire-pacejka:

Pacejka (Magic Formula)
================================================================================

Nonlinear relationship between tire lateral force and slip angle expressed by a semi-empirical model with experimental coefficients :cite:`Bakker1989new`.

The lateral force can be written as

.. math:: F_y = - \frac{\mu_y}{\mu_{y, n}} \left( F_{y, n} \left(\alpha_{eq} \right) + S_v \right)

where :math:`\alpha_{eq}` is the equivalent slip angle

.. math:: \alpha_{eq} = \frac{\mu_{y0}}{\mu_y} \frac{F_{z0}}{F_z} \left( \alpha + S_h \right)

and :math:`F_{y, n}` is the reference function of the lateral force

.. math:: F_{y, n} = D \sin\left(C \arctan\left(B \alpha - E \left(B \alpha - \arctan\left(B \alpha\right)\right)\right)\right)

The coefficients :math:`B`, :math:`C`, :math:`D` and :math:`E` can be written as

.. math::

    C &= a_0 \\
    D &= \mu_{y, n} F_z = (a_1 F_z + a_2) F_z \\
    B &= \frac{B C D}{C D} = a_3 \sin \left\{ 2 \arctan \left( \frac{F_z}{a_4} \right) \right\} (1 - a_5 | \gamma |) \\
    E &= a_6 F_z + a_7

The horizontal and vertical shifts of the curve are calculated as

.. math::

    S_h &= a_8 \gamma + a_9 F_z + a_{10} \\
    S_v &= a_{11} F_z \gamma + a_{12} F_z + a_{13}

The model implemented here converts the slip angle using the following equation

 _ALPHA = asin(sin(alpha));_

This equation alters the slip angle in such way that the characteristic equation becames symmetric in relation to the vertical line at 90 degrees and the lateral force becomes zero at 180 degrees. The same analogy can be made with negative values of the slip angle. Thus, this model covers the whole range of slip angles (-180 to 180 degrees).

.. _tire-polynomial:

Polynomial
================================================================================

Nonlinear relation between tire lateral force and slip angle expressed by a third order polinomial equation.

This model is used by several authors :cite:`Johnson1984nonlinear,Sadri2013stability,Samsundar1998estimating`.

The equation of the model is given by

.. math::  F_y = k_1 \alpha  - k_2\alpha^3

where :math:`F_y` is the lateral force and :math:`\alpha` is the tire slip angle. :math:`k_1` and :math:`k_2` are the model coefficients. This model is valid up to the maximal lateral force (Tire saturation).

Tire comparison
================================================================================

Under construction.
