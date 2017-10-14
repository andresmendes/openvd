Simple vehicle models
********************************************************************************

Models of vehicles without articulation.

Simple vehicle 3 DOF
================================================================================

The linear version of this model is described by equations ?? and the nonlinear version of this model is described by equations ??.

The simple vehicle model is illustrated in :numref:`modelSimple`. The vector basis :math:`\{ {\rm O} {\bf i} {\bf j} {\bf k} \}` is fixed to the inertial reference frame. The vector basis :math:`\{ {\rm T} {\bf t}_x {\bf t}_y {\bf t}_z \}` is fixed to the simple vehicle. The vector basis :math:`\{ {\rm F} {\bf e}_x {\bf e}_y {\bf e}_z \}` is fixed to the front axle.

The center of gravity of the vehicle is located at the point :math:`{\rm T}`. The front and rear axles are located at the points :math:`{\rm F}` and :math:`{\rm R}`, respectively. Point :math:`{\rm O}` is the origin. The constant :math:`a` measures the distance of point :math:`{\rm F}` to :math:`{\rm T}` and :math:`b` the distance of point :math:`{\rm T}` to :math:`{\rm R}`. The angles :math:`\alpha_{\rm F}` e :math:`\alpha_{\rm R}` are the front and rear slip angles, respectively. :math:`\alpha_{\rm T}` is the vehicle side slip angle and :math:`\psi` is the vehicle yaw angle. :math:`\delta` is the steering angle.

.. _modelSimple:
.. figure:: _illustrations/modelSimple.svg
    :align:   center

    Simple model.

Equations of Motion
--------------------------------------------------------------------------------

The generalized coordinates are

.. math::

    q_1 &= x     \\
    q_2 &= y     \\
    q_3 &= \psi, \\

where :math:`x` and :math:`y` are the coordinates of the CG of the vehicle. :math:`\psi` is the yaw angle.

The position of the CG in relation to the origin :math:`{\rm O}` is

.. math:: {\bf p}_{{\rm T}/{\rm O}} = x \, {\bf i} + y \, {\bf j}.
    :label: positionVehicle

The time derivative of equation :eq:`positionVehicle` gives the velocity

.. math:: {\bf v}_{\rm T} = \dot{x} \, {\bf i} + \dot{y} \, {\bf j}.

The velocity of the front axle is

.. math:: {\bf v}_{\rm F} = {\bf v}_{\rm T} + {\bf r} \wedge {\bf p}_{{\rm F}/{\rm T}},

where :math:`{\bf p}_{{\rm F}/{\rm T}}` is the position of point :math:`{\rm F}` in relation to point :math:`{\rm T}` and :math:`{\bf r} = \dot{\psi} \, {\bf k}`. Thus,

.. math:: {\bf v}_{\rm F} = \left( \dot{x} - a \dot{\psi} \sin \psi \right) {\bf i} + \left( \dot{y} + a \dot{\psi} \cos \psi \right) {\bf j}.
    :label: simpleVehicleVelocityVectorFront

the velocity of the rear axle is

.. math:: {\bf v}_{\rm R} = {\bf v}_{\rm T} + {\bf r} \wedge {\bf p}_{{\rm R}/{\rm T}},

where :math:`{\bf p}_{{\rm R}/{\rm T}}` is the position of point :math:`{\rm R}` in relation to point :math:`{\rm T}`. Thus,

.. math:: {\bf v}_{\rm R} = \left( \dot{x} + b \dot{\psi} \sin \psi \right) {\bf i} + \left( \dot{y} -b \dot{\psi} \cos \psi \right) {\bf j}.
    :label: simpleVehicleVelocityVectorRear

Using equations :eq:`simpleVehicleVelocityVectorFront` and :eq:`simpleVehicleVelocityVectorRear`, the slip angles can be written as

.. math::
    :label: simpleVehicleSlipAngle

    \alpha_{\rm F} &= \arctan \left( \frac{\dot{y} + a \dot{\psi} \cos \psi}{ \dot{x} - a \dot{\psi} \sin \psi} \right) - \left( \delta + \psi \right) \\
    \alpha_{\rm R} &= \arctan \left( \frac{\dot{y} - b \dot{\psi} \cos \psi}{ \dot{x} + b \dot{\psi} \sin \psi} \right) - \psi

The force vector at the front axle is

.. math:: {\bf F}_{\rm F} = F_{x,{\rm F}} \, {\bf e}_x + F_{y,{\rm F}} \, {\bf e}_x,

It can be also written as

.. math:: {\bf F}_{\rm F} = \left[ F_{x,{\rm F}} \cos \left( \psi + \delta \right) - F_{y,{\rm F}} \sin \left( \psi + \delta \right) \right] {\bf i} + \left[ F_{x,{\rm F}} \sin \left( \psi + \delta \right) + F_{y,{\rm F}} \cos \left( \psi + \delta \right) \right] {\bf j}.
    :label: ForceAtFront

The force vector at the rear axle is

.. math:: {\bf F}_{\rm R} = F_{x,{\rm R}} {\bf t}_x + F_{y,{\rm R}} {\bf t}_y

or

.. math:: {\bf F}_{\rm R} = \left[ F_{x,{\rm R}} \cos \psi - F_{y,{\rm R}} \sin \psi \right] {\bf i} + \left[ F_{x,{\rm R}} \sin \psi + F_{y,{\rm R}} \cos \psi \right] {\bf j}.
    :label: ForceAtRear

The generalized forces are

.. math:: Q_k = \sum_{j = 1} ^p {\bf F}_j \cdot \frac{\partial {\bf p}_j}{\partial q_k}

In the model

.. math::
    :label: simpleVehicleGeneralizedForces

    Q_1 &= {\bf F}_{\rm F} \cdot \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial q_1} + {\bf F}_{\rm R} \cdot \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial q_1} \\
    Q_2 &= {\bf F}_{\rm F} \cdot \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial q_2} + {\bf F}_{\rm R} \cdot \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial q_2} \\
    Q_3 &= {\bf F}_{\rm F} \cdot \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial q_3} + {\bf F}_{\rm R} \cdot \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial q_3},

where

.. math::
    :label: simpleVehicleTermGenFor1

    \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial q_1} &= \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial x} = {\bf i} \\
    \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial q_2} &= \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial y} = {\bf j} \\
    \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial q_3} &= \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial \psi} = - a \sin \psi \, {\bf i} + a \cos \psi \, {\bf j}

and

.. math::
    :label: simpleVehicleTermGenFor2

    \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial q_1} &= \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial x} = {\bf i} \\
    \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial q_2} &= \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial y} = {\bf j} \\
    \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial q_3} &= \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial \psi} = b \sin \psi \, {\bf i} - b \cos \psi \, {\bf j}

Substituting equations :eq:`ForceAtFront`, :eq:`ForceAtRear`, :eq:`simpleVehicleTermGenFor1` and :eq:`simpleVehicleTermGenFor2` in equations :eq:`simpleVehicleGeneralizedForces` we have

.. math::
    Q_1 &= F_{x,{\rm F}} \cos \left( \psi + \delta \right) + F_{x,{\rm R}} \cos \psi - F_{y,{\rm F}} \sin \left( \psi + \delta \right) - F_{y,{\rm R}} \sin \psi \\
    Q_2 &= F_{x,{\rm F}} \sin \left( \psi + \delta \right)+ F_{x,{\rm R}} \sin \psi + F_{y,{\rm F}} \cos \left( \psi + \delta \right) + F_{y,{\rm R}} \cos \psi \\
    Q_3 &=  F_{x,{\rm F}} a \sin \delta  + F_{y,{\rm F}} a \cos \delta - F_{y,{\rm R}} b.

The kinetic energy of the system is given by

.. math:: T = \frac{1}{2} m_{\rm T} {\bf v}_{\rm T} \cdot {\bf v}_{\rm T} + \frac{1}{2} I_{\rm T} \dot{\psi}^2.

or

.. math:: T = \frac{1}{2} m_{\rm T} \left( \dot{x}^2 + \dot{y}^2 \right) + \frac{1}{2} I_{\rm T} \dot{\psi}^2.

The Lagrange formulation is given by

.. math:: \frac{d}{dt} \left( \frac{\partial T}{\partial \dot{q}_k} \right) - \frac{\partial T}{\partial q_k} = Q_k.
    :label: simpleVehicleLagrange

For the three generalized coordinates of the system

.. math::
    :label: lagrangeSecond

    \frac{\partial T}{\partial q_1} &= \frac{\partial T}{\partial x} = 0 \\
    \frac{\partial T}{\partial q_2} &= \frac{\partial T}{\partial y} = 0 \\
    \frac{\partial T}{\partial q_3} &= \frac{\partial T}{\partial \psi} = 0

and

.. math::

    \frac{\partial T}{\partial \dot{q}_1} &= \frac{\partial T}{\partial \dot{x}} = m_{\rm T} \dot{x} \\
    \frac{\partial T}{\partial \dot{q}_2} &= \frac{\partial T}{\partial \dot{y}} = m_{\rm T} \dot{y} \\
    \frac{\partial T}{\partial \dot{q}_3} &= \frac{\partial T}{\partial \dot{\psi}} = I_{\rm T} \dot{\psi}.


The time derivative is given by

.. math::
    :label: lagrangeFirst

    \frac{d}{dt} \left( \frac{\partial T}{\partial \dot{q}_1} \right) &= \frac{d}{dt} \left( \frac{\partial T}{\partial \dot{x}} \right) = m_{\rm T} \ddot{x} \\
    \frac{d}{dt} \left( \frac{\partial T}{\partial \dot{q}_2} \right) &= \frac{d}{dt} \left( \frac{\partial T}{\partial \dot{y}} \right) = m_{\rm T} \ddot{y} \\
    \frac{d}{dt} \left( \frac{\partial T}{\partial \dot{q}_3} \right) &= \frac{d}{dt} \left( \frac{\partial T}{\partial \dot{\psi}} \right) = I_{\rm T} \ddot{\psi}.

Substituting equations :eq:`lagrangeSecond` and :eq:`lagrangeFirst` in :eq:`simpleVehicleLagrange` the equations of motion become

.. math::
    m_{\rm T} \ddot{x} &= F_{x,{\rm F}} \cos \left( \psi + \delta \right) + F_{x,{\rm R}} \cos \psi - F_{y,{\rm F}} \sin \left( \psi + \delta \right) - F_{y,{\rm R}} \sin \psi \\
    m_{\rm T} \ddot{y} &= F_{x,{\rm F}} \sin \left( \psi + \delta \right)+ F_{x,{\rm R}} \sin \psi + F_{y,{\rm F}} \cos \left( \psi + \delta \right) + F_{y,{\rm R}} \cos \psi \\
    I_{\rm T} \ddot{\psi} &= F_{x,{\rm F}} a \sin \delta  + F_{y,{\rm F}} a \cos \delta - F_{y,{\rm R}} b,

this is,

.. math::
    :label: simpleVehicleEquationOfMotion

    \ddot{x} &= \frac{F_{x,{\rm F}} \cos \left( \psi + \delta \right) + F_{x,{\rm R}} \cos \psi - F_{y,{\rm F}} \sin \left( \psi + \delta \right) - F_{y,{\rm R}} \sin \psi}{m_{\rm T}} \\
    \ddot{y} &= \frac{F_{x,{\rm F}} \sin \left( \psi + \delta \right)+ F_{x,{\rm R}} \sin \psi + F_{y,{\rm F}} \cos \left( \psi + \delta \right) + F_{y,{\rm R}} \cos \psi}{m_{\rm T}} \\
    \ddot{\psi} &= \frac{F_{x,{\rm F}} a \sin \delta  + F_{y,{\rm F}} a \cos \delta - F_{y,{\rm R}} b}{I_{\rm T}}.

State Equations
--------------------------------------------------------------------------------

The chosen state vector is

.. math::

    {\rm z}_1 &= x \\
    {\rm z}_2 &= y \\
    {\rm z}_3 &= \psi \\
    {\rm z}_4 &= \dot{x} \\
    {\rm z}_5 &= \dot{y} \\
    {\rm z}_6 &= \dot{\psi}

Thus, considering the equation of motion in equation :eq:`simpleVehicleEquationOfMotion`, the state equation is given by

.. math::

    \dot{{\rm z}}_1 &= {\rm z}_4 \\
    \dot{{\rm z}}_2 &= {\rm z}_5 \\
    \dot{{\rm z}}_3 &= {\rm z}_6 \\
    \dot{{\rm z}}_4 &= \frac{F_{x,{\rm F}} \cos \left( {\rm z}_3 + \delta \right) + F_{x,{\rm R}} \cos {\rm z}_3 - F_{y,{\rm F}} \sin \left( {\rm z}_3 + \delta \right) - F_{y,{\rm R}} \sin {\rm z}_3}{m_{\rm T}} \\
    \dot{{\rm z}}_5 &= \frac{F_{x,{\rm F}} \sin \left( {\rm z}_3 + \delta \right)+ F_{x,{\rm R}} \sin {\rm z}_3 + F_{y,{\rm F}} \cos \left( {\rm z}_3 + \delta \right) + F_{y,{\rm R}} \cos {\rm z}_3}{m_{\rm T}} \\
    \dot{{\rm z}}_6 &= \frac{F_{x,{\rm F}} a \sin \delta  + F_{y,{\rm F}} a \cos \delta - F_{y,{\rm R}} b}{I_{\rm T}}

With the slip angles

.. math::

    \alpha_{\rm F} &= \arctan \left( \frac{{\rm z}_5 + a {\rm z}_6 \cos {\rm z}_3}{ {\rm z}_4 - a {\rm z}_6 \sin {\rm z}_3} \right) - \left( \delta + {\rm z}_3 \right) \\
    \alpha_{\rm R} &= \arctan \left( \frac{{\rm z}_5 - b {\rm z}_6 \cos {\rm z}_3}{ {\rm z}_4 + b {\rm z}_6 \sin {\rm z}_3} \right) - {\rm z}_3,

see equation :eq:`simpleVehicleSlipAngle`.

However, in many occasions it is more convenint to use the states :math:`v_{\rm T}` e :math:`\alpha_{\rm T}` instead of :math:`\dot{x}` e :math:`\dot{y}`.

The relation between this pair of states is given by

.. math::
    :label: simpleVehicleStateSubstitution

    \dot{x} &= v_{\rm T} \cos \left( \psi + \alpha_{\rm T} \right) \\
    \dot{y} &= v_{\rm T} \sin \left( \psi + \alpha_{\rm T} \right).

The time derivative o equation :eq:`simpleVehicleStateSubstitution` is given by

.. math::
    :label: simpleVehicleStateDiffSubstitution

    \ddot{x} &= \dot{v}_{\rm T} \cos \left( \psi + \alpha_{\rm T} \right) - v_{\rm T} \left( \dot{\psi} + \dot{\alpha}_{\rm T} \right) \sin \left( \psi + \alpha_{\rm T} \right) \\
    \ddot{y} &= \dot{v}_{\rm T} \sin \left( \psi + \alpha_{\rm T} \right) + v_{\rm T} \left( \dot{\psi} + \dot{\alpha}_{\rm T} \right) \cos \left( \psi + \alpha_{\rm T} \right).

Substituting equation :eq:`simpleVehicleStateDiffSubstitution` in the equations of motion :eq:`simpleVehicleEquationOfMotion` we have

.. math::

    \dot{v}_{\rm T} \cos \left( \psi + \alpha_{\rm T} \right) - v_{\rm T} \left( \dot{\psi} + \dot{\alpha}_{\rm T} \right) \sin \left( \psi + \alpha_{\rm T} \right) &= \frac{F_{x,{\rm F}} \cos \left( \psi + \delta \right) + F_{x,{\rm R}} \cos \psi - F_{y,{\rm F}} \sin \left( \psi + \delta \right) - F_{y,{\rm R}} \sin \psi}{m_{\rm T}} \\
    \dot{v}_{\rm T} \sin \left( \psi + \alpha_{\rm T} \right) + v_{\rm T} \left( \dot{\psi} + \dot{\alpha}_{\rm T} \right) \cos \left( \psi + \alpha_{\rm T} \right) &= \frac{F_{x,{\rm F}} \sin \left( \psi + \delta \right)+ F_{x,{\rm R}} \sin \psi + F_{y,{\rm F}} \cos \left( \psi + \delta \right) + F_{y,{\rm R}} \cos \psi}{m_{\rm T}} \\
    \ddot{\psi} &= \frac{F_{x,{\rm F}} a \sin \delta  + F_{y,{\rm F}} a \cos \delta - F_{y,{\rm R}} b}{I_{\rm T}}.

Manipulating and simplifying

.. math::

    \dot{v}_{\rm T} &= \frac{F_{x,{\rm F}} \cos \left( \alpha_{\rm T} - \delta \right) + F_{x,{\rm R}} \cos \alpha_{\rm T} + F_{y,{\rm F}} \sin \left( \alpha_{\rm T} - \delta \right) + F_{y,{\rm R}} \sin \alpha_{\rm T}}{m_{\rm T}} \\
    \dot{\alpha}_{\rm T} &=  \frac{- F_{x,{\rm F}} \sin \left( \alpha_{\rm T} - \delta \right) - F_{x,{\rm R}} \sin \alpha_{\rm T} + F_{y,{\rm F}} \cos \left( \alpha_{\rm T} - \delta \right) + F_{y,{\rm R}} \cos \alpha_{\rm T} - m_{\rm T} v \dot{\psi}}{m_{\rm T} v_{\rm T}} \\
    \ddot{\psi} &= \frac{F_{x,{\rm F}} a \sin \delta  + F_{y,{\rm F}} a \cos \delta - F_{y,{\rm R}} b}{I_{\rm T}}.

The slip angles become

.. math::

    \alpha_{\rm F} &= \arctan \left( \frac{v_{\rm T} \sin \left( \psi + \alpha_{\rm T} \right) + a \dot{\psi} \cos \psi}{ v_{\rm T} \cos \left( \psi + \alpha_{\rm T} \right) - a \dot{\psi} \sin \psi} \right) - \left( \delta + \psi \right) \\
    \alpha_{\rm R} &= \arctan \left( \frac{v_{\rm T} \sin \left( \psi + \alpha_{\rm T} \right) - b \dot{\psi} \cos \psi}{ v_{\rm T} \cos \left( \psi + \alpha_{\rm T} \right) + b \dot{\psi} \sin \psi} \right) - \psi.

Simplifying

.. math::
    :label: simpleVehicleSlipAngleSimple

    \alpha_{\rm F} &= \arctan \left( \frac{v_{\rm T} \sin \alpha_{\rm T} + a \dot{\psi}}{ v_{\rm T} \cos \alpha_{\rm T}} \right) - \delta \\
    \alpha_{\rm R} &= \arctan \left( \frac{v_{\rm T} \sin \alpha_{\rm T} - b \dot{\psi}}{ v_{\rm T} \cos \alpha_{\rm T}} \right)

Therefore, the new set of states is

.. math::

    {\rm x}_1 &= x \\
    {\rm x}_2 &= y \\
    {\rm x}_3 &= \psi \\
    {\rm x}_4 &= v_{\rm T} \\
    {\rm x}_5 &= \alpha_{\rm T} \\
    {\rm x}_6 &= \dot{\psi}

and the state equation is

.. math::
    :label: stateEquationX

    \dot{{\rm x}}_1 &= {\rm x}_4 \cos \left( {\rm x}_3 + {\rm x}_5 \right) \\
    \dot{{\rm x}}_2 &= {\rm x}_4 \sin \left( {\rm x}_3 + {\rm x}_5 \right) \\
    \dot{{\rm x}}_3 &= {\rm x}_6 \\
    \dot{{\rm x}}_4 &= \frac{F_{x,{\rm F}} \cos \left( {\rm x}_5 - \delta \right) + F_{x,{\rm R}} \cos {\rm x}_5 + F_{y,{\rm F}} \sin \left( {\rm x}_5 - \delta \right) + F_{y,{\rm R}} \sin {\rm x}_5}{m_{\rm T}} \\
    \dot{{\rm x}}_5 &= \frac{- F_{x,{\rm F}} \sin \left( {\rm x}_5 - \delta \right) - F_{x,{\rm R}} \sin {\rm x}_5 + F_{y,{\rm F}} \cos \left( {\rm x}_5 - \delta \right) + F_{y,{\rm R}} \cos \alpha_{\rm T} - m_{\rm T} {\rm x}_4 {\rm x}_6}{m_{\rm T} {\rm x}_4} \\
    \dot{{\rm x}}_6 &= \frac{F_{x,{\rm F}} a \sin \delta  + F_{y,{\rm F}} a \cos \delta - F_{y,{\rm R}} b}{I_{\rm T}}

The nonlinear model of the package uses the equations :eq:`simpleVehicleSlipAngleSimple` and :eq:`stateEquationX`

Linearization
--------------------------------------------------------------------------------

The state equation :eq:`stateEquationX` describes the nonlinear version of the model. It can be written as

.. math:: \dot{{\bf x}} = {\bf f} \left( {\bf x}, {\bf u} \right),

where the state vector is given by

.. math:: {\bf x} = \left[ \begin{array}{c} {\rm x}_1 \\ {\rm x}_2 \\ {\rm x}_3 \\ {\rm x}_4 \\ {\rm x}_5 \\ {\rm x}_6 \end{array} \right] = \left[ \begin{array}{c} x \\ y \\ \psi \\ v_{\rm T} \\ \alpha_{\rm T} \\ \dot{\psi} \end{array} \right],

the input vector is

.. math:: {\bf u} = \left[ \begin{array}{c} \delta \\ F_{x,{\rm F}} \\ F_{x,{\rm R}} \\ F_{y,{\rm F}} \\ F_{y,{\rm R}} \end{array} \right]

and the vector function :math:`{\bf f}` is

.. math:: {\bf f} = \left[ \begin{array}{c} {\rm f}_1 \\ {\rm f}_2 \\ {\rm f}_3 \\ {\rm f}_4 \\ {\rm f}_5 \\ {\rm f}_6 \end{array} \right] = \left[ \begin{array}{c} \dot{{\rm x}}_1 \\ \dot{{\rm x}}_2 \\ \dot{{\rm x}}_3 \\ \dot{{\rm x}}_4 \\ \dot{{\rm x}}_5 \\ \dot{{\rm x}}_6 \end{array} \right].

The linearization can be made choosing an operating point that represents the vehicle traveling in a straight line with a constant forward velocity

A linearização deste sistema pode ser feita para um veículo se movimentando em linha reta com uma determinada velocidade :math:`v_{{\rm T},0} > 0`. In this case, the operating point of each state are

.. math::

    {\rm x}_{1,op} &= x_{op} \\
    {\rm x}_{2,op} &= y_{op} \\
    {\rm x}_{3,op} &= \psi_{op} = 0 \\
    {\rm x}_{4,op} &= v_{{\rm T},op} = v_{{\rm T},0} \\
    {\rm x}_{5,op} &= \alpha_{{\rm T},op} = 0 \\
    {\rm x}_{6,op} &= \dot{\psi}_{op} = 0,

As we are going to see below, the operating point of the states :math:`x` e :math:`y` do not have to be defined. They do not influence the dynamics of the linear system.

The vector of the states operating points is

.. math:: {\bf x}_{op} = \left[ \begin{array}{c} {\rm x}_{1,op} \\ {\rm x}_{2,op} \\ {\rm x}_{3,op} \\ {\rm x}_{4,op} \\ {\rm x}_{5,op} \\ {\rm x}_{6,op} \end{array} \right].

The operating point of the inputs is

.. math::

    \delta_{op} &= 0 \\
    F_{x,{\rm F},op} &= 0 \\
    F_{x,{\rm R},op} &= 0 \\
    F_{y,{\rm F},op} &= 0 \\
    F_{y,{\rm R},op} &= 0.

The vector of the input operating points is

.. math:: {\bf u}_{op} = \left[ \begin{array}{c} \delta_{op} \\ F_{x,{\rm F},op} \\ F_{x,{\rm R},op} \\ F_{y,{\rm F},op} \\ F_{y,{\rm R},op} \end{array} \right].

Expanding the system in a Taylor series and neglecting the higher order terms, we have

.. math:: {\bf f}_{lin}\left( {\bf x}, {\bf u} \right) = {\bf f} \left( {\bf x}_{op}, {\bf u}_{op} \right) + \nabla{\bf f}\left( {\bf x}_{op}, {\bf u}_{op} \right) \left[ \begin{array}{c} {\bf x} - {\bf x}_{op} \\ {\bf u} - {\bf u}_{op} \end{array} \right].
    :label: simpleVehicleTaylorSeries

where

.. math:: {\bf f} \left( {\bf x}_{op}, {\bf u}_{op} \right) = \left[ \begin{array}{c} 0 \\ 0 \\ 0 \\ v_{{\rm T},0} \\0 \\ 0 \\ 0 \\ 0 \\ 0 \\ 0 \\ 0  \end{array}\right].
    :label: functionOP

The gradient of :math:`{\bf f}` is

.. math:: \nabla {\bf f} = \left[ \begin{array}{ccccccccccc} \frac{ \partial f_1 }{ \partial x } & \frac{ \partial f_1 }{ \partial y } & \frac{ \partial f_1 }{ \partial \psi } & \frac{ \partial f_1 }{ \partial v } & \frac{ \partial f_1 }{ \partial \alpha_{\rm T} } & \frac{ \partial f_1 }{ \partial \dot{\psi} } & \frac{ \partial f_1 }{ \partial \delta } & \frac{ \partial f_1 }{ \partial F_{x, {\rm F}} } & \frac{ \partial f_1 }{ \partial F_{x, {\rm R}} } & \frac{\partial f_1}{F_{y,{\rm F}}} & \frac{\partial f_1}{\partial F_{y,{\rm R}}}  \\ \frac{ \partial f_2 }{ \partial x } & \frac{ \partial f_2 }{ \partial y } & \frac{ \partial f_2 }{ \partial \psi } & \frac{ \partial f_2 }{ \partial v } & \frac{ \partial f_2 }{ \partial \alpha_{\rm T} } & \frac{ \partial f_2 }{ \partial \dot{\psi} } & \frac{ \partial f_2 }{ \partial \delta } & \frac{ \partial f_2 }{ \partial F_{x, {\rm F}} } & \frac{ \partial f_2 }{ \partial F_{x, {\rm R}} } & \frac{\partial f_2}{F_{y,{\rm F}}} & \frac{\partial f_2}{\partial F_{y,{\rm R}}}  \\ \frac{ \partial f_3 }{ \partial x } & \frac{ \partial f_3 }{ \partial y } & \frac{ \partial f_3 }{ \partial \psi } & \frac{ \partial f_3 }{ \partial v } & \frac{ \partial f_3 }{ \partial \alpha_{\rm T} } & \frac{ \partial f_3 }{ \partial \dot{\psi} } & \frac{ \partial f_3 }{ \partial \delta } & \frac{ \partial f_3 }{ \partial F_{x, {\rm F}} } & \frac{ \partial f_3 }{ \partial F_{x, {\rm R}} } & \frac{\partial f_3}{F_{y,{\rm F}}} & \frac{\partial f_3}{\partial F_{y,{\rm R}}}  \\ \frac{ \partial f_4 }{ \partial x } & \frac{ \partial f_4 }{ \partial y } & \frac{ \partial f_4 }{ \partial \psi } & \frac{ \partial f_4 }{ \partial v } & \frac{ \partial f_4 }{ \partial \alpha_{\rm T} } & \frac{ \partial f_4 }{ \partial \dot{\psi} } & \frac{ \partial f_4 }{ \partial \delta } & \frac{ \partial f_4 }{ \partial F_{x, {\rm F}} } & \frac{ \partial f_4 }{ \partial F_{x, {\rm R}} } & \frac{\partial f_4}{F_{y,{\rm F}}} & \frac{\partial f_4}{\partial F_{y,{\rm R}}}  \\ \frac{ \partial f_5 }{ \partial x } & \frac{ \partial f_5 }{ \partial y } & \frac{ \partial f_5 }{ \partial \psi } & \frac{ \partial f_5 }{ \partial v } & \frac{ \partial f_5 }{ \partial \alpha_{\rm T} } & \frac{ \partial f_5 }{ \partial \dot{\psi} } & \frac{ \partial f_5 }{ \partial \delta } & \frac{ \partial f_5 }{ \partial F_{x, {\rm F}} } & \frac{ \partial f_5 }{ \partial F_{x, {\rm R}} } & \frac{\partial f_5}{F_{y,{\rm F}}} & \frac{\partial f_5}{\partial F_{y,{\rm R}}}  \\ \frac{ \partial f_6 }{ \partial x } & \frac{ \partial f_6 }{ \partial y } & \frac{ \partial f_6 }{ \partial \psi } & \frac{ \partial f_6 }{ \partial v } & \frac{ \partial f_6 }{ \partial \alpha_{\rm T} } & \frac{ \partial f_6 }{ \partial \dot{\psi} } & \frac{ \partial f_6 }{ \partial \delta } & \frac{ \partial f_6 }{ \partial F_{x, {\rm F}} } & \frac{ \partial f_6 }{ \partial F_{x, {\rm R}} } & \frac{\partial f_6}{F_{y,{\rm F}}} & \frac{\partial f_6}{\partial F_{y,{\rm R}}} \end{array} \right].

Calculating the partial derivatives,

.. math:: \nabla {\bf f}\left( {\bf x}_{op}, {\bf u}_{op} \right) = \left[ \begin{array}{ccccccccccc} 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & v_{{\rm T},0} & 0 & v_{{\rm T},0} & 0 & 0 & 0 & 0 & 0 & 0\\ 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 & 0 & \frac{1}{m_{\rm T}} & \frac{1}{m_{\rm T}} & 0  & 0 \\ 0 & 0 & 0 & 0 & 0 & -1 & 0 & 0 & 0 & \frac{1}{m_{\rm T} v_{{\rm T},0}} & \frac{1}{m_{\rm T} v_{{\rm T},0}} \\ 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & \frac{a}{I_{{\rm T}}} & - \frac{b}{I_{{\rm T}}} \end{array} \right]
    :label: jacobian

Substituting equations :eq:`functionOP` and :eq:`jacobian` in :eq:`simpleVehicleTaylorSeries` the linearized system becomes

.. math::
    :label: linearModel

    f_{1,lin} &= \dot{x} = v_{\rm T}  \\
    f_{2,lin} &= \dot{y} = v_{{\rm T},0} \left( \psi + \alpha_{{\rm T}}\right) \\
    f_{3,lin} &= \dot{\psi} = \dot{\psi} \\
    f_{4,lin} &= \dot{v}_{\rm T} = \frac{F_{x,{\rm F}} + F_{x,{\rm R}}}{m_{\rm T}} \\
    f_{5,lin} &= \dot{\alpha}_{\rm T} = \frac{F_{y,{\rm F}} + F_{y,{\rm R}}}{m_{\rm T} v_{{\rm T},0}} - \dot{\psi} \\
    f_{6,lin} &= \ddot{\psi} = \frac{a F_{y,{\rm F}} -  b F_{y,{\rm R}}}{I_{\rm T}}

It is important to note that if the longitudinal forces of the tire, :math:`F_{x,{\rm F}}` and :math:`F_{x,{\rm R}}`, are zero, the state :math:`v_{\rm T}` is constant.

In the same operating point, the linear slip angles from equation :eq:`simpleVehicleSlipAngleSimple` become

.. math::
    :label: linearSlipAngles

    \alpha_{{\rm F},lin} &= \alpha_{{\rm T}} + \frac{a}{v_{{\rm T},0}} \dot{\psi} - \delta \\
    \alpha_{{\rm F},lin} &= \alpha_{{\rm T}} - \frac{b}{v_{{\rm T},0}} \dot{\psi}.

The linear model of the package uses equations :eq:`linearModel` and :eq:`linearSlipAngles`.

Finally, a linear tire model

.. math::
    :label: simpleVehicleLinearTire

    F_{y,{\rm F}} &= - K_{\rm F} \alpha_{\rm F} = - K_{\rm F} \alpha_{{\rm T}} - \frac{a K_{\rm F}}{v_{{\rm T},0}} \dot{\psi} + K_{\rm F} \delta \\
    F_{y,{\rm R}} &= - K_{\rm R} \alpha_{\rm R} =  - K_{\rm R} \alpha_{{\rm T}} + \frac{b K_{\rm R}}{v_{{\rm T},0}} \dot{\psi}

can be used in the linearized equations :eq:`linearModel`. In this case

.. math::

    f_{1,lin} &= \dot{x} = v_{\rm T} \\
    f_{2,lin} &= \dot{y} = v_{{\rm T},0} \left( \psi + \alpha_{{\rm T}}\right) \\
    f_{3,lin} &= \dot{\psi} = \dot{\psi} \\
    f_{4,lin} &= \dot{v}_{\rm T} = \frac{F_{x,{\rm F}} + F_{x,{\rm R}}}{m_{\rm T}} \\
    f_{5,lin} &= \dot{\alpha}_{{\rm T}} = - \frac{K_{\rm F} + K_{\rm R}}{m_{\rm T} v_{{\rm T},0}} \alpha_{{\rm T}} - \frac{m_{\rm T} v_{{\rm T},0} + \frac{a K_{\rm F} - b K_{\rm R}}{v_{{\rm T},0}}}{m_{\rm T} v_{{\rm T},0}} \dot{\psi} + \frac{K_{\rm F}}{m_{\rm T} v_{{\rm T},0}} \delta \\
    f_{6,lin} &= \ddot{\psi} = - \frac{a K_{\rm F} - b K_{\rm R}}{I_{\rm T}} \alpha_{{\rm T}} - \frac{a^2 K_{\rm F} + b^2 K_{\rm R}}{I_{\rm T}  v_{{\rm T},0}} \dot{\psi} + \frac{a K_{\rm F}}{I_{\rm T}} \delta

In the matrix form

.. math:: \dot{{\bf x}} = {\bf A} {\bf x} + {\bf B} {\bf \hat{u}}

or

.. math:: \left[ \begin{array}{c} \dot{x} \\ \dot{y} \\ \dot{\psi} \\ \dot{v}_{\rm T} \\ \dot{\alpha}_{\rm T} \\ \ddot{\psi} \end{array} \right] = \left[ \begin{array}{cccccc} 0 & 0 & 0 & 1 & 0 & 0 \\ 0 & 0 & v_{{\rm T},0} & 0 & v_{{\rm T},0} & 0 \\ 0 & 0 & 0 & 0 & 0 & 1 \\ 0 & 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & - \frac{K_{\rm F} + K_{\rm R}}{m_{\rm T} v_{{\rm T},0}} & - \frac{m_{\rm T} v_{{\rm T},0} + \frac{a K_{\rm F} - b K_{\rm R}}{v_{{\rm T},0}}}{m_{\rm T} v_{{\rm T},0}} \\ 0 & 0 & 0 & 0 & - \frac{a K_{\rm F} - b K_{\rm R}}{I_{\rm T}} & - \frac{a^2 K_{\rm F} + b^2 K_{\rm R}}{I_{\rm T}  v_{{\rm T},0}} \end{array} \right] \left[ \begin{array}{c} x \\ y \\ \psi \\ v_{\rm T} \\ \alpha_{\rm T} \\ \dot{\psi} \end{array} \right] + \left[ \begin{array}{ccccc} 0 & 0 & 0 & \\ 0 & 0 & 0 & \\ 0 & 0 & 0 & \\ 0 & \frac{1}{m_{\rm T}} & \frac{1}{m_{\rm T}} & \\ \frac{K_{\rm F}}{m_{\rm T} v_{{\rm T},0}} & 0 & 0 & \\  \frac{a K_{\rm F}}{I_{\rm T}} & 0 & 0 \end{array} \right] \left[ \begin{array}{c} \delta \\ F_{x,{\rm F}} \\ F_{x,{\rm R}} \end{array} \right]
    :label: linearModelMatrix

A popular model used in the literature considers just the last two rows of the model described in :eq:`linearModelMatrix` with only the steering angle as input. This is

.. math:: \left[ \begin{array}{c} \dot{\alpha}_{\rm T} \\ \ddot{\psi} \end{array} \right] = \left[ \begin{array}{cccccc} - \frac{K_{\rm F} + K_{\rm R}}{m_{\rm T} v_{{\rm T},0}} & - \frac{m_{\rm T} v_{{\rm T},0} + \frac{a K_{\rm F} - b K_{\rm R}}{v_{{\rm T},0}}}{m_{\rm T} v_{{\rm T},0}} \\ - \frac{a K_{\rm F} - b K_{\rm R}}{I_{\rm T}} & - \frac{a^2 K_{\rm F} + b^2 K_{\rm R}}{I_{\rm T}  v_{{\rm T},0}} \end{array} \right] \left[ \begin{array}{c} \alpha_{\rm T} \\ \dot{\psi} \end{array} \right] + \left[ \begin{array}{ccccc} \frac{K_{\rm F}}{m_{\rm T} v_{{\rm T},0}} \\  \frac{a K_{\rm F}}{I_{\rm T}} \end{array} \right] \left[ \begin{array}{c} \delta \end{array} \right]

Simple vehicle 4 DOF
================================================================================

The linear version of this model is described by equations ?? and the nonlinear version of this model is described by equations ??.

Nonlinear bicycle model nonlinear with 4 degrees of freedom.

Description

The center of gravity of the vehicle is located at the point :math:`T`. The front and rear axles are located at the points :math:`F` and :math:`R`, respectively. The constant :math:`a` measures the distance of point :math:`F` to :math:`T` and :math:`b` the distance of point :math:`T` to :math:`R`. The angles :math:`\alpha_F` e :math:`\alpha_R` are the front and rear slip angles, respectively. :math:`\alpha_T` is the vehicle side slip angle and :math:`\psi` is the vehicle yaw angle. :math:`\delta` is the steering angle.

.. figure:: https://andresmendes.github.io/openvd/illustrations/modelSimple4DOF.svg
