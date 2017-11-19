.. _sinusoidal-steering:

Sinusoidal Steering
********************************************************************************

Simulation of a simple vehicle with sinusoidal steering actuation.

First, we choose the tire and vehicle models.

.. code-block:: matlab
    
    TireModel = TirePacejka();          % Choosing tire
    System = VehicleSimpleNonlinear();  % Choosing vehicle
    
The system is completely defined once we attribute the chosen tire model to the vehicle object.

.. code-block:: matlab
    
    System.tire = TireModel;
    
The friction coefficient between the tire and the road can be set as

.. code-block:: matlab
    
    System.muy = 1.0;
    
After this, we define the simulation time span

.. code-block:: matlab
    
    T = 4;                              % Total simulation time [s]
    resol = 50;                         % Resolution
    TSPAN = 0:T/resol:T;                % Time span [s]
    
We want to simulate the model with an open loop sinusoidal steering input. So, since we have the time span, we can create an array with the value of the steering angle for each simulated instants.

.. code-block:: matlab
    
    System.deltaf = 1*pi/180*sin(T^-1*2*pi*TSPAN);
    
The steering input over time can be plotted with

.. code-block:: matlab
    
    f1 = figure(1);
    grid on ; box on;
    plot(TSPAN,180/pi*System.deltaf)
    xlabel('time [s]')
    ylabel('Steering angle [deg]')
    
The resulting plot is illustrated below

.. figure::  ../illustrations/plot/SinusoidalSteeringFig1.svg
    :align:   center
    :width: 40%

    Open loop steering angle over time

To define a simulation object (:mod:`Simulator`) the arguments must be the vehicle object and the time span. This is,

.. code-block:: matlab
    
    simulator = Simulator(System, TSPAN);
    
No simulation parameters need to be altered. So, we have everything needed to run the simulation. For this, we use

.. code-block:: matlab
    
    simulator.Simulate();
    
The resulting time response of each state is stored in separate variables:

.. code-block:: matlab
    
    XT = simulator.XT;
    YT = simulator.YT;
    PSI = simulator.PSI;
    VEL = simulator.VEL;
    ALPHAT = simulator.ALPHAT;
    dPSI = simulator.dPSI;
    
Frame and animation can be generated defining a graphic object (:mod:`Graphics`). The only argument of the graphic object is the simulator object after the simulation.

.. code-block:: matlab
    
    g = Graphics(simulator);
    
To change the color of the vehicle run

.. code-block:: matlab
    
    g.TractorColor = 'r';
    
To generate the frame/animation with a different horizontal and vertical scale run

.. code-block:: matlab
    
    g.Frame('scalefig',3);
    g.Animation('scalefig',3);
    
Both graphics feature can be seen below.

.. figure::  ../illustrations/frame/SinusoidalSteeringFrame.svg
    :align:   center
    :width: 60%

    Frame of the articulated vehicle model.

.. figure::  ../illustrations/animation/SinusoidalSteeringAnimation.gif
    :align:   center

    Animation of the articulated vehicle model.

