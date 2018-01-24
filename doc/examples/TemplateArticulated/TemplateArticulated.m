% .. _template-articulated:
%
% Template Articulated
% ********************************************************************************
%
% This template shows how to simulate an articulated vehicle and plot the results.
%
% In this case, the parameters are defined by the user.
%
% First, we define the Pacejka tire model (:ref:`tire-pacejka`).
%
% .. code-block:: matlab

TireModel = TirePacejka();                      % Choosing tire

% Then, the tire parameters are defined as
%
% .. code-block:: matlab

TireModel.a0 = 1;
TireModel.a1 = 2;
TireModel.a2 = 700;
TireModel.a3 = 5000;
TireModel.a4 = 80;
TireModel.a5 = 0;
TireModel.a6 = 0;
TireModel.a7 = 0.6;

% Check all the tire parameters at :mod:`TirePacejka`.
%
% The nonlinear articulated vehicle model (:ref:`vehicle-articulated-4dof`) is defined as
%
% .. code-block:: matlab

VehicleModel = VehicleArticulatedNonlinear();   % Choosing vehicle

% The vehicle parameters can be changed as
%
% .. code-block:: matlab

VehicleModel.mF0 = 5200;
VehicleModel.mR0 = 2400;
VehicleModel.mF = 6000;
VehicleModel.mR = 10000;
VehicleModel.mM = 17000;
VehicleModel.IT = 46000;
VehicleModel.IS = 450000;
VehicleModel.lT = 3.5;
VehicleModel.lS = 7.7;
VehicleModel.c = -0.3;
VehicleModel.nF = 2;
VehicleModel.nR = 4;
VehicleModel.nM = 8;
VehicleModel.wT = 2.6;
VehicleModel.wS = 2.4;
VehicleModel.muy = 0.3;

% Check all the vehicle parameters at :mod:`VehicleSimpleNonlinear`.
%
% The input variables can be defined as
%
% .. code-block:: matlab

VehicleModel.deltaf = 0;
VehicleModel.Fxf = 0;
VehicleModel.Fxr = 0;
VehicleModel.Fxm = 0;

% When the input variables are defined as a scalar quantity, the attributed value remains the same for the entire simulation span.
%
% The System is completely defined once we attribute the chosen tire model to the vehicle object.
%
% .. code-block:: matlab

VehicleModel.tire = TireModel;

% Choosing simulation time span
%
% .. code-block:: matlab

T = 7;                              % Total simulation time [s]
resol = 50;                         % Resolution
TSPAN = 0:T/resol:T;                % Time span [s]

% To define a simulation object (:mod:`Simulator`) the arguments must be the vehicle object and the time span.
%
% .. code-block:: matlab

simulator = Simulator(VehicleModel, TSPAN);

% The default parameters of the simulation object can be found in :mod:`Simulator`. However, we are interested in changing the initial conditions of the simulation object. This can be done running
%
% .. code-block:: matlab

simulator.ALPHAT0 = 0.3;            % Initial tractor side slip angle [rad]
simulator.dPSI0 = 0.25;             % Initial tractor yaw rate [rad/s]
simulator.dPHI0 = 0.25;             % Initial articulation rate [rad/s]

% Now, we have everything needed to run the simulation. For this, we use
%
% .. code-block:: matlab

simulator.Simulate();

% In this example, the plots of the time response of each state are no presented. For that, see :ref:`template-simple`.
%
% Frame and animation can be generated defining a graphic object (:mod:`Graphics`). The only argument of the graphic object is the simulator object after the simulation.
%
% .. code-block:: matlab

g = Graphics(simulator);

% To change the color of the tractor and the semitrailer run
%
% .. code-block:: matlab

g.TractorColor = 'r';
g.SemitrailerColor = 'g';

% After that, just run
%
% .. code-block:: matlab

g.Frame();
g.Animation();

% Both graphics feature can be seen below.
%
% .. figure::  ../illustrations/frame/TemplateArticulatedFrame.svg
%     :align:   center
%     :width: 60%
%
%     Frame of the articulated vehicle model.
%
% .. figure::  ../illustrations/animation/TemplateArticulatedAnimation.gif
%     :align:   center
%
%     Animation of the articulated vehicle model.
%
