% .. _template-simple-simulink:
%
% Template Simple Simulink
% *******************************************************************************
%
% This template shows how to simulate a simple vehicle in Simulink using a s-function. The graphics are also plotted.
%
% This model uses the s-function SimpleVehicleSFunction.m in Simulink. The package and this s-function must be in Matlab path.
%
% The "SimpleVehicleSimulink.slx" available in the repository ("docs/examples/TemplateSimpleSimulink") is illustrated below:
%
% .. figure:: ../illustrations/misc/SimpleVehicleSimulink.png
%     :align:   center
%
%     Simple vehicle simulink S-Function
%
% It can be seen that the longitudinal forces of the tire are zero for the entire simulation. The steering angle recieve a step input.
%
% Template
% ================================================================================
%
% To simulate the model run
%
% .. code-block:: matlab

sim('SimpleVehicleSimulink');

% Each vehicle state variable goes to a scope. And the output of the model is saved in the workspace.
%
% To generate the graphics, the same model used in :ref:`vehicle-simple-sfunction` must be defined.
%
% .. todo:: Improve generation of graphics. Define parameters once -> Run Simulink -> Retrieve responses -> Graphics
%
% First, define the tire model.
%
% .. code-block:: matlab

TireModel = TirePacejka();          % Choosing tire model

% Defining the tire parameters to match the parameters used in the S-Function.
%
% .. code-block:: matlab

TireModel.a0 = 1;
TireModel.a1 = 0;
TireModel.a2 = 800;
TireModel.a3 = 3000;
TireModel.a4 = 50;
TireModel.a5 = 0;
TireModel.a6 = 0;
TireModel.a7 = -1;
TireModel.a8 = 0;
TireModel.a9 = 0;
TireModel.a10 = 0;
TireModel.a11 = 0;
TireModel.a12 = 0;
TireModel.a13 = 0;

% The vehicle model is defined as
%
% .. code-block:: matlab

VehicleModel = VehicleSimpleNonlinear();

% Defining the vehicle parameters to match the parameters used in the S-Function.
%
% .. code-block:: matlab

VehicleModel.mF0 = 700;
VehicleModel.mR0 = 600;
VehicleModel.IT = 10000;
VehicleModel.lT = 3.5;
VehicleModel.nF = 2;
VehicleModel.nR = 2;
VehicleModel.wT = 2;
VehicleModel.muy = .8;
VehicleModel.tire = TireModel;

% Defining the simulation object.
%
% .. code-block:: matlab

simulator = Simulator(VehicleModel, tout);

% Retrieving state responses from Simulink model
%
% .. code-block:: matlab

simulator.XT = simout.Data(:,1);
simulator.YT = simout.Data(:,2);
simulator.PSI = simout.Data(:,3);
simulator.VEL = simout.Data(:,4);
simulator.ALPHAT = simout.Data(:,5);
simulator.dPSI = simout.Data(:,6);

% Frame and animation can be generated defining a graphic object (:mod:`Graphics`). The only argument of the graphic object is the simulator object after the simulation.
%
% .. code-block:: matlab

g = Graphics(simulator);

% To change the color of the vehicle run
%
% .. code-block:: matlab

g.TractorColor = 'r';

% After that, just run
%
% .. code-block:: matlab

g.Frame();
g.Animation();

% Both graphics feature can be seen below.
%
% .. figure:: ../illustrations/frame/TemplateSimpleSimulinkFrame.jpeg
%     :align:   center
%     :width: 60%
%
%     Frame of the simple vehicle model in Simulink.
%
% .. todo:: Include Template Articulated Simulink Animation
%
% As expected the vehicle starts traveling in a straight line and starts a turn at \(t = 1 \, s\) because of the step function.
%
% .. _vehicle-simple-sfunction:
%
% Simple Vehicle S-Function
% ================================================================================
%
% The simple vehicle S-Function is depicted below.
%
% .. literalinclude:: ../../docs/examples/TemplateSimpleSimulink/SimpleVehicleSFunction.m
