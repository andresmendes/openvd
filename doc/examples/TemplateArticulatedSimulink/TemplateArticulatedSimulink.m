% .. _template-articulated-simulink:
%
% Template Articulated Simulink
% ********************************************************************************

% This template shows how to simulate an articulated vehicle in Simulink using a s-function. The graphics are also plotted.
%
% This model uses the s-function ArticulatedVehicleSFunction.m in Simulink. The package and this s-function must be in Matlab path.
%
% The "ArticulatedVehicleSimulink.slx" available in the repository ("examples/TemplateArticulatedSimulink") is illustrated below:
%
% .. figure:: ../illustrations/misc/ArticulatedVehicleSimulink.png
%     :align:   center
%
%     Articulated vehicle simulink S-Function
%
% It can be seen that the longitudinal forces of the tire are zero for the entire simulation. The steering angle recieve a step input.

%
% Template
% ================================================================================
%
% To simulate the model run
%
% .. code-block:: matlab

sim('ArticulatedVehicleSimulink');

% Each vehicle state variable goes to a scope. And the output of the model is saved in the workspace.
%
% To generate the graphics, the same model used in :ref:`vehicle-articulated-sfunction` must be defined.
%
% .. todo:: Improve generation of graphics. Define parameters once -> Run Simulink -> Retrieve responses -> Graphics
%
% First, define the tire model.
%
% .. code-block:: matlab
%
TireModel = TirePacejka();          % Choosing tire model

% Defining the tire parameters to match the parameters used in the S-Function.
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
TireModel.a8 = 0;
TireModel.a9 = 0;
TireModel.a10 = 0;
TireModel.a11 = 0;
TireModel.a12 = 0;
TireModel.a13 = 0;

% The vehicle model is defined as
%
% .. code-block:: matlab

VehicleModel = VehicleArticulatedNonlinear();

% Defining the vehicle parameters to match the parameters used in the S-Function.
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
VehicleModel.muy = 0.8;
VehicleModel.deltaf = 0;
VehicleModel.Fxf = 0;
VehicleModel.Fxr = 0;
VehicleModel.Fxm = 0;
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
simulator.PHI = simout.Data(:,4);
simulator.VEL = simout.Data(:,5);
simulator.ALPHAT = simout.Data(:,6);
simulator.dPSI = simout.Data(:,7);
simulator.dPHI = simout.Data(:,8);

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
% .. figure:: ../illustrations/frame/TemplateArticulatedSimulinkFrame.png
%     :align:   center
%     :width: 60%
%
%     Frame of the simple vehicle model in Simulink.
%
% .. todo:: Include Template Simple Simulink Animation
%
% As expected the vehicle starts traveling in a straight line and starts a turn at \(t = 1 \, s\) because of the step function.
%
% .. _vehicle-articulated-sfunction:
%
% Articulated Vehicle S-Function
% ================================================================================
%
% The articulated vehicle S-Function is depicted below.
%
% .. literalinclude:: ../../docs/examples/TemplateArticulatedSimulink/ArticulatedVehicleSFunction.m
