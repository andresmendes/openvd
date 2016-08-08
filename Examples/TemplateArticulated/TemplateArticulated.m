%% Template Articulated
% This template show how to simulate an articulated vehicle and plot the results.
%
%%
%

clear all                   % Clear workspace
close all                   % Closing figures
clc                         % Clear command window

%%
%
import VehicleDynamicsLateral.*

% Choosing tire
TireModel = TirePacejka();
TireModel.a0 = 1;
TireModel.a1 = 2;
TireModel.a2 = 700;
TireModel.a3 = 5000;
TireModel.a4 = 80;
TireModel.a5 = 0;
TireModel.a6 = 0;
TireModel.a7 = 0.6;

% Choosing vehicle
System = VehicleArticulatedNonlinear();
System.tire = TireModel;
% Choosing simulation
T = 7;                      % Total simulation time [s]
resol = 50;                 % Resolution
TSPAN = 0:T/resol:T;        % Time span [s]
simulator = Simulator(System, TSPAN);

simulator.ALPHAT0 = 0.3;              % Initial tractor side slip angle [rad]
simulator.dPSI0 = 0.25;               % Initial tractor yaw rate [rad/s]
simulator.dPHI0 = 0.25;        % Initial articulation rate [rad/s]


% Simulation
simulator.Simulate();

%% Results
%

g = Graphics(simulator);
g.TractorColor = 'r';
g.SemitrailerColor = 'g';
g.Frame();
g.Animation();

%%
%
% <<illustrations/AnimationArticulated.gif>>
%
%% See Also
%
% <index.html Index> | <TemplateSimple.html TemplateSimple>
%
