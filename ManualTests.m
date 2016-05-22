clear all                   % Clear workspace
close all                   % Closing figures
clc                         % Clear command window

T = 6;                      % Total simulation time [s]
resol = 50;                 % Resolution
TSPAN = 0:T/resol:T;        % Time span [s]

%% Tire parameters
% Chosen tire: <TirePacejka.html TirePacejka.m>.
%

% TireModel = VehicleDynamicsLateral.TireLinear;
%
% %% Vehicle parameters
% % Chosen Vehicle: <VehicleSimpleNonlinear.html VehicleSimpleNonlinear.m>.
%
% System = VehicleDynamicsLateral.VehicleSimpleNonlinear();
% System.tire = TireModel;
% simulator = VehicleDynamicsLateral.Simulator(System, TSPAN);
% simulator.Simulate();

% % Retrieving states
% XT = simulator.XT;
% YT = simulator.YT;
% PSI = simulator.PSI;
% VEL = simulator.VEL;
% ALPHAT = simulator.ALPHAT;
% dPSI = simulator.dPSI;

% g = VehicleDynamicsLateral.Graphics(simulator);
% g.Frame(0);
% g.Animation(0);

truck = VehicleDynamicsLateral.VehicleArticulatedNonlinear;
truck.tire = VehicleDynamicsLateral.TireLinear;
%
simulator = VehicleDynamicsLateral.Simulator(truck, TSPAN);
simulator.Simulate();
%
% Retrieving states
XT = simulator.XT;
YT = simulator.YT;
PSI = simulator.PSI;
PHI = simulator.PHI;
VEL = simulator.VEL;
ALPHAT = simulator.ALPHAT;
dPSI = simulator.dPSI;
dPHI = simulator.dPHI;

g = VehicleDynamicsLateral.Graphics(simulator);
g.Frame(0);
% % g.Animation(0);
