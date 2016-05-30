clear all                   % Clear workspace
close all                   % Closing figures
clc                         % Clear command window

% Choosing tire
TireModel = VehicleDynamicsLateral.TirePacejka();
% Choosing vehicle
System = VehicleDynamicsLateral.VehicleArticulatedNonlinear();
System.tire = TireModel;
% Choosing simulation
T = 6;                      % Total simulation time [s]
resol = 50;                 % Resolution
TSPAN = 0:T/resol:T;        % Time span [s]
simulator = VehicleDynamicsLateral.Simulator(System, TSPAN);

% Simulation
simulator.Simulate();

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
g.TractorColor = 'c';
g.SemitrailerColor = 'm';
g.Frame(0);
g.Animation(0);
