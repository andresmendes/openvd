clear all                   % Clear workspace
close all                   % Closing figures
clc                         % Clear command window

T = 6;                      % Total simulation time [s]
resol = 50;                 % Resolution
TSPAN = 0:T/resol:T;        % Time span [s]

%% Tire parameters
% Chosen tire: <TirePacejka1989.html TirePacejka1989.m>.
%

TireModel = VehicleDynamicsLateral.TirePacejka1989();

%% Vehicle parameters
% Chosen Vehicle: <VehicleSimpleNonlinear3DOF.html VehicleSimpleNonlinear3DOF.m>.

System = VehicleDynamicsLateral.VehicleSimpleNonlinear3DOF();
System.tire = TireModel;
% Initial conditions
System.dPSI0 = 0.7;                % Initial yaw rate [rad/s]
System.ALPHAT0 = -0.2;             % Initial side slip angle [rad]
System.PSI0 = 0;                   % Initial yaw angle [rad]
System.X0 = 0;                     % Initial CG horizontal position [m]
System.Y0 = 0;                     % Initial CG vertical position [m]
System.V0 = 20;                    % Initial CG velocity [m/s]

simulator = VehicleDynamicsLateral.Simulator(System, TSPAN);
simulator.Simulate();

% Retrieving states
XT = simulator.XT;
YT = simulator.YT;
PSI = simulator.PSI;
VEL = simulator.VEL;
ALPHAT = simulator.ALPHAT;
dPSI = simulator.dPSI;



% truck = VehicleDynamicsLateral.VehicleArticulatedNonlinear4DOF;
% truck.dPSI0 = 0.25;               % Initial tractor yaw rate [rad/s]
% truck.ALPHAT0 = 0.3;              % Initial tractor side slip angle [rad]
% truck.dPHI0 = truck.dPSI0;        % Initial articulation rate [rad/s]
% truck.V0 = 20;                    % Initial tractor CG velocity [m/s]
% truck.PHI0 = 0;                   % Initial articulation angle [rad]
% truck.PSI0 = 0;                   % Initial tractor yaw angle [rad]
% truck.X0 = 0;                     % Initial tractor CG horizontal position [m]
% truck.Y0 = 0;                     % Initial tractor CG vertical position [m]
% truck.tire = VehicleDynamicsLateral.TirePolynomial;
%
% simulator = VehicleDynamicsLateral.Simulator(truck, TSPAN);
% simulator.Simulate();
