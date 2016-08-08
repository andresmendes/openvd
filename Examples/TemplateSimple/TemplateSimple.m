%% Template Simple
% This template show how to simulate a simple vehicle and plot the results.
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
% Choosing vehicle
System = VehicleSimpleNonlinear();
% Defining vehicle parameters
System.mF0 = 700;
System.mR0 = 600;
System.IT = 10000;
System.lT = 3.5;
System.nF = 2;
System.nR = 2;
System.wT = 2;
System.muy = .8;
System.deltaf = 0;

System.tire = TireModel;
% Choosing simulation
T = 6;                      % Total simulation time [s]
resol = 50;                 % Resolution
TSPAN = 0:T/resol:T;        % Time span [s]
simulator = Simulator(System, TSPAN);

% Changing initial conditions
simulator.ALPHAT0 = -0.2;             % Initial side slip angle [rad]
simulator.dPSI0 = 0.7;                % Initial yaw rate [rad/s]


% Simulation
simulator.Simulate();

%% Results

% Retrieving states
XT = simulator.XT;
YT = simulator.YT;
PSI = simulator.PSI;
VEL = simulator.VEL;
ALPHAT = simulator.ALPHAT;
dPSI = simulator.dPSI;

figure(1)
plot(TSPAN,XT)
xlabel('time [s]')
ylabel('Distance in the x direction [m]')

figure(2)
plot(TSPAN,YT)
xlabel('time [s]')
ylabel('Distance in the y direction [m]')

figure(3)
plot(TSPAN,PSI)
xlabel('time [s]')
ylabel('Yaw angle [rad]')

figure(4)
plot(TSPAN,VEL)
xlabel('time [s]')
ylabel('Velocity [m/s]')

figure(5)
plot(TSPAN,ALPHAT)
xlabel('time [s]')
ylabel('Vehicle slip angle [rad/s]')

figure(6)
plot(TSPAN,dPSI)
xlabel('time [s]')
ylabel('Yaw rate [rad/s]')

%%
% Frame and animation

g = Graphics(simulator);
g.TractorColor = 'r';

g.Frame();
g.Animation();

%%
% <<illustrations/AnimationSimple.gif>>
%
%% See Also
%
% <index.html Index>
%
