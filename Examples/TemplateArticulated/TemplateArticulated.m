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
% Choosing vehicle
System = VehicleArticulatedNonlinear();
System.tire = TireModel;
% Choosing simulation
T = 6;                      % Total simulation time [s]
resol = 50;                 % Resolution
TSPAN = 0:T/resol:T;        % Time span [s]
simulator = Simulator(System, TSPAN);

% Simulation
simulator.Simulate();

%% Results
%

g = Graphics(simulator);
g.TractorColor = 'c';
g.SemitrailerColor = 'm';
g.Frame('~/Desktop/plot/trajectory');
g.Animation('~/Desktop/animation/animated_trajectory');

%%
%
% <<illustrations/AnimationArticulated.gif>>
%
%% See Also
%
% <index.html Index> | <TemplateSimple.html TemplateSimple>
%
