%% Sinusoidal Steering
% Simulation of a simple vehicle with sinusoidal steering actuation.
%
% <<../illustrations/animation/SinusoidalSteeringAnimation.gif>>
%
%%
%
% Choosing simulation
T = 4;                      % Total simulation time [s]
resol = 50;                 % Resolution
TSPAN = 0:T/resol:T;        % Time span [s]

% Choosing tire
TireModel = TirePacejka();
% Choosing vehicle
System = VehicleSimpleNonlinear();

% Steering angle
System.deltaf = 1*pi/180*sin(T^-1*2*pi*TSPAN);
% Tire model
System.tire = TireModel;
System.muy = 1.0;


simulator = Simulator(System, TSPAN);

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

f1 = figure(1);
grid on ; box on;
plot(TSPAN,180/pi*System.deltaf)
xlabel('time [s]')
ylabel('Steering angle [deg]')

%%
% <<../illustrations/plot/SinusoidalSteeringFig1.svg>>
%

%%
% Frame and animation

g = Graphics(simulator);
g.TractorColor = 'r';

g.Frame('scalefig',3);

%%
% <<../illustrations/frame/SinusoidalSteeringFrame.svg>>
%

g.Animation('scalefig',3);

%%
% <<../illustrations/animation/SinusoidalSteeringAnimation.gif>>
%
%% See Also
%
% <../index.html Home>
%
