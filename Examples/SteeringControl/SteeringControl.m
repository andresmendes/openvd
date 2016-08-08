%% Steering Control
% Steering Control of Autonomous Vehicles in Obstacle Avoidance Maneuvers.
%
%%
%

clear all                   % Clear workspace
close all                   % Closing figures
clc                         % Clear command window

% Adding package path
addpath('/home/andre/Repos/Vehicle-Dynamics/Vehicle-Dynamics-Lateral/')

%% Vehicle model
% *Bicycle model*
%
% <<illustrations/modelSimple.svg>>
%
% *Nonlinear model*
%
% State vector
%
% $$ {\bf x} = \left[ \begin{array}{c} {\rm x}_1 \\ {\rm x}_2 \\ {\rm x}_3 \\ {\rm x}_4 \\ {\rm x}_5 \\ {\rm x}_6 \end{array} \right] = \left[ \begin{array}{c} x \\ y \\ \psi \\ v_{\rm T} \\ \alpha_{\rm T} \\ \dot{\psi} \end{array} \right] $$
%
% State equations
%
% $$ \dot{{\rm x}}_1 = {\rm x}_4 \cos \left( {\rm x}_3 + {\rm x}_5 \right) $$
%
% $$ \dot{{\rm x}}_2 = {\rm x}_4 \sin \left( {\rm x}_3 + {\rm x}_5 \right) $$
%
% $$ \dot{{\rm x}}_3 = {\rm x}_6 $$
%
% $$ \dot{{\rm x}}_4 = \frac{F_{y,{\rm F}} \sin \left( {\rm x}_5 - \delta \right) + F_{y,{\rm R}} \sin {\rm x}_5}{m_{T}} $$
%
% $$ \dot{{\rm x}}_5 = \frac{F_{y,{\rm F}} \cos \left( {\rm x}_5 - \delta \right) + F_{y,{\rm R}} \cos \alpha_{\rm T} - m_{T} {\rm x}_4 {\rm x}_6}{m_{T} {\rm x}_4} $$
%
% $$ \dot{{\rm x}}_6 = \frac{F_{y,{\rm F}} a \cos \delta - F_{y,{\rm R}} b}{I_{T}} $$
%
% Slip angles
%
% $$ \alpha_{\rm F} = \arctan \left( \frac{v_{\rm T} \sin \alpha_{\rm T} + a \dot{\psi}}{ v_{\rm T} \cos \alpha_{\rm T}} \right) - \delta $$
%
% $$ \alpha_{\rm R} = \arctan \left( \frac{v_{\rm T} \sin \alpha_{\rm T} - b \dot{\psi}}{ v_{\rm T} \cos \alpha_{\rm T}} \right) $$
%
%
% *Linear model*
%
% $$ \dot{x} = v_{\rm T} $$
%
% $$ \dot{y} = v_{{\rm T},0} \left( \psi + \alpha_{{\rm T}}\right) $$
%
% $$ \dot{\psi} = \dot{\psi} $$
%
% $$ \dot{v}_{\rm T} = 0 $$
%
% $$ \dot{\alpha}_{\rm T} = \frac{F_{y,{\rm F}} + F_{y,{\rm R}}}{m_{T} v_{{\rm T},0}} - \dot{\psi} $$
%
% $$ \ddot{\psi} = \frac{a F_{y,{\rm F}} -  b F_{y,{\rm R}}}{I_{T}} $$
%
% Neglecting equations of $x$ and $v_T$
%
%
% $$ \left[ \begin{array}{c} \dot{y} \\ \dot{\psi} \\ \dot{\alpha}_T \\ \ddot{\psi} \end{array} \right] = \left[ \begin{array}{cccc} 0 & v_{T,0} & v_{T,0} & 0 \\ 0 & 0 & 0 & 1 \\ 0 & 0 & -\frac{K_F+K_R}{m_T v_{T,0}} & - \frac{m_T v_{T,0} + \frac{a K_F - b K_R}{v_{T,0}}}{m_T v_{T,0}} \\ 0 & 0 & - \frac{a K_F - b K_R}{I_T} & - \frac{a^2 K_F + b^2 K_R}{I_T v_{T,0}} \end{array} \right] \left[ \begin{array}{c} y \\ \psi \\ \alpha_T \\ \dot{\psi} \end{array} \right] + \left[ \begin{array}{c} 0 \\ 0 \\ \frac{K_F}{m_T v_{T,0}} \\ \frac{a K_F}{I_T}  \end{array} \right] \delta $$
%
% Slip angles
%
% $$ \alpha_{{\rm F},lin} = \alpha_{{\rm T}} + \frac{a}{v_{{\rm T},0}} \dot{\psi} - \delta $$
%
% $$ \alpha_{{\rm F},lin} = \alpha_{{\rm T}} - \frac{b}{v_{{\rm T},0}} \dot{\psi} $$
%
%% Tire model
%
% Typical characteristic curve and slip angle definition
%
% <<illustrations/CurvaCaracteristica.svg>>
%
% *Pacejka*
%
% $$ F_{y} = D \sin \left[ C \arctan{B \alpha - E( B \alpha -\arctan(B \alpha))} \right] $$
%
% *Linear*
%
% $$ F_ y = K \alpha$$
%

deriva = (0:0.1:15)*pi/180;         % ngulo de deriva [rad]

a0 = 1.3;
a1 = 2.014156;
a2 = 710.5013;
a3 = 5226.341;
a4 = 78.87699;
a5 = 0.01078379;
a6 = -0.004759443;
a7 = -1.8572;
a8 = 0;
a9 = 0;
a10 = 0;
a11 = 0;
a12= 0;
a13 = 0;

TirePac = VehicleDynamicsLateral.TirePacejka();

Fz = 4e+03;
camber = 0;
TirePac.a0 = a0;
TirePac.a1 = a1;
TirePac.a2 = a2;
TirePac.a3 = a3;
TirePac.a4 = a4;
TirePac.a5 = a5;
TirePac.a6 = a6;
TirePac.a7 = a7;
TirePac.a8 = a8;
TirePac.a9 = a9;
TirePac.a10 = a10;
TirePac.a11 = a11;
TirePac.a12= a12;
TirePac.a13 = a13;



muy0 = TirePac.a1 * Fz/1000 + TirePac.a2;
D = muy0 * Fz/1000;
BCD = TirePac.a3 * sin(2 * atan(Fz/1000/TirePac.a4))*(1-TirePac.a5 * abs(camber));

% Pneu linear equivalente

K = BCD * 180/pi;

TireLin = VehicleDynamicsLateral.TireLinear();
TireLin.k = K;

% Lateral force
FyPac = TirePac.Characteristic(deriva, Fz, muy0/1000);
FyLin = TireLin.Characteristic(deriva);

% Graphics
g = VehicleDynamicsLateral.Graphics(TirePac);

%%
% Comparison of tire models

figure(1)
ax = gca;
set(ax, 'NextPlot', 'add', 'Box', 'on', 'XGrid', 'on', 'YGrid', 'on')
p = plot(deriva * 180/pi,-FyLin, 'Color', 'g', 'Marker', 's', 'MarkerFaceColor', 'g', 'MarkeredgeColor', 'k', 'MarkerSize', 7);
g.changeMarker(p, 10);
p = plot(deriva * 180/pi,-FyPac, 'Color', 'r', 'Marker', 'o', 'MarkerFaceColor', 'r', 'MarkeredgeColor', 'k', 'MarkerSize', 7);
g.changeMarker(p, 10);
xlabel('$\alpha$ [grau]', 'Interpreter', 'Latex')
ylabel('$F_y$ [N]', 'Interpreter', 'Latex')
l = legend('Linear', 'Pacejka');
set(l, 'Interpreter', 'Latex', 'Location', 'NorthWest')

%
%% Plant model
% Nonlinear vehicle + Pacejka tire

% TireModel = VehicleDynamicsLateral.TireLinear();
TirePlant = VehicleDynamicsLateral.TirePacejka();

disp(TirePlant)

% Choosing vehicle
% System = VehicleDynamicsLateral.VehicleSimpleLinear();
VehiclePlant = VehicleDynamicsLateral.VehicleSimpleNonlinear();
% Defining vehicle parameters
VehiclePlant.mF0 = 700;
VehiclePlant.mR0 = 600;
VehiclePlant.IT = 10000;
VehiclePlant.lT = 3.5;
VehiclePlant.nF = 1;
VehiclePlant.nR = 1;
VehiclePlant.wT = 2;
VehiclePlant.muy = .8;
VehiclePlant.tire = TirePlant;

disp(VehiclePlant)

% Choosing simulation
T = 5;                      % Total simulation time [s]
resol = 500;                 % Resolution
TSPAN = 0:T/resol:T;        % Time span [s]
simulator = VehicleDynamicsLateral.Simulator(VehiclePlant, TSPAN);

% Changing initial conditions
simulator.ALPHAT0 = -0.2;             % Initial side slip angle [rad]
simulator.dPSI0 = 0.7;                % Initial yaw rate [rad/s]

% Simulation
simulator.Simulate();

% g = VehicleDynamicsLateral.Graphics(simulator);
% g.Animation('animations/plantAnimation');
%
%%
% <<../Vehicle-Dynamics-Lateral/Examples/SteeringControl/animations/plantAnimation.gif>>
%
%% Reference model
%
% *Vehicle parameters*
mT = 1300;
IT = 10000;
a = 1.6154;
b = 1.8846;
vT0 = 20;
KF = 40000;
KR = 40000;

%%
% *Linear system*
%

A = [      0   vT0            vT0                         0                       ;...
           0    0              0                          1                       ;...
           0    0      -(KF+KR)/(mT*vT0)  -(mT*vT0+(a*KF-b*KR)/(vT0))/(mT*vT0)    ;...
           0    0      -(a*KF-b*KR)/IT    -(a^2*KF+b^2*KR)/(IT*vT0)               ];

B = [   0                  ;...
        0                  ;...
        KF/(mT*vT0)        ;...
        a*KF/IT            ];


C = [1 0 0 0];

%%
% A
disp(A)
%%
% B
disp(B)
%%
% C
disp(C)

%%
% *LQR design*
%

Q = [   1 0 0 0 ;...
        0 1 0 0 ;...
        0 0 1 0 ;...
        0 0 0 1 ];

%%
% Q

disp(Q)

R = 1;

%%
% R

disp(R)

K = lqr(A,B,Q,R);

%%
% K

disp(K)

%%
% Control law
%
% $$\delta = - {\bf K} {\bf z} + K_1 r$$
%
%% Simulation 1 - Lane change maneuver
% *Control - Step y*
%
% Reference - r = 2 m
%
% $$ \delta_{max} = \pm 70 deg $$

% Choosing tire
TireModel = VehicleDynamicsLateral.TirePacejka();
% Choosing vehicle
System = VehicleDynamicsLateral.VehicleSimpleNonlinear();
% Defining vehicle parameters
System.mF0 = 700;
System.mR0 = 600;
System.IT = 10000;
System.lT = 3.5;
System.nF = 1;
System.nR = 1;
System.wT = 2;
System.muy = .8;
System.deltaf = @ControlLaw1;

System.tire = TireModel;
% Choosing simulation
T = 3.5;                      % Total simulation time [s]
resol = 500;                 % Resolution
TSPAN = 0:T/resol:T;        % Time span [s]
simulator = VehicleDynamicsLateral.Simulator(System, TSPAN);

% Simulation
simulator.Simulate();

% Retrieving states
% XT = simulator.XT;
YT = simulator.YT;
PSI = simulator.PSI;
% VEL = simulator.VEL;
ALPHAT = simulator.ALPHAT;
dPSI = simulator.dPSI;

x = [YT PSI ALPHAT dPSI];

r = 2;
% Control gain
K = [1.0000    8.3851    4.1971    0.6460];

u = zeros(length(TSPAN),1);
output = zeros(length(TSPAN),1);
for ii = 1:length(TSPAN)
    u(ii) = - K*x(ii,:)' + K(1)*r;
    % Saturation at 70 deg
    if abs(u(ii)) < 70*pi/180
        output(ii) = u(ii);
    else
        output(ii) = sign(u(ii))*70*pi/180;
    end
end

f1 = figure;
set(f1,'PaperUnits','centimeters')
set(f1,'PaperPosition',[0 0 8.9 5])
PaperPos = get(f1,'PaperPosition');
set(f1,'PaperSize',PaperPos(3:4))
hold on; box on; grid on
plot(TSPAN,YT,'r')
plot(TSPAN,PSI,'g')
plot(TSPAN,ALPHAT,'b')
plot(TSPAN,dPSI,'c')
xlabel('Time [s]')
ylabel('States')
l = legend('$y$','$\psi$','$\alpha_T$','$\dot{\psi}$');
set(l,'Interpreter','Latex','Location','East')
print(gcf,'-dpdf','animations/controlStates1.pdf')


% g = VehicleDynamicsLateral.Graphics(simulator);
% g.Animation('animations/controlAnimation1');
% g.Frame('animations/controlFrame1');

%%
% <<../Vehicle-Dynamics-Lateral/Examples/SteeringControl/animations/controlAnimation1.gif>>

%
f1 = figure;
set(f1,'PaperUnits','centimeters')
set(f1,'PaperPosition',[0 0 8.9 3.5])
PaperPos = get(f1,'PaperPosition');
set(f1,'PaperSize',PaperPos(3:4))
hold on; box on; grid on
plot(TSPAN,output*180/pi,'k')
xlabel('Time [s]')
y = ylabel('$\delta [deg]$');
set(y,'Interpreter','Latex')
print(gcf,'-dpdf','animations/controlInput1.pdf')

%% Simulation 2 - Double lane change maneuver
% *Control - Step y*
%
% if t<= 3
%     r = 2;
% else
%     r = 0;
% end
%

System.muy = 0.4;
System.deltaf = @ControlLaw2;

T = 7;                      % Total simulation time [s]
resol = 500;                 % Resolution
TSPAN = 0:T/resol:T;        % Time span [s]


% Choosing simulation
simulator = VehicleDynamicsLateral.Simulator(System, TSPAN);

% Simulation
simulator.Simulate();

% Retrieving states
XT = simulator.XT;
YT = simulator.YT;
PSI = simulator.PSI;
VEL = simulator.VEL;
ALPHAT = simulator.ALPHAT;
dPSI = simulator.dPSI;

x = [YT PSI ALPHAT dPSI];

u = zeros(length(TSPAN),1);
output = zeros(length(TSPAN),1);
for ii = 1:length(TSPAN)
    t = TSPAN(ii);
    if t <= 3
        r = 2;
    else
        r = 0;
    end

    u(ii) = - K*x(ii,:)' + K(1)*r;
    % Saturation at 70 deg
    if abs(u(ii)) < 70*pi/180
        output(ii) = u(ii);
    else
        output(ii) = sign(u(ii))*70*pi/180;
    end
end




f1 = figure;
set(f1,'PaperUnits','centimeters')
set(f1,'PaperPosition',[0 0 8.9 5])
PaperPos = get(f1,'PaperPosition');
set(f1,'PaperSize',PaperPos(3:4))
hold on; box on; grid on
plot(TSPAN,YT,'r')
plot(TSPAN,PSI,'g')
plot(TSPAN,ALPHAT,'b')
plot(TSPAN,dPSI,'c')
xlabel('Time [s]')
ylabel('States')
l = legend('$y$','$\psi$','$\alpha_T$','$\dot{\psi}$');
set(l,'Interpreter','Latex','Location','NorthEast')

print(gcf,'-dpdf','animations/controlStates2.pdf')

%%
%
close all
% g = VehicleDynamicsLateral.Graphics(simulator);
% g.Animation('animations/controlAnimation2');
% g.Frame('animations/controlFrame2');

%%
% <<../Vehicle-Dynamics-Lateral/Examples/SteeringControl/animations/controlAnimation2.gif>>
%

f1 = figure;
set(f1,'PaperUnits','centimeters')
set(f1,'PaperPosition',[0 0 8.9 3.5])
PaperPos = get(f1,'PaperPosition');
set(f1,'PaperSize',PaperPos(3:4))
hold on; box on; grid on
plot(TSPAN,output*180/pi,'k')
xlabel('Time [s]')
y = ylabel('$\delta [deg]$');
set(y,'Interpreter','Latex')
print(gcf,'-dpdf','animations/controlInput2.pdf')


%% See Also
%
% <index.html Index>
%
