%% FILTRO DE KALMAN
%
%
%%
% Média e matriz de covariância dos erros, condicionadas à história das observações
%
%
% Problemas
%
% Mesmo que a condição inicial do estado do sistema seja Gaussiana, como as transformações são não lineares, não se pode garantir que qualquer estado subsequente seja Gaussiano.
%
% Filtros sub-ótimos
%
% O sistema dinâmico é descrito por uma equação diferencial estocástica vetorial
%
% $$ \dot{x} = ... $$
%
%
%% Planta
%
%

clear all                   % Clear workspace
close all                   % Closing figures
clc                         % Clear command window

% % Parâmetros da planta
% % Choosing plant tire
% TirePlant = VehicleDynamicsLateral.TirePacejka()
% %%
%
% % Choosing plant vehicle
% VehiclePlant = VehicleDynamicsLateral.VehicleSimpleNonlinear();
% VehiclePlant.tire = TirePlant
% %%
%
% % Choosing simulation
% T = 6;                      % Total simulation time [s]
% resol = 50;                 % Resolution
% TSPAN = 0:T/resol:T;        % Time span [s]
% simulatorPlant = VehicleDynamicsLateral.Simulator(VehiclePlant, TSPAN);
%
% % Simulation
% simulatorPlant.Simulate()
%
% simulatorPlant
% %%
%
% % Retrieving states
% XTPlant = simulatorPlant.XT;
% YTPlant = simulatorPlant.YT;
% PSIPlant = simulatorPlant.PSI;
% PHIPlant = simulatorPlant.PHI;
% VELPlant = simulatorPlant.VEL;
% ALPHATPlant = simulatorPlant.ALPHAT;
% dPSIPlant = simulatorPlant.dPSI;
% dPHIPlant = simulatorPlant.dPHI;
%
% gPlant = VehicleDynamicsLateral.Graphics(simulatorPlant);
% gPlant.Frame(0);
%
%
% %% Modelo
% %
% %
% close all                   % Closing figures
% % Parâmetros da planta
% % Choosing plant tire
% TireModel = VehicleDynamicsLateral.TireLinear()
% %%
%
% % Choosing plant vehicle
% VehicleModel = VehicleDynamicsLateral.VehicleSimpleNonlinear();
% VehicleModel.tire = TireModel
% %%
%
% % Choosing simulation
% T = 6;                      % Total simulation time [s]
% resol = 50;                 % Resolution
% TSPAN = 0:T/resol:T;        % Time span [s]
% simulatorModel = VehicleDynamicsLateral.Simulator(VehicleModel, TSPAN);
%
% % Simulation
% simulatorModel.Simulate()
%
% simulatorModel
% %%
%
% % Retrieving states
% XTModel = simulatorModel.XT;
% YTModel = simulatorModel.YT;
% PSIModel = simulatorModel.PSI;
% PHIModel = simulatorModel.PHI;
% VELModel = simulatorModel.VEL;
% ALPHATModel = simulatorModel.ALPHAT;
% dPSIModel = simulatorModel.dPSI;
% dPHIModel = simulatorModel.dPHI;
%
% gModel = VehicleDynamicsLateral.Graphics(simulatorModel);
% gModel.Frame(0);
%
% %%
% close all                   % Closing figures
% %%
% % Comparação - Diferença de 10 m na direção X no momento da curva.
% gPlant.Frame(0);
% hold on
% gModel.Frame(0);

%% Matrizes do modelo linearizado

syms XT YT PSI vT ALPHAT dPSI mT IT a b K

%%
% Slip angles

ALPHAF = atan((vT * sin(ALPHAT) + a * dPSI)/(vT * cos(ALPHAT))); % Dianteiro
ALPHAR = atan((vT * sin(ALPHAT) - b * dPSI)/(vT * cos(ALPHAT))); % Traseiro

%%
pretty(ALPHAF)
%%
pretty(ALPHAR)

%%
% Lateral forces
FyF = -K*ALPHAF;
FyR = -K*ALPHAR;

%%
pretty(FyF)
%%
pretty(FyR)

%%
% Equations of motion
f1 = vT * cos(ALPHAT + PSI);
f2 = vT * sin(ALPHAT + PSI);
f3 = dPSI;
f4 = (FyF * sin(ALPHAT) + FyR * sin(ALPHAT))/(mT);
f5 = (FyF * cos(ALPHAT) + FyR * cos(ALPHAT) - mT * vT * dPSI) / (mT * vT);
f6 = (FyF * a - FyR * b) / IT;

f = [f1 ; f2 ; f3 ; f4 ; f5 ; f6];

%%
pretty(f)

%%
% Vetor de estados

States = [XT ; YT ; PSI ; vT ; ALPHAT ; dPSI];

F = jacobian(f,States);
F = simplify(F);

%%
pretty(F)

%% Medição


%%
% $$ \dot{x} = v_{\rm T} \cos \left( \psi + \alpha_{\rm T} \right) $$
%
%
% $$ \dot{y} = v_{\rm T} \sin \left( \psi + \alpha_{\rm T} \right) $$
%
% $$ \ddot{x} = \dot{v}_{\rm T} \cos \left( \psi + \alpha_{\rm T} \right) - v_{\rm T} \left( \dot{\psi} + \dot{\alpha}_{\rm T} \right) \sin \left( \psi + \alpha_{\rm T} \right) $$
%
%
% $$ \ddot{y} = \dot{v}_{\rm T} \sin \left( \psi + \alpha_{\rm T} \right) + v_{\rm T} \left( \dot{\psi} + \dot{\alpha}_{\rm T} \right) \cos \left( \psi + \alpha_{\rm T} \right) $$
%

ddX = f4*cos(PSI + ALPHAT) - vT*(dPSI + f5)*sin(PSI + ALPHAT);
ddY = f4*sin(PSI + ALPHAT) + vT*(dPSI + f5)*cos(PSI + ALPHAT);

% $$ {\bf A} = \left( \ddot{x} \cos \psi - \ddot{y} \sin \psi \right) {\bf t}_x + \left( - \ddot{x} \sin \psi + \ddot{y} \sin \psi \right) {\bf t}_y$$

%%
ACEL = [ddX*cos(PSI) - ddY*sin(PSI) ; -ddX*sin(PSI) + ddY*cos(PSI)];
ACEL = simplify(ACEL);
%%
pretty(ACEL)

H = jacobian(ACEL,States);
H = simplify(H);

%%
% Testando a linearização na reta
syms v0
A = subs(F,States,[0 ; 0 ; 0 ; v0 ; 0 ; 0]);
%%
pretty(A)

%%
C = subs(H,States,[0 ; 0 ; 0 ; v0 ; 0 ; 0]);
pretty(C)

%% Filtro estendido de Kalman
