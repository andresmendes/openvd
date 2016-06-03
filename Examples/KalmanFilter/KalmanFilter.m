%% KALMAN FILTER
% Kalman filter application.
%
% VERIFICAR A NORMA DE P - PARECE ERRADA
%
% FAZER NORMA DO GANHO DE KALMAN
%
% RESIDO? OU ALGO PARECIDO COM O GRAFICO QUE COMPARA P- e P+
%
%% System
% Supondo um sistema não linear descrito por uma equação diferencial estocástica vetorial dada por:
%
% $$ \dot{{\bf x}} = {\bf f} ( {\bf x}(t) , t ) + {\bf G} (t) {\bf w} (t) $$
%
% onde
%
% * ${\bf w}$ é de processo branco gaussiano. ${\bf w}(t)$ ~ $N ( {\bf 0} , {\bf Q}(t) )$
% * ${\bf f} ( {\bf x}(t) , t )$ é uma função vetorial de dimensão $n$
% * ${\bf x}(t)$ é o vetor de estados $(n \times 1)$
% * ${\bf G}$ é uma matriz conhecida $(n \times s)$
%
%% Plant
% Neste exemplo, a planta é representada por um modelo dinâmico não linear baseado no modelo físico ilustrado na seguinte figura
%
% <<../illustrations/modelSimple.svg>>
%
% O modelo da planta é composto pelos modelos não lineares de maior complexidade disponível no pacote, ou seja, o modelo de veículo <../DocVehicleSimpleNonlinear.html Vehicle Simple Nonlinear> com o modelo de pneu <../DocTirePacejka.html Tire Pacejka>. A descrição das equações de movimento podem ser encontradas em <../theory/vehicleSimple.pdf Simple equations of motion>
%
% Iniciando o código:
%

clear all                   % Clear workspace
close all                   % Closing figures
clc                         % Clear command window

%%
% O pacote é importado com o comando:
%

import VehicleDynamicsLateral.*

%%
% O pneu escolhido <../DocTirePacejka.html Tire Pacejka> é inicializado com os parâmetros predefinidos (default).
%

TirePlant = TirePacejka;
disp(TirePlant)

%%
% O modelo de veículo <../DocVehicleSimpleNonlinear.html Vehicle Simple Nonlinear> é inicializado, também, com os parâmetros predefinidos (default). Em seguida o pneu escolhido é passado para o veículo.
%

VehiclePlant = VehicleSimpleNonlinear;
VehiclePlant.tire = TirePlant;
disp(VehiclePlant)

%% Maneuver
% Este tópico apresenta a manobra do veículo (plant) que deverá ser estimada pelo Filtro de Kalman.
%
% Choosing simulation parameters:
%

T = 6;                      % Total simulation time [s]
resol = 50;                 % Resolution
TSPAN = 0:T/resol:T;        % Time span [s]

%%
% Inicializando o simulador e simulando.
%

simulatorPlant = Simulator(VehiclePlant, TSPAN);
simulatorPlant.Simulate

%%
% Obtendo os parâmetros da simulação.
%

disp(simulatorPlant)

%%
% Retrieving states
%

XTPlant = simulatorPlant.XT;
YTPlant = simulatorPlant.YT;
PSIPlant = simulatorPlant.PSI;
vTPlant = simulatorPlant.VEL;
ALPHATPlant = simulatorPlant.ALPHAT;
dPSIPlant = simulatorPlant.dPSI;


XOUTPlant = [XTPlant YTPlant PSIPlant vTPlant ALPHATPlant dPSIPlant];

%%
% Gerando os gráficos da manobra do carro (plant)
%

gPlant = Graphics(simulatorPlant);
gPlant.TractorColor = 'r';
gPlant.Frame(0);
legend('Plant');

%%
%

close all                   % Closing figures

%% Modelo
% O modelo utilizado no algoritmo de estimação é baseado no mesmo modelo físico considerado no modelo da planta. Além disso, o modelo parte da premissa de que o projetista do estimador não tem conhecimento adequado da curva característica do pneu. Logo, o modelo de veículo é igual ao utilizado na planta, porém, o modelo de pneu é dado pelo modelo linear <../DocTireLinear.html Tire linear>, resultando num modelo do sistema de menor complexidade (em relação à planta) e com mais hipóteses simplificadoras.
%
% Inicializando o pneu
%

TireModel = TireLinear;
disp(TireModel)

%%
% Choosing model vehicle
%

VehicleModel = VehicleSimpleNonlinear;
VehicleModel.tire = TireModel;
disp(VehicleModel)

%%
% Simulador com o mesmo vetor TSPAN e simulação
%

simulatorModel = Simulator(VehicleModel, TSPAN);
simulatorModel.Simulate;

disp(simulatorModel)

%%
% Retrieving states
%

XTModel = simulatorModel.XT;
YTModel = simulatorModel.YT;
PSIModel = simulatorModel.PSI;
PHIModel = simulatorModel.PHI;
VELModel = simulatorModel.VEL;
ALPHATModel = simulatorModel.ALPHAT;
dPSIModel = simulatorModel.dPSI;
dPHIModel = simulatorModel.dPHI;

%%
% A manobra gerada pelo modelo escolhido pelo projetista a partir da mesma condição inicial é ilustrada na figura a seguir
%

gModel = Graphics(simulatorModel);
gModel.TractorColor = 'g';
gModel.Frame(0);
legend('Model');

%%
%

close all                   % Closing figures

%% Plant and model comparison
% Comparação - Diferença de 10 m na direção X no momento da curva.
%

gPlant.Frame(0);
hold on
gModel.Frame(0);
l = legend('Plant','Model');
set(l,'Location','NorthWest')

%%
%

close all                   % Closing figures

%% Model linearization
% A expressão geral da equação linearizada é obtida neste tópico utilizando o processador simbólico.
%
% Definindo os símbolos:
%

syms XT YT PSI vT ALPHAT dPSI mT IT a b K

%%
% Slip angles
%

ALPHAF = atan((vT * sin(ALPHAT) + a * dPSI)/(vT * cos(ALPHAT))); % Dianteiro
ALPHAR = atan((vT * sin(ALPHAT) - b * dPSI)/(vT * cos(ALPHAT))); % Traseiro

%%
%

pretty(ALPHAF)

%%
%

pretty(ALPHAR)

%%
% Lateral forces
%

FyF = -K*ALPHAF;
FyR = -K*ALPHAR;

%%
%

pretty(FyF)

%%
%

pretty(FyR)

%%
% State equations
%

f1 = vT * cos(ALPHAT + PSI);
f2 = vT * sin(ALPHAT + PSI);
f3 = dPSI;
f4 = (FyF * sin(ALPHAT) + FyR * sin(ALPHAT))/(mT);
f5 = (FyF * cos(ALPHAT) + FyR * cos(ALPHAT) - mT * vT * dPSI) / (mT * vT);
f6 = (FyF * a - FyR * b) / IT;

f = [f1 ; f2 ; f3 ; f4 ; f5 ; f6];

%%
%

pretty(f)

%%
% Vetor de estados

States = [XT ; YT ; PSI ; vT ; ALPHAT ; dPSI];

%%
%

pretty(States)

%%
% O sistema linearizado é escrito na forma
%
% $$ \dot{\bf x} = {\bf F} {\bf x}$$
%
% onde ${\bf F}$ é a matriz dinâmica do modelo linear que é calculada a partir da equação não linear expandida em série de Taylor e truncada nos termos de primeira ordem. Logo, a matriz ${\bf F}$ é dada por
%
% $${\bf F} = \left[ \frac{\partial f_i}{\partial x_j} \right]_{n \times n}$$
%
% onde $i$ e $j$ indicam as equações e variáveis de estado utilizadas no cálculo correpondente à posição $(i,j)$ da matriz jacobiana.
%

F = jacobian(f,States);
F = simplify(F);

%%
%

pretty(F)

%% Medição
% Continuando simbolicamente, as grandezas medidas são: aceleração longitudinal e aceleração transversal do veículo.
%
% Para isso, utiliza-se as relações:
%
% $$ \dot{x} = v_{\rm T} \cos \left( \psi + \alpha_{\rm T} \right) $$
%
% $$ \dot{y} = v_{\rm T} \sin \left( \psi + \alpha_{\rm T} \right) $$
%
% Aceleração
%
% $$ \ddot{x} = \dot{v}_{\rm T} \cos \left( \psi + \alpha_{\rm T} \right) - v_{\rm T} \left( \dot{\psi} + \dot{\alpha}_{\rm T} \right) \sin \left( \psi + \alpha_{\rm T} \right) $$
%
% $$ \ddot{y} = \dot{v}_{\rm T} \sin \left( \psi + \alpha_{\rm T} \right) + v_{\rm T} \left( \dot{\psi} + \dot{\alpha}_{\rm T} \right) \cos \left( \psi + \alpha_{\rm T} \right) $$
%
% Implementando
%

ddX = f4*cos(PSI + ALPHAT) - vT*(dPSI + f5)*sin(PSI + ALPHAT);
ddY = f4*sin(PSI + ALPHAT) + vT*(dPSI + f5)*cos(PSI + ALPHAT);

%%
% Estes valores obtidos são as acelerações escritas na base fixa $\{ O {\bf i} {\bf j} {\bf k} \}$. A projeção destas grandezas na base móvel $\{ O {\bf t}_x {\bf t}_y {\bf t}_z \}$ é feita através da equação
%
% $$ {\bf a} = \left( \ddot{x} \cos \psi - \ddot{y} \sin \psi \right) {\bf t}_x + \left( - \ddot{x} \sin \psi + \ddot{y} \sin \psi \right) {\bf t}_y$$
%
% Implementando
%

ACEL = [ddX*cos(PSI) - ddY*sin(PSI) ; -ddX*sin(PSI) + ddY*cos(PSI)];
ACEL = simplify(ACEL);

%%
%

pretty(ACEL)

%%
% A equação de observações não linear é dada por
%
% $$ {\bf z}_k = {\bf h} ({\bf x}_k) + {\bf v}_k $$
%
% com
%
% ${\bf v}_k$ ~ $N ( {\bf 0} , {\bf R}_k )$
%
% Linearizando termos
%
% $$ {\bf z}_k = {\bf H} {\bf x}_k + {\bf v}_k $$
%
% onde
%
% $${\bf H} = \left[ \frac{\partial h_i}{\partial x_j} \right]_{m \times n}$$
%
% ou seja, a matriz de saídas {\bf H} é a matriz jacobiana da equação de ACEL em relação aos estados.
%
% Implementando
%

H = jacobian(ACEL,States);
H = simplify(H);

%% Verificação da linearização
% Para verificar o procedimento de linearização, um ponto de operação referente à movimentação do veículo em linha reta com uma velocidade prescrita $v_0$ é utilizado. Esta escolha é típica e pode ser verificada facilmente na literatura.
%
% Definindo $v_0$
%

syms v0

%%
% Obtendo a matriz dinâmica

A = subs(F,States,[0 ; 0 ; 0 ; v0 ; 0 ; 0]);

%%
%

pretty(A)

%%
% Obtendo a matriz de saídas
%

C = subs(H,States,[0 ; 0 ; 0 ; v0 ; 0 ; 0]);

%%
%

pretty(C)

%% Filtro estendido de Kalman
% Implementação do algoritmo
%
% Matriz que distribui o ruído na equação do estado constante
%

G = eye(6); % Matriz identidade (6 x 6)

%%
% Matrizes de covariância
%

Q = eye(6); % Matriz identidade (6 x 6)
R = eye(2); % Matriz identidade (2 x 2)

%%
% Matriz

P0 = eye(6);


%%
% Recuperando as condições iniciais usadas acima
%

X0Num = simulatorModel.X0;
Y0Num = simulatorModel.Y0;
PSI0Num = simulatorModel.PSI0;
VEL0Num = simulatorModel.V0;
ALPHAT0Num = simulatorModel.ALPHAT0;
dPSI0Num = simulatorModel.dPSI0;

x0 = [ X0Num ; Y0Num ; PSI0Num ; VEL0Num ; ALPHAT0Num ; dPSI0Num ];

%%
% Recuperando os parâmetros do veículo
%

mTNum = VehicleModel.mT;
ITNum = VehicleModel.IT;
aNum = VehicleModel.a;
bNum = VehicleModel.b;
KNum = TireModel.k;

parameters = [mTNum ITNum aNum bNum KNum];

%%
% Obtendo e verificando a aceleração que vai ser medida.
%


saidas = [XTPlant YTPlant PSIPlant vTPlant ALPHATPlant dPSIPlant];
matDerivEstados = zeros(size(saidas));
for i = 1:size(saidas,1)
    auxil = VehiclePlant.Model(0,saidas(i,:));
    matDerivEstados(i,:) = auxil';
end

dXTPlant = matDerivEstados(:,1);
dYTPlant = matDerivEstados(:,2);
dPSIPlant = matDerivEstados(:,3);
dvTPlant = matDerivEstados(:,4);
dALPHATPlant = matDerivEstados(:,5);
ddPSIPlant = matDerivEstados(:,6);

ddXPlant = dvTPlant.*cos(PSIPlant + ALPHATPlant) - vTPlant.*(dPSIPlant + dALPHATPlant).*sin(PSIPlant + ALPHATPlant);
ddYPlant = dvTPlant.*sin(PSIPlant + ALPHATPlant) + vTPlant.*(dPSIPlant + dALPHATPlant).*cos(PSIPlant + ALPHATPlant);

ACELNum = [(ddXPlant.*cos(PSIPlant) - ddYPlant.*sin(PSIPlant))  (-ddXPlant.*sin(PSIPlant) + ddYPlant.*cos(PSIPlant))];

figure(1)
ax = gca;
set(ax,'NextPlot','add','Box','on','XGrid','on','YGrid','on')
plot(TSPAN,ACELNum(:,1),'r')
plot(TSPAN,ACELNum(:,2),'g')
xlabel('time [s]')
ylabel('acc. [m/s]')
legend('X','Y')

%%
% Inicializando o tempo de intervalo entre uma observação e outra.
%

intervalo = 0.1;

%%
% Prealocando
%

z = zeros(2,1);                     % Vetor de observações
t = 0:intervalo:T;                  % Vetor com os instantes de observação
XOUTopt = zeros(length(t) + 1,length(States)); % Estimativa dos estados após atualização
Popt = zeros(length(t) + 1,1);                 % Matriz de covariância após atualização

%%
% Atribuindo os primeiros valores
%

XOUTopt(1,:) = x0';
Popt(1,1) = norm(P0);

%%
% Iteração
%


for j = 1:length(t)
    % Índice variando por todos os instantes de observação

    % Vetor de tempo de integração para a etapa de propagação
    tspan = t(j):intervalo/100:t(j)+intervalo;

    % Obtendo as medidas da iteração
    z(1) = interp1(TSPAN,ACELNum(:,1),t(j));
    z(2) = interp1(TSPAN,ACELNum(:,2),t(j));

    Fnum = subs(F,[States.' mT IT a b K],[x0.' parameters]);
    Fnum = double(Fnum);
    Hnum = subs(H,[States.' mT IT a b K],[x0.' parameters]);
    Hnum = double(Hnum);

    % Ciclo de propagação
    %%
    % Transformando a matriz PMat0 $(6 \times 6)$ em um vetor P0 $(1 \times 36)$
    %

    P0 = reshape(P0,[1 size(P0,1)*size(P0,2)]);

    [TOUT,Pout] = ode45(@(t,P) IntCov(t,P,Fnum,G,Q),tspan,P0);

    Pmatrix = reshape(Pout(end,:),[6 6])';


    simulatorKalman = Simulator(VehicleModel, tspan);
    simulatorKalman.TSpan = tspan;
    % Definindo as condições iniciais
    simulatorKalman.X0 = x0(1);
    simulatorKalman.Y0 = x0(2);
    simulatorKalman.PSI0 = x0(3);
    simulatorKalman.V0 = x0(4);
    simulatorKalman.ALPHAT0 = x0(5);
    simulatorKalman.dPSI0 = x0(6);
    % Simulando
    simulatorKalman.Simulate()

    XTKalman = simulatorKalman.XT;
    YTKalman = simulatorKalman.YT;
    PSIKalman = simulatorKalman.PSI;
    vTKalman = simulatorKalman.VEL;
    ALPHATKalman = simulatorKalman.ALPHAT;
    dPSIKalman = simulatorKalman.dPSI;

    XOUTKalman = [XTKalman YTKalman PSIKalman vTKalman ALPHATKalman dPSIKalman];

    % Ciclo de atualização


    ACELKalman = subs(ACEL,[States.' mT IT a b K],[XOUTKalman(end,:) parameters]);
    ACELKalman = double(ACELKalman);

    KKalman = Pmatrix*Hnum' / (Hnum*Pmatrix*Hnum' + R);

    XKalman = XOUTKalman(end,:)' + KKalman*(z - ACELKalman);
    PKalman = Pmatrix - KKalman*Hnum*Pmatrix;

    x0 = XKalman;
    P0 = PKalman;

    XOUTopt(j+1,:) = XKalman';
    Popt(j+1) = norm(PKalman);

end

%% Comparação
figure(1)
hold on
plot(TSPAN,XOUTPlant(:,1),'r')
plot(t,XOUTopt(1:end-1,1),'r--')

figure(2)
hold on
plot(TSPAN,XOUTPlant(:,2),'g')
plot(t,XOUTopt(1:end-1,2),'g--')

figure(3)
hold on
plot(TSPAN,XOUTPlant(:,3),'b')
plot(t,XOUTopt(1:end-1,3),'b--')

figure(4)
hold on
plot(TSPAN,XOUTPlant(:,4),'c')
plot(t,XOUTopt(1:end-1,4),'c--')

figure(5)
hold on
plot(TSPAN,XOUTPlant(:,5),'m')
plot(t,XOUTopt(1:end-1,5),'m--')

figure(6)
hold on
plot(TSPAN,XOUTPlant(:,6),'k')
plot(t,XOUTopt(1:end-1,6),'k--')


figure(7)
plot(t,Popt(1:end-1))

%%
%
close all

%% Trajetória da estimativa
% Usando o simulatorPlant para inicializar o Graphics do Kalman

gKalman = Graphics(simulatorKalman);
gKalman.Simulator.TSpan = t;
gKalman.Simulator.XT = XOUTopt(1:end-1,1);
gKalman.Simulator.YT = XOUTopt(1:end-1,2);
gKalman.Simulator.PSI = XOUTopt(1:end-1,3);
gKalman.Simulator.VEL = XOUTopt(1:end-1,4);
gKalman.Simulator.ALPHAT = XOUTopt(1:end-1,5);
gKalman.Simulator.dPSI = XOUTopt(1:end-1,6);
gKalman.TractorColor = 'b';
gKalman.Frame(0)

%%
close all

%%
%
gPlant.Frame(0);
hold on
gModel.Frame(0);
gKalman.Frame(0);
