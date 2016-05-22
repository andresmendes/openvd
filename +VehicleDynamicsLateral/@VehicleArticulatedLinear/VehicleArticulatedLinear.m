%% Nonlinear 4 DOF articulated vehicle model
%
%% Sintax
% |dx = _VehicleModel_.Model(~,estados)|
%
%% Arguments
% The following table describes the input arguments:
%
% <html> <table border=1 width="97%">
% <tr> <td width="30%"><tt>estados</tt></td> <td width="70%">Estados do modelo: [dPSI ALPHAT dPHI VEL PHI PSI XT YT]</td> </tr>
% </table> </html>
%
%% Description
% O �ngulo $\psi$ define a orienta��o do caminh�o-trator em rela��o ao referencial inercial. O estado $\phi$ � o �ngulo formado entre o caminh�o-trator e o semirreboque. O �ngulo $\alpha_T$ � o �ngulo de deriva do m�dulo dianteiro e � formado pelo vetor velocidade do centro de massa e a linha longitudinal do caminh�o-trator. Por fim, $v$ � o m�dulo do vetor velocidade do centro de massa do caminh�o-trator. Os pontos $T$ e $S$ s�o coincidentes com os centros de massa do caminh�o-trator e semirreboque, respectivamente. Os pontos F e R s�o coincidentes com os eixos dianteiro e traseiro do caminh�o-trator, respectivamente. M � o ponto que representa o eixo do semirreboque e A � o ponto de articula��o ente as duas unidades. As grandezas a, b e c da unidade motora s�o as dist�ncias entre os pontos F-T, T-R e R-A, respectivamente. Na unidade movida, d e e definem as dist�ncias entre os pontos A-S e S-M, respectivamente.
%
% <<illustrations/modeloArticulado.svg>>
%
% Este modelo � escrito na forma:
%
% $$ M(x) \dot{x} = f(x)$$
%
% Onde $x$ � o vetor de estados, $M(x)$ � a matriz de massa do sistema e $f(x)$ � uma fun��o vetorial n�o linear. Logo, � necess�ria uma fun��o que permita a integra��o do sistema com a matriz de massa escrita explicitamente. Uma op��o � utilizar a fun��o _ode45_. Details: <http://www.mathworks.com/help/matlab/ref/ode45.html?searchHighlight=%22mass%20matrix%22 ode45 (Mass matrix)>
%
%% Code
%

classdef VehicleArticulatedLinear < VehicleDynamicsLateral.VehicleArticulated
	methods
        % Constructor
        function self = VehicleArticulatedLinear()
            self.mF0 = 5200;
            self.mR0 = 2400;
            self.mF = 6000;
            self.mR = 10000;
            self.mM = 17000;
            self.IT = 46000;
            self.IS = 450000;
            self.lT = 3.5;
            self.lS = 7.7;
            self.c = -0.3;
            self.nF = 2;
            self.nR = 4;
            self.nM = 8;
            self.wT = 2.6;
            self.wS = 2.4;
            self.muy = 0.3;
            self.deltaf = 0;
            self.g = 9.81;
        end

        %% Model
        % Fun��o com as equa��es de estado do modelo
        function dx = Model(self,~,estados)
            % Dados do ve�culo
            mT = self.mT;       % massa do veiculo [kg]
            mS = self.mS;       % massa do veiculo [kg]
            % IT = self.IT;       % massa do veiculo [kg]
            % IS = self.IS;       % massa do veiculo [kg]
            a = self.a;        % distancia do eixo dianteiro ao centro de massa do caminh�o-trator [m]
            b = self.b;        % distancia do eixo traseiro ao centro de massa do caminh�o-trator [m]
            c = self.c;         % distancia da articula��o ao centro de massa do caminh�o-trator [m]
            d = self.d;        % distancia do eixo traseiro ao centro de massa do caminh�o-trator [m]
            e = self.e;        % distancia da articula��o ao centro de massa do caminh�o-trator [m]
            deltaf = self.deltaf;     % Ester�amento [rad]
            nF = self.nF;       % N�mero de tires no eixo dianteiro do caminh�o-trator
            nR = self.nR;       % N�mero de tires no eixo traseiro do caminh�o-trator
            nM = self.nM;       % N�mero de tires no eixo do semirreboque
            g = self.g;                   % Gravity acceleration [m/s^2]
            FzF = self.mF * g;   % Carga vertical no eixo dianteiro [N]
            FzR = self.mR * g;   % Carga vertical no eixo traseiro [N]
            FzM = self.mM * g;   % Carga vertical no eixo do semirreboque [N]
            muy = self.muy;      % Coeficiente de atrito de opera��o

            v0 = 20;

            % State variables
            X = estados(1,1);         % Not used
            Y = estados(2,1);         % Not used
            PSI     = estados(3,1);
            PHI     = estados(4,1);
            V       = estados(5,1);
            ALPHAT  = estados(6,1);
            dPSI    = estados(7,1);
            dPHI    = estados(8,1);

            % Slip angles - linear
            ALPHAF = ALPHAT + a/v0*dPSI - deltaf;
            ALPHAR = ALPHAT - b/v0*dPSI;
            ALPHAM = ALPHAT + PHI - (dPSI*(b + c + d + e))/v0 + (dPHI*(d + e))/v0;

            % Longitudinal forces
            FxF = 0;
            FxR = 0;
            FxM = 0;

            % Lateral forces - Characteristic curve
            FyF = nF*self.tire.Characteristic(ALPHAF,FzF/nF,muy);
            FyR = nR*self.tire.Characteristic(ALPHAR,FzR/nR,muy);
            FyM = nM*self.tire.Characteristic(ALPHAM,FzM/nM,muy);

A = [ 0  0   0  0  1   0                       0  0;...
      0  0  v0  0  0  v0                       0  0;...
      0  0   0  0  0   0                       1  0;...
      0  0   0  0  0   0                       0  1;...
      0  0   0  0  0   0                       0  0;...
      0  0   0  0  0   0           (-v0*(mS + mT))  0;...
      0  0   0  0  0   0  (mS*(d*v0 + v0*(b + c)))  0;...
      0  0   0  0  0   0                (-d*mS*v0)  0];

B = [ 0  0  0  0  0   0                0;...
      0  0  0  0  0   0                0;...
      0  0  0  0  0   0                0;...
      0  0  0  0  0   0                0;...
      0  1  1  1  0   0                0;...
      0  0  0  0  1   1                1;...
      0  0  0  0  a  -b  (- b - c - d - e);...
      0  0  0  0  0   0            (d + e)];

vetEst = [X ; Y ; PSI ; PHI ; V ; ALPHAT ; dPSI ; dPHI];
vetEnt = [deltaf ; FxF ; FxR ; FxM ; FyF ; FyR ; FyM];

            % Integrator output
            dx = A*vetEst + B*vetEnt;
        end

        %% Matriz de massa
        %

        function E = MassMatrix(self,~,~)
            % Vehicle parameters
            mT = self.mT;       % massa do veiculo [kg]
            mS = self.mS;       % massa do veiculo [kg]
            IT = self.IT;       % massa do veiculo [kg]
            IS = self.IS;       % massa do veiculo [kg]
            % a = self.a;        % distancia do eixo dianteiro ao centro de massa do caminh�o-trator [m]
            b = self.b;        % distancia do eixo traseiro ao centro de massa do caminh�o-trator [m]
            c = self.c;         % distancia da articula��o ao centro de massa do caminh�o-trator [m]
            d = self.d;        % distancia do eixo traseiro ao centro de massa do caminh�o-trator [m]
            % e = self.e;        % distancia da articula��o ao centro de massa do caminh�o-trator [m]
            % deltaf = self.deltaf;     % Ester�amento [rad]
            % nF = self.nF;       % N�mero de tires no eixo dianteiro do caminh�o-trator
            % nR = self.nR;       % N�mero de tires no eixo traseiro do caminh�o-trator
            % nM = self.nM;       % N�mero de tires no eixo do semirreboque
            % g = self.g;                   % Gravity acceleration [m/s^2]

            v0 = 20;

            % Mass matrix
            E = [ 1  0  0  0        0                        0                                               0                            0;...
                  0  1  0  0        0                        0                                               0                            0;...
                  0  0  1  0        0                        0                                               0                            0;...
                  0  0  0  1        0                        0                                               0                            0;...
                  0  0  0  0  (mS + mT)                      0                                               0                            0;...
                  0  0  0  0        0             (v0*(mS + mT))                                 (-mS*(b + c + d))                         (d*mS);...
                  0  0  0  0        0  ( -mS*v0*(b + c + d))            (IS + IT + mS*(b + c + d)^2)  (- IS - mS*(d^2 + (b + c)*d));...
                  0  0  0  0        0                  (d*mS*v0)                     (- IS - mS*(d^2 + (b + c)*d))                  (mS*d^2 + IS)];

        end
    end
end

%% See Also
%
% <index.html Index> | <VehicleSimpleNonlinear3DOF.html VehicleSimpleNonlinear3DOF>
%
