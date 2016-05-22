%%  Nonlinear 4 DOF articulated vehicle model
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

classdef VehicleArticulatedNonlinear < VehicleDynamicsLateral.VehicleArticulated
    methods
        % Constructor
        function self = VehicleArticulatedNonlinear()
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
        function dx = Model(self, ~, estados)
            % Dados do ve�culo
            mT = self.mT;       % massa do veiculo [kg]
            mS = self.mS;       % massa do veiculo [kg]
            IT = self.IT;       % massa do veiculo [kg]
            IS = self.IS;       % massa do veiculo [kg]
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

            % Defini��o dos estados
            PSI = estados(3,1);               % �ngulo de orienta��o do caminh�o-trator [rad]
            PHI = estados(4,1);         % M�dulo do vetor velocidade do CG do caminh�o-trator [m/s]
            VT = estados(5,1);         % �ngulo relativo entre o semirreboque e o caminh�o-trator [rad]
            ALPHAT = estados(6,1);      % �ngulo de deriva do CG do caminh�o-trator [rad]
            dPSI = estados(7,1);              % Velocidade angular do caminh�o-trator [rad/s]
            dPHI = estados(8,1);              % Velocidade angular relativa entre o semirreboque e o caminh�o-trator [rad/s]

            % Angulos de deriva n�o linear
            ALPHAF = atan2((a * dPSI + VT * sin(ALPHAT)),(VT * cos(ALPHAT))) - deltaf;
            ALPHAR = atan2((-b * dPSI + VT * sin(ALPHAT)),(VT * cos(ALPHAT)));
            ALPHAM = atan2(((d + e)*(dPHI - dPSI) + VT * sin(ALPHAT + PHI) - b * dPSI * cos(PHI) - ...
                     c * dPSI * cos(PHI)),(VT * cos(ALPHAT + PHI) + b * dPSI * sin(PHI) + c * dPSI * sin(PHI)));

            % For�as longitudinais
            FxF = 0;
            FxR = 0;
            FxM = 0;
            % For�as laterais nos tires - Curva caracter�stica
            FyF = nF * self.tire.Characteristic(ALPHAF, FzF/nF, muy);
            FyR = nR * self.tire.Characteristic(ALPHAR, FzR/nR, muy);
            FyM = nM * self.tire.Characteristic(ALPHAM, FzM/nM, muy);

            f = [...
            VT*cos(PSI+ALPHAT);...
            VT*sin(PSI+ALPHAT);...
            dPSI;...
            dPHI;...
            FxF*cos(PSI + deltaf) + FxR*cos(PSI) + FxM*cos(PSI - PHI) - FyF*sin(PSI + deltaf) - FyR*sin(PSI) - FyM*sin(PSI - PHI) - mS*(b+c)*dPSI^2*cos(PSI) - mS*d*(dPSI - dPHI)^2*cos(PSI - PHI) + (mT + mS)*VT*sin(PSI+ALPHAT)*dPSI;...
            FxF*sin(PSI + deltaf) + FxR*sin(PSI) + FxM*sin(PSI - PHI) + FyF*cos(PSI + deltaf) + FyR*cos(PSI) + FyM*cos(PSI - PHI) - mS*(b+c)*dPSI^2*sin(PSI) - mS*d*(dPSI - dPHI)^2*sin(PSI - PHI) - (mT + mS)*VT*cos(PSI+ALPHAT)*dPSI;...
            FxF*a*sin(deltaf) + FxM*(b + c)*sin(PHI) + FyF*a*cos(deltaf) - FyR*b - FyM*((b+c)*cos(PHI) + (d+e)) - mS*(b+c)*d*(dPSI - dPHI)^2*sin(PHI) + mS*(b+c)*d*dPSI^2*sin(PHI) + mS*((b+c)*VT*cos(ALPHAT) + d*VT*cos(ALPHAT + PHI))*dPSI;...
            FyM*(d + e) - mS*(b+c)*d*dPSI^2*sin(PHI) - mS*d*VT*cos(ALPHAT + PHI)*dPSI ];

            dx = f;
        end

        %% Matriz de massa
        %
        function M = MassMatrix(self,~,estados)
            % Vehicle Parameters
            mT = self.mT;       % massa do veiculo [kg]
            mS = self.mS;       % massa do veiculo [kg]
            IT = self.IT;       % massa do veiculo [kg]
            IS = self.IS;       % massa do veiculo [kg]
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

            % States
            PSI = estados(3,1);               % �ngulo de orienta��o do caminh�o-trator [rad]
            PHI = estados(4,1);         % M�dulo do vetor velocidade do CG do caminh�o-trator [m/s]
            VT = estados(5,1);         % �ngulo relativo entre o semirreboque e o caminh�o-trator [rad]
            ALPHAT = estados(6,1);      % �ngulo de deriva do CG do caminh�o-trator [rad]
            dPSI = estados(7,1);              % Velocidade angular do caminh�o-trator [rad/s]
            dPHI = estados(8,1);              % Velocidade angular relativa entre o semirreboque e o caminh�o-trator [rad/s]

            % Matriz de massa
            M55 = (mT + mS)*cos(PSI + ALPHAT);
            M56 = -(mT + mS)*VT*sin(PSI + ALPHAT);
            M57 = mS*( (b+c)*sin(PSI) + d*sin(PSI - PHI) );
            M58 = -mS*d*sin(PSI - PHI);
            M65 = (mT + mS)*sin(PSI + ALPHAT);
            M66 = (mT + mS)*VT*cos(PSI + ALPHAT);
            M67 = -mS*( (b+c)*cos(PSI) + d*cos(PSI - PHI) );
            M68 = mS*d*cos(PSI - PHI);
            M75 = -mS*( (b+c)*sin(ALPHAT) + d*sin(ALPHAT + PHI) );
            M76 = -mS*( (b+c)*VT*cos(ALPHAT) + d*VT*cos(ALPHAT + PHI) );
            M77 = mS*( (b+c)^2 + 2*(b+c)*d*cos(PHI) + d^2 ) + IT + IS;
            M78 = -( mS*( (b+c)*d*cos(PHI) + d^2 ) + IS);
            M85 = mS*d*sin(ALPHAT + PHI);
            M86 = mS*d*VT*cos(ALPHAT + PHI);
            M87 = - (mS*(d^2 + (b+c)*d*cos(PHI)) + IS);
            M88 = mS*d^2 + IS;

            M = [   1 0 0 0  0   0   0   0 ;...
                    0 1 0 0  0   0   0   0 ;...
                    0 0 1 0  0   0   0   0 ;...
                    0 0 0 1  0   0   0   0 ;...
                    0 0 0 0 M55 M56 M57 M58 ;...
                    0 0 0 0 M65 M66 M67 M68 ;...
                    0 0 0 0 M75 M76 M77 M78 ;...
                    0 0 0 0 M85 M86 M87 M88 ];
        end
    end
end


%% See Also
%
% <index.html Index> | <VehicleSimpleNonlinear.html VehicleSimpleNonlinear>
%
