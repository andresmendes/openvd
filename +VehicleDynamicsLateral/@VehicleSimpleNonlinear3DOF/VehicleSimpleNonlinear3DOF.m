%% Nonlinear 3 DOF vehicle model
% Bicycle model nonlinear with 3 degrees of freedom.
%
%% Sintax
% |dx = _VehicleModel_.Model(~,estados)|
%
%% Arguments
% The following table describes the input arguments:
%
% <html> <table border=1 width="97%">
% <tr> <td width="30%"><tt>estados</tt></td> <td width="70%">Estados do modelo: [dPSI ALPHAT PSI XT YT VEL]</td> </tr>
% </table> </html>
%
%% Description
% O centro de massa do ve�culo � dado pelo ponto $T$ e os eixos dianteiro e traseiro s�o dados pelos pontos $F$ e $R$, respectivamente. A constante $a$ mede a dist�ncia do ponto $F$ ao $T$ e $b$ a dist�ncia do ponto $T$ ao $R$. Os �ngulos $\alpha_F$ e $\alpha_R$ s�o os �ngulos de deriva nos eixos dianteiro e traseiro. $\alpha_T$ is the vehicle side slip angle and $\psi$ is the vehicle yaw angle. Por fim, $\delta$ � o �ngulo de ester�amento.
%
% <<illustrations/modeloSimples.svg>>
%
%% Code
%

classdef VehicleSimpleNonlinear3DOF < VehicleDynamicsLateral.VehicleSimple
	methods
        % Constructor
        function self = VehicleSimpleNonlinear3DOF(varargin)
            if nargin == 0
                % Entrada padr�o dos dados do ve�culo
                mF0 = 5237;     % Massa no eixo dianteiro [kg]
                mR0 = 2440;     % Massa no eixo traseiro [kg]
                IT = 46100;     % Momento de in�rcia [kg*m2]
                DELTA = 0;      % Ester�amento do eixo dianteiro [rad]
                lT = 3.550;     % Dist�ncia entre os eixos [m]
                nF = 2;         % N�mero de tires no eixo dianteiro
                nR = 2;         % N�mero de tires no eixo traseiro
                largT = 2;      % width do ve�culo[m]
                muy = 0.3;      % Coeficiente de atrito de opera��o
                entradaVetor = [mF0 mR0 IT DELTA lT nF nR largT muy];
                % Definindo os par�metros da classe
                self.params = self.convert(entradaVetor);
                self.tire = VehicleDynamicsLateral.tirePacejka;
            else
                self.params = self.convert(varargin{1});
                self.tire = varargin{2};
            end
                self.distFT = self.params(11);
                self.distTR = self.params(12);
                self.width = self.params(8);
        end

        %% Model
        % Fun��o com as equa��es de estado do modelo
        function dx = Model(self,~,estados)
            % Dados do ve�culo
            m = self.params(10);        % massa do veiculo [kg]
            I = self.params(3);         % momento de inercia [kg]
            a = self.params(11);        % distancia do eixo dianteiro ao centro de massa [m]
            b = self.params(12);        % distancia do eixo dianteiro ao centro de massa [m]
            nF = self.params(6);        % N�mero de tires no eixo dianteiro do caminh�o-trator
            nR = self.params(7);        % N�mero de tires no eixo traseiro do caminh�o-trator
            muy = self.params(9);       % Coeficiente de atrito de opera��o
            DELTA = self.params(4);
            g = 9.81;                   % Acelera��o da gravidade [m/s^2]
            FzF = self.params(1)*g;     % Carga vertical no eixo dianteiro [N]
            FzR = self.params(2)*g;     % Carga vertical no eixo traseiro [N]
            % Estados
            dPSI = estados(1);
            ALPHAT = estados(2);
            v = estados(6);
            PSI = estados(3);

            % Angulos de deriva n�o linear
            ALPHAF = atan2((v*sin(ALPHAT) + a*dPSI),(v*cos(ALPHAT))) - DELTA; % Dianteiro
            ALPHAR = atan2((v*sin(ALPHAT) - b*dPSI),(v*cos(ALPHAT)));         % Traseiro

            % For�as longitudinais
            FxF = 0;
            FxR = 0;

            % Curva caracter�stica
            FyF = nF*self.tire.Characteristic(ALPHAF,FzF/nF,muy);
            FyR = nR*self.tire.Characteristic(ALPHAR,FzR/nR,muy);

            % Equa��es de estado
            dx(1,1) = (FyF*a*cos(DELTA) - FyR*b + FxF*a*sin(DELTA))/I;
            dx(2,1) = (FyR + FyF*cos(DELTA) + FxF*sin(DELTA) - m*(dPSI*v*cos(ALPHAT) + (sin(ALPHAT)*(FxR + 	FxF*cos(DELTA) - FyF*sin(DELTA) +...
                      dPSI*m*v*sin(ALPHAT)))/(m*cos(ALPHAT))))/(m*(v*cos(ALPHAT) + (v*sin(ALPHAT)^2)/cos(ALPHAT)));
            dx(6,1) = (FxR*cos(ALPHAT) + FyR*sin(ALPHAT) - FyF*cos(ALPHAT)*sin(DELTA) + FyF*cos(DELTA)*sin(ALPHAT) + ...
                      FxF*sin(ALPHAT)*sin(DELTA) + FxF*cos(ALPHAT)*cos(DELTA))/(m*cos(ALPHAT)^2 + m*sin(ALPHAT)^2);

            % Obten��o da orienta��o
            dx(3,1) = dPSI; % dPSI

            % Equa��es adicionais para o posicionamento (N�o necess�rias para a din�mica em guinada)
            dx(4,1) = v*cos(ALPHAT + PSI); % X
            dx(5,1) = v*sin(ALPHAT + PSI); % Y
        end

    end

    methods (Static)
        %% convert
        % A fun��o convert adiciona no vetor de entrada ([mF0 mR0 IT DELTA lT nF nR largT muy]) os par�metros restantes do modelo de ve�culo ([mT a b]).
        function parametros = convert(entrada)
            mF0 = entrada(1);       % Massa no eixo dianteiro [kg]
            mR0 = entrada(2);       % Massa no eixo traseiro [kg]
            lT = entrada(5);        % Dist�ncia entre os eixos [m]
            % Convers�o dos dados para os par�metros usados na equa��o de movimento
            mT = mF0 + mR0;         % massa do ve�culo [kg]
            a = mR0/mT*lT;          % Dist�ncia (F-T) [m]
            b = lT - a;             % Dist�ncia (R-T) [m]
            % Sa�da
            parametros = [entrada mT a b];
        end
    end

    %% Properties
    %

    properties
        params
        tire
        distFT
        distTR
        width
    end

end

%% See Also
%
% <index.html Index> | <VehicleArticulatedNonlinear4DOF.html VehicleArticulatedNonlinear4DOF>
%
