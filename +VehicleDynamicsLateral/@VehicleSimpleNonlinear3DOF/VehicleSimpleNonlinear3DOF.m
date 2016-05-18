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
        function self = VehicleSimpleNonlinear3DOF()
	        self.mF0 = 700;
	        self.mR0 = 600;
	        self.IT = 10000;
	        self.lT = 3.5;
	        self.nF = 2;
	        self.nR = 2;
	        self.wT = 2;
	        self.muy = .8;
	        self.deltaf = 0;
        end

        %% Model
        % Função com as equações de estado do modelo
        function dx = Model(self, ~, estados)
			% Data
            m = self.mT;
            I = self.IT;
            a = self.a;
            b = self.b;
            nF = self.nF;
            nR = self.nR;
            muy = self.muy;
            deltaf = self.deltaf;

			g = 9.81;                 % Acelera��o da gravidade [m/s^2]

            FzF = self.mF0 * g;       % Vertical load @ F [N]
            FzR = self.mR0 * g;       % Vertical load @ R [N]

            % Estados
            dPSI = estados(1);
            ALPHAT = estados(2);
            PSI = estados(3);
            v = estados(6);

            % Ângulos de deriva não linear
            ALPHAF = atan2((v * sin(ALPHAT) + a * dPSI), (v * cos(ALPHAT))) - deltaf; % Dianteiro
            ALPHAR = atan2((v * sin(ALPHAT) - b * dPSI), (v * cos(ALPHAT)));         % Traseiro

            % Forças longitudinais
            FxF = 0;
            FxR = 0;

            % Curva característica
            FyF = nF * self.tire.Characteristic(ALPHAF,FzF/nF,muy);
            FyR = nR * self.tire.Characteristic(ALPHAR,FzR/nR,muy);

            % Equações de estado
            dx(1,1) = (FyF * a * cos(deltaf) - FyR * b + FxF * a * sin(deltaf)) / I;
            dx(2,1) = (FyR + FyF * cos(deltaf) + FxF * sin(deltaf) - m*(dPSI * v * cos(ALPHAT) + (sin(ALPHAT)*(FxR + 	FxF * cos(deltaf) - FyF * sin(deltaf) +...
                      dPSI * m * v * sin(ALPHAT)))/(m * cos(ALPHAT))))/(m*(v * cos(ALPHAT) + (v * sin(ALPHAT)^2)/cos(ALPHAT)));
            dx(6,1) = (FxR * cos(ALPHAT) + FyR * sin(ALPHAT) - FyF * cos(ALPHAT)*sin(deltaf) + FyF * cos(deltaf)*sin(ALPHAT) + ...
                      FxF * sin(ALPHAT)*sin(deltaf) + FxF * cos(ALPHAT)*cos(deltaf))/(m * cos(ALPHAT)^2 + m * sin(ALPHAT)^2);

            % Obtenção da orientação
            dx(3,1) = dPSI; % dPSI

            % Equações adicionais para o posicionamento (Não necessárias para a dinâmica em guinada)
            dx(4,1) = v * cos(ALPHAT + PSI); % X
            dx(5,1) = v * sin(ALPHAT + PSI); % Y
        end
    end
end

%% See Also
%
% <index.html Index> | <VehicleArticulatedNonlinear4DOF.html VehicleArticulatedNonlinear4DOF>
%
