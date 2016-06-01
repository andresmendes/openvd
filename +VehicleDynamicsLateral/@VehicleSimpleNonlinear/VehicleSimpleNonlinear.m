classdef VehicleSimpleNonlinear < VehicleDynamicsLateral.VehicleSimple
    % VehicleSimpleNonlinear Nonlinear simple vehicle model.
    %
    % It inherits properties from VehicleSimple.

    methods
        % Constructor
        function self = VehicleSimpleNonlinear()
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
            DELTA = self.deltaf;

            g = 9.81;                 % Acelera��o da gravidade [m/s^2]

            FzF = self.mF0 * g;       % Vertical load @ F [N]
            FzR = self.mR0 * g;       % Vertical load @ R [N]

            % Estados
            PSI = estados(3);
            v = estados(4);
            ALPHAT = estados(5);
            dPSI = estados(6);

            % Slip angles
            ALPHAF = atan2((v * sin(ALPHAT) + a * dPSI), (v * cos(ALPHAT))) - DELTA; % Dianteiro
            ALPHAR = atan2((v * sin(ALPHAT) - b * dPSI), (v * cos(ALPHAT)));         % Traseiro

            % Longitudinal forces
            FxF = 0;
            FxR = 0;

            % Characteristic curve
            FyF = nF * self.tire.Characteristic(ALPHAF, FzF/nF, muy);
            FyR = nR * self.tire.Characteristic(ALPHAR, FzR/nR, muy);

            % Equations of motion
            dx(1,1) = v * cos(ALPHAT + PSI); % X
            dx(2,1) = v * sin(ALPHAT + PSI); % Y
            dx(3,1) = dPSI; % dPSI
            dx(4,1) = (FxF * cos(ALPHAT - DELTA) + FxR * cos(ALPHAT) + FyF * sin(ALPHAT - DELTA) + FyR * sin(ALPHAT))/(m);
            dx(5,1) = ( - FxF * sin(ALPHAT - DELTA) - FxR * sin(ALPHAT) + FyF * cos(ALPHAT - DELTA) + FyR * cos(ALPHAT) - m * v * dPSI) / (m * v);
            dx(6,1) = (FxF * a * sin(DELTA) + FyF * a * cos(DELTA) - FyR * b) / I;

        end
    end
end
