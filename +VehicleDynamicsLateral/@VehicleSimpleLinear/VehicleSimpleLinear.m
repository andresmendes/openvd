classdef VehicleSimpleLinear < VehicleDynamicsLateral.VehicleSimple
    % VehicleSimpleLinear Linear simple vehicle model.
    %
    % It inherits properties from VehicleSimple.

    methods
        function self = VehicleSimpleLinear()
            % Constructor for the vehicle
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
        % Function with the model
        function dx = Model(self, ~, estados)
            % Data
            mT = self.mT;
            IT = self.IT;
            a = self.a;
            b = self.b;
            nF = self.nF;
            nR = self.nR;
            muy = self.muy;
            deltaf = self.deltaf;

            g = 9.81;                 % Gravity [m/s^2]

            FzF = self.mF0 * g;       % Vertical load @ F [N]
            FzR = self.mR0 * g;       % Vertical load @ R [N]

            v0 = 20;                  % [m/s]

            % State variables
            % X = estados(1,1);         % Not used
            % Y = estados(2,1);         % Not used
            PSI     = estados(3,1);
            VT       = estados(4,1);
            ALPHAT  = estados(5,1);
            dPSI    = estados(6,1);

            % Slip angles
            ALPHAF = ALPHAT + a/v0*dPSI - deltaf;
            ALPHAR = ALPHAT - b/v0*dPSI;

            % Longitudinal forces
            FxF = 0;
            FxR = 0;

            % Lateral force
            FyF = nF * self.tire.Characteristic(ALPHAF, FzF / nF, muy);
            FyR = nR * self.tire.Characteristic(ALPHAR, FzR / nR, muy);

            % State equations
            dx(1,1) = VT;
            dx(2,1) = v0*(PSI + ALPHAT);
            dx(3,1) = dPSI;
            dx(4,1) = (FxF + FxR)/mT;
            dx(5,1) = (FyF + FyR)/(mT*v0) - dPSI;
            dx(6,1) = (a*FyF - b*FyR)/IT;

        end
    end
end
