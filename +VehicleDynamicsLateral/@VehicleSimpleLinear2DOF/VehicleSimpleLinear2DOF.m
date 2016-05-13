%% Nonlinear 2 DOF Simple Vehicle
% Nonlinear bicycle model with 2 degrees of freedom.
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
% The center of gravity of the vehicle is located at the point $T$. The front and rear axles are located ate the points $F$ and $R$, respectively. The constant $a$ measures the distance of point $F$ to $T$ and $b$ the distance of point $T$ to $R$. The angles $\alpha_F$ e $\alpha_R$ are the front and rear slip angles, respectively. $\alpha_T$ is the vehicle side slip angle and $\psi$ is the vehicle yaw angle. $\delta$ is the steering angle.
%
% <<illustrations/modeloSimples.svg>>
%
%% Code
%

classdef VehicleSimpleLinear2DOF < VehicleDynamicsLateral.VehicleSimple
	methods
        function self = VehicleSimpleLinear2DOF(IT, lf, lr, mF0, mR0, deltaf, lT, nF, nR, wT, muy, tire)
        	% Constructor for the vehicle
	        self.IT = IT;
	        self.lf = lf;
	        self.lr = lr;

			self.mT = self.mF0 + self.mR0;
	        self.a = self.mR0 / self.mT * self.lT;
	        self.b = self.lT - self.a;

	        self.mF0 = lr * m / (lf + lr);
	        self.mR0 = lf * m / (lf + lr);
	        self.deltaf = deltaf;
	        self.lT = lf + lr;

	        self.nF = nR;
	        self.nR = nF;
	        self.wT = wT;
	        self.muy = muy;

	        self.params = [mF0 mR0 IT DELTA lT nF nR largT muy mT a b];
	        self.tire = tire;
        end

        %% Model
        % Function with the model
        function dx = Model(self, ~, estados)
            % Data
            m = self.mT;
            Iz = self.IT;
            lf = self.a;
            lr = self.b;
            nF = self.nF;
            nR = self.nR;
            muy = self.muy;
            deltaf = self.deltaf;

            g = 9.81;                 % Gravity [m/s^2]

			FzF = self.mF0 * g;       % Vertical load @ F [N]
            FzR = self.mR0 * g;       % Vertical load @ R [N]

			vx = 20;                  % [m/s]

            % States
            vy = estados(1);
            r = estados(2);
            PSI = estados(3);

            % Slip angles
            alphaf = - deltaf + (vy + lf * r) / vx;    % Front
            alphar = (vy - lr*r)/vx;                   % Rear

            % Lateral force
            Fyf = nF * self.tire.Characteristic(alphaf, FzF / nF, muy);
            Fyr = nR * self.tire.Characteristic(alphar, FzR / nR, muy);

            % State equations
            dvy = (Fyf * cos(deltaf) + Fyr - m * vx * r) / m;
            dr = (lf * Fyf * cos(deltaf) - lr * Fyr)/Iz;

            % State derivative
            dx(1,1) = dvy;
            dx(2,1) = dr;

            ALPHAT = asin(vy / vx);
            % Additional states for trajectory
            dx(3,1) = r; % dPSI
            dx(4,1) = vx * cos(ALPHAT + PSI); % X
            dx(5,1) = vx * sin(ALPHAT + PSI); % Y
        end
    end
end

%% See Also
%
% <index.html Index> | <VehicleArticulatedNonlinear4DOF.html VehicleArticulatedNonlinear4DOF>
%
