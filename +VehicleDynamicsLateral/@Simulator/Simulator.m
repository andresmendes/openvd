classdef Simulator<handle
	% Simulator Vehicle dynamics simulator
	% The simulator receives a vehicle object that inherits from VehicleSimple, simulates its behavior during a given time span and provides its behavior during time via its properties. Each property is a (timespan, 1) vector in which each value represents that parameter's value in time.
	methods
        % Constructor
        function self = Simulator(vehicle, tspan)
            self.Vehicle = vehicle;
            self.TSpan = tspan;
        end

		function Simulate(self)
			% TODO: gravity can be passed to the simulator so vertical load and other forces are calculated here

			% integration
			% if vehicle is articulated, adds mass matrix as an integration option
			if isa(self.Vehicle, 'VehicleDynamicsLateral.VehicleArticulated')
				options = odeset('Mass', @self.Vehicle.MassMatrix);
				[TOUT,XOUT] = ode45(@(t, estados) self.Vehicle.Model(t, estados), self.TSpan, self.Vehicle.getInitialState(), options);

				% retrieve states exclusive to the articulated vehicle
                self.XT = XOUT(:, 1);
                self.YT = XOUT(:, 2);
                self.PSI = XOUT(:, 3);
				self.PHI = XOUT(:, 4);
                self.VEL = XOUT(:, 5);
                self.ALPHAT = XOUT(:, 6);
                self.dPSI = XOUT(:, 7);
				self.dPHI = XOUT(:, 8);
			else
				[TOUT, XOUT] = ode45(@(t, estados) self.Vehicle.Model(t, estados), self.TSpan, self.Vehicle.getInitialState());

                % Retrieving states post integration
    			self.XT = XOUT(:, 1);
    			self.YT = XOUT(:, 2);
    			self.PSI = XOUT(:, 3);
    			self.VEL = XOUT(:, 4);
    			self.ALPHAT = XOUT(:, 5);
    			self.dPSI = XOUT(:, 6);
			end

            % Graphics
            G = VehicleDynamicsLateral.Graphics(self.Vehicle);
            G.Frame([self.XT self.YT self.PSI self.VEL self.ALPHAT self.dPSI],TOUT,0);
            G.Animation([self.XT self.YT self.PSI self.VEL self.ALPHAT self.dPSI],TOUT,0);

        end
	end

    properties
		Vehicle % Vehicle model to be used inthe simulation
		TSpan % a vector indicating the intervals in which the simulation steps will be conducted
        XT % CG horizontal position [m]
        YT % CG vertical position [m]
        PSI % Yaw angle [rad]
        PHI % Relative yaw angle [rad]
        VEL % CG velocity [m/s]
        ALPHAT % Side slip angle [rad]
        dPSI % Yaw rate [rad/s]
        dPHI % Relative yaw rate [rad/s]
	end
end
