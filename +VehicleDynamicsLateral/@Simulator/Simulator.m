classdef Simulator
	% Simulator Vehicle dynamics simulator
	% The simulator receives a vehicle object that inherits from VehicleSimple, simulates its behavior during a given time span and provides its behavior during time via its properties. Each property is a (timespan, 1) vector in which each value represents that parameter's value in time.
	methods
		function simulate = Simulate(self, vehicle, tspan)

			% TODO: gravity can be passed to the simulator so vertical load and other forces are calculated here

			% integration
			% if vehicle is articulated, adds mass matrix as an integration option
			if isa(vehicle, VehicleDynamicsLateral.VehicleArticulated)
				options = odeset('Mass', @System.MassMatrix);
				[TOUT,XOUT] = ode45(@(t, estados) System.Model(t, estados), tspan, vehicle.getInitialState(), options);
			else
				[TOUT, XOUT] = ode45(@(t, estados) System.Model(t, estados), tspan, vehicle.getInitialState());

			% Retrieving states post integration
			self.dPSI = XOUT(:, 1);           % Yaw rate [rad/s]
			self.ALPHAT = XOUT(:, 2);         % Side slip angle [rad]
			self.PSI = XOUT(:, 3);            % Yaw angle [rad]
			self.XT = XOUT(:, 4);             % CG horizontal position [m]
			self.YT = XOUT(:, 5);             % CG vertical position [m]
			self.VEL = XOUT(:, 6);            % CG velocity [m/s]
			self.dPHI = XOUT(:, 7);           % Articulation rate [rad/s]
			self.PHI = XOUT(:, 8);            % Articulation angle [rad]
        end
	end

    properties
        dPSI % Yaw rate [rad/s]
        ALPHAT % Side slip angle [rad]
        PSI % Yaw angle [rad]
        XT % CG horizontal position [m]
        YT % CG vertical position [m]
        VEL % CG velocity [m/s]
	end
end
