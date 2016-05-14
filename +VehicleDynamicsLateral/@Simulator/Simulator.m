classdef Simulator
	% Simulator Vehicle dynamics simulator
	% The simulator receives a vehicle object that inherits from VehicleSimple, simulates its behavior during a given time span and provides its behavior during time via its properties. Each property is a (timespan, 1) vector in which each value represents that parameter's value in time.
	methods
        % Constructor
        function self = TireLinear(vehicle, tspan)
                self.Vehicle = vehicle;
                self.TSpan = tspan;
            end
        end

		function simulate = Simulate(self)

			% TODO: gravity can be passed to the simulator so vertical load and other forces are calculated here

			% integration
			% if vehicle is articulated, adds mass matrix as an integration option
			if isa(self.Vehicle, VehicleDynamicsLateral.VehicleArticulated)
				options = odeset('Mass', @System.MassMatrix);
				[TOUT,XOUT] = ode45(@(t, estados) System.Model(t, estados), self.TSpan, self.Vehicle.getInitialState(), options);

				% retrieve states exclusive to the articulated vehicle
				self.dPHI = XOUT(:, 7);           % Articulation rate [rad/s]
				self.PHI = XOUT(:, 8);            % Articulation angle [rad]
			else
				[TOUT, XOUT] = ode45(@(t, estados) System.Model(t, estados), self.TSpan, self.Vehicle.getInitialState());

			% Retrieving states post integration
			self.dPSI = XOUT(:, 1);           % Yaw rate [rad/s]
			self.ALPHAT = XOUT(:, 2);         % Side slip angle [rad]
			self.PSI = XOUT(:, 3);            % Yaw angle [rad]
			self.XT = XOUT(:, 4);             % CG horizontal position [m]
			self.YT = XOUT(:, 5);             % CG vertical position [m]
			self.VEL = XOUT(:, 6);            % CG velocity [m/s]
        end
	end

    properties
		Vehicle % Vehiclemodel to be used inthe simulation
		TSpan % a vector indicating the intervals in which the simulation steps will be conducted
        dPSI % Yaw rate [rad/s]
        ALPHAT % Side slip angle [rad]
        PSI % Yaw angle [rad]
        XT % CG horizontal position [m]
        YT % CG vertical position [m]
        VEL % CG velocity [m/s]
	end
end
