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
				self.dPHI = XOUT(:, 7);
				self.PHI = XOUT(:, 8);
			else
				[TOUT, XOUT] = ode45(@(t, estados) self.Vehicle.Model(t, estados), self.TSpan, self.Vehicle.getInitialState());
			end

			% Retrieving states post integration
			self.dPSI = XOUT(:, 1);
			self.ALPHAT = XOUT(:, 2);
			self.PSI = XOUT(:, 3);
			self.XT = XOUT(:, 4);
			self.YT = XOUT(:, 5);
			self.VEL = XOUT(:, 6);
        end
	end

    properties
		Vehicle % Vehicle model to be used inthe simulation
		TSpan % a vector indicating the intervals in which the simulation steps will be conducted
        dPSI % Yaw rate [rad/s]
        ALPHAT % Side slip angle [rad]
        PSI % Yaw angle [rad]
        XT % CG horizontal position [m]
        YT % CG vertical position [m]
        VEL % CG velocity [m/s]
	end
end
