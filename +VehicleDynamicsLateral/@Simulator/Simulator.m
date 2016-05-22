classdef Simulator<handle
    % Simulator Vehicle dynamics simulator
    % The simulator receives a vehicle object that inherits from VehicleSimple, simulates its behavior during a given time span and provides its behavior during time via its properties. Each property is a (timespan, 1) vector in which each value represents that parameter's value in time.
    methods
        % Constructor
        function self = Simulator(vehicle, tspan)
            self.Vehicle = vehicle;
            self.TSpan = tspan;
            if isa(self.Vehicle, 'VehicleDynamicsLateral.VehicleArticulated')
                self.X0 = 0;                     % Initial tractor CG horizontal position [m]
                self.Y0 = 0;                     % Initial tractor CG vertical position [m]
                self.PSI0 = 0;                   % Initial tractor yaw angle [rad]
                self.PHI0 = 0;                   % Initial articulation angle [rad]
                self.V0 = 20;                    % Initial tractor CG velocity [m/s]
                self.ALPHAT0 = 0.3;              % Initial tractor side slip angle [rad]
                self.dPSI0 = 0.25;               % Initial tractor yaw rate [rad/s]
                self.dPHI0 = self.dPSI0;        % Initial articulation rate [rad/s]
            else
                self.X0 = 0;                     % Initial CG horizontal position [m]
                self.Y0 = 0;                     % Initial CG vertical position [m]
                self.PSI0 = 0;                   % Initial yaw angle [rad]
                self.V0 = 20;                    % Initial CG velocity [m/s]
                self.ALPHAT0 = -0.2;             % Initial side slip angle [rad]
                self.dPSI0 = 0.7;                % Initial yaw rate [rad/s]
            end
        end

        function f = getInitialState(self)
            % Transforms properties into a vector so it can be used by the integrator
            if isa(self.Vehicle, 'VehicleDynamicsLateral.VehicleArticulated')
                f = [self.X0 self.Y0 self.PSI0 self.PHI0 self.V0 self.ALPHAT0 self.dPSI0 self.dPHI0];
            else
                f = [self.X0 self.Y0 self.PSI0 self.V0 self.ALPHAT0 self.dPSI0];
            end


        end

        function Simulate(self)
            % TODO: gravity can be passed to the simulator so vertical load and other forces are calculated here

            % integration
            % if vehicle is articulated, adds mass matrix as an integration option
            if isa(self.Vehicle, 'VehicleDynamicsLateral.VehicleArticulated')
                fun = self.Vehicle;
                options = odeset('Mass', @fun.MassMatrix);
                [TOUT, XOUT] = ode45(@(t, estados) self.Vehicle.Model(t, estados), self.TSpan, self.getInitialState(), options);

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
                [TOUT, XOUT] = ode45(@(t, estados) self.Vehicle.Model(t, estados), self.TSpan, self.getInitialState());

                % Retrieving states post integration
                self.XT = XOUT(:, 1);
                self.YT = XOUT(:, 2);
                self.PSI = XOUT(:, 3);
                self.VEL = XOUT(:, 4);
                self.ALPHAT = XOUT(:, 5);
                self.dPSI = XOUT(:, 6);
            end

            % TSpan and TOUT contain the same values, but the first is passed in columns, while the second is a vector
            self.TSpan = TOUT;
        end
    end

    properties
        Vehicle % Vehicle model to be used inthe simulation
        TSpan % a vector indicating the intervals in which the simulation steps will be conducted
        X0 % Initial CG horizontal position [m]
        Y0 % Initial CG vertical position [m]
        PSI0 % Initial yaw angle [rad]
        PHI0 % Initial articulation angle [rad]
        V0 % Initial CG velocity [m/s]
        ALPHAT0 % Initial side slip angle [rad]
        dPSI0 % Initial yaw rate [rad/s]
        dPHI0 % Initial articulation rate [rad/s]
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
