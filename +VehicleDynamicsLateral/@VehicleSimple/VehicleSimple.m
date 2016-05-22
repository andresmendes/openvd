classdef (Abstract) VehicleSimple
    % VehicleSimple Tractor vehicle without semitrailer
    % Abstract class representing a tractor vehicle without semitrailer. Extend this class in order to create a new vehicle model to be used with the simulator.

    methods(Abstract)
        Model(self, t, estados)
    end

    properties
        mT % Vehicle total mass [kg]
        IT % Moment of inertia [kg * m2]
        a % [m]
        b % [m]
        g % Gravity
        mF0 % Vehicle frontal mass [kg]
        mR0 % Vehicle rear mass [kg]
        lT % [m]
        nF % Number of front tires
        nR % Number of rear tires
        deltaf % Steering angle [rad]
        wT % Width [m]
        muy % Operational friction coefficient
        tire % Tire model
    end

    methods

        function value = get.mT(self)
            value = self.mF0 + self.mR0;
        end

        function value = get.a(self)
            value = self.mR0 / self.mT * self.lT;
        end

        function value = get.b(self)
            value = self.lT - self.a;
        end
    end

end

%% See Also
%
% <index.html Index> | <VehicleArticulated.html VehicleArticulated>
%
