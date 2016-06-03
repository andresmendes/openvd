classdef TireLinear < VehicleDynamicsLateral.Tire
    % TireLinear Linear tire model
    %
    % It inherits methods from Tire.

    methods
        % Constructor
        function self = TireLinear()
            self.k = 40000;
        end

        function Fy = Characteristic(self, alpha, varargin)
            Fy = -self.k * alpha;
        end
    end

    properties
        k % Cornering stiffness [N/rad]
    end

end
