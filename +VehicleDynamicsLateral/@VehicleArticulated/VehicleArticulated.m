%% Ve�culo articulado (Abstract)
%

classdef (Abstract) VehicleArticulated  < VehicleDynamicsLateral.VehicleSimple
    % VehicleArticulated Articulated vehicle abstract class. It inherits properties from VehicleSimple and has additional properties related to the semitrailer.

	methods(Abstract)
		Model(self, t, estados)

		function f = getInitialState()
			% Transforms properties into a vector so it can be used by the integrator
			return [self.dPSI0 self.ALPHAT0 self.PSI0 self.X0 self.Y0 self.V0 self.dPHI0 self.PHI0];
		end
	end

    properties
        nM % Number of tires on semitrailer axle
        wS % semitrailer width [m]
        lS % Distance from joint to semitrailer axle [m]
        c % Distance from joint to rear axle of the tractor (A-R) [m]
        dPHI0 % Initial articulation rate [rad/s]
        PHI0 % Initial articulation angle [rad]
	end
end

%% See Also
%
% <index.html Index> | <VehicleSimple.html VehicleSimple>
%
