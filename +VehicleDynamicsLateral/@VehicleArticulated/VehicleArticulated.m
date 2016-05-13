%% Ve�culo articulado (Abstract)
%

classdef (Abstract) VehicleArticulated  < VehicleDynamicsLateral.VehicleSimple
    % VehicleArticulated Articulated vehicle abstract class. It inherits properties from VehicleSimple and has additional properties related to the semitrailer.
    properties
        nM % Number of tires on semitrailer axle
        wS % semitrailer width [m]
        lS % Distance from joint to semitrailer axle [m]
        c % Distance from joint to rear axle of the tractor (A-R) [m]
        dPHI0 % Initial articulation rate [rad/s]
        VEL0 % Initial tractor CG velocity [m/s]
        PHI0 % Initial articulation angle [rad]
	end
end

%% See Also
%
% <index.html Index> | <VehicleSimple.html VehicleSimple>
%
