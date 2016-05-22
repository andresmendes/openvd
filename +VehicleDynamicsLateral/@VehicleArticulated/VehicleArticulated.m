%% Ve�culo articulado (Abstract)
%

classdef (Abstract) VehicleArticulated  < VehicleDynamicsLateral.VehicleSimple
    % VehicleArticulated Articulated vehicle abstract class. It inherits properties from VehicleSimple and has additional properties related to the semitrailer.

    methods(Abstract)
        % MassMatrix(self, t, estados)
    end

    properties
        mF
        mR
        mM
        IS
        nM % Number of tires on semitrailer axle
        wS % semitrailer width [m]
        lS % Distance from joint to semitrailer axle [m]
        c % Distance from joint to rear axle of the tractor (A-R) [m]
        d
        e
        A
        mS
    end

    methods

        function value = get.A(self)
            value = self.mF * self.g + self.mR * self.g - self.mT * self.g;
        end

        function value = get.mS(self)
            value = (self.A + self.mM * self.g)/self.g;
        end

        function value = get.d(self)
            value = (self.lS * self.mM) / self.mS;
        end

        function value = get.e(self)
            value = self.lS - self.d;
        end
    end
end

%% See Also
%
% <index.html Index> | <VehicleSimple.html VehicleSimple>
%
