%% Ve�culo simples (Abstract)
%

classdef (Abstract) VehicleSimple
	methods(Abstract)
		Model(self, t, estados)
	end

    properties
		params
        tire
        distFT
        distTR
        width
	end
end

%% See Also
%
% <index.html Index> | <VehicleArticulated.html VehicleArticulated>
%
