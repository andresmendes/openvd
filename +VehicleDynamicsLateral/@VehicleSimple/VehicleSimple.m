%% Ve�culo simples (Abstract)
%

classdef (Abstract) VehicleSimple
	methods(Abstract)
		Model(self, t, estados)
	end

    properties
		params
		IT
		lf
		lr
		mF0
		mR0
		deltaf
		lT
		nF
		nR
		wT
		muy
		tire
	end
end

%% See Also
%
% <index.html Index> | <VehicleArticulated.html VehicleArticulated>
%
