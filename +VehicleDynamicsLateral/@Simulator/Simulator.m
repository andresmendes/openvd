classdef Simulator
	% Simulator Vehicle dynamics simulator
	% The simulator receives a vehicle object that inherits from VehicleSimple, simulates its behavior during a given time span and provides its behavior during time via its properties. Each property is a (timespan, 1) vector in which each value represents that parameter's value in time.
	methods
		function simulate = Simulate(self, t, states)
			% a ode45 deve ser chamada aqui e os parâmetros dos estados futuros devem ser disponibilizados através das properties

			% as condições iniciais pra veículos simples e articulados são diferentes, então elas podem ser passadas no objeto veículo
        end
	end

    properties
        dPSI % Yaw rate [rad/s]
        ALPHAT % Side slip angle [rad]
        PSI % Yaw angle [rad]
        XT % CG horizontal position [m]
        YT % CG vertical position [m]
        VEL % CG velocity [m/s]
	end
end
