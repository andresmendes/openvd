classdef Tests < matlab.unittest.TestCase
    % SolverTest tests solutions to the quadratic equation
    % a * x^2 + b * x + c = 0

    methods (Test)
        function testVehicleSimple(testCase)
            % Simulation time
            T = 6;                      % Total simulation time [s]
            resol = 50;                 % Resolution
            TSPAN = 0:T/resol:T;        % Time span [s]

            %% Tire parameters
            % Chosen tire: <TirePacejka1989.html TirePacejka1989.m>.
            %

            TireModel = VehicleDynamicsLateral.TirePacejka1989();

            %% Vehicle parameters
            % Chosen Vehicle: <VehicleSimpleNonlinear3DOF.html VehicleSimpleNonlinear3DOF.m>.

            System = VehicleDynamicsLateral.VehicleSimpleNonlinear3DOF();
            System.tire = TireModel;

            % Initial conditions
            System.dPSI0 = 0.7;                % Initial yaw rate [rad/s]
            System.ALPHAT0 = -0.2;             % Initial side slip angle [rad]
            System.PSI0 = 0;                   % Initial yaw angle [rad]
            System.X0 = 0;                     % Initial CG horizontal position [m]
            System.Y0 = 0;                     % Initial CG vertical position [m]
            System.V0 = 20;                    % Initial CG velocity [m/s]

            simulator = VehicleDynamicsLateral.Simulator(System, TSPAN);
            simulator.Simulate();

            % Retrieving states
            dPSI = simulator.dPSI;
            ALPHAT = simulator.ALPHAT;
            PSI = simulator.PSI;
            XT = simulator.XT;
            YT = simulator.YT;
            VEL = simulator.VEL;
        end
    end
end
