classdef SolverTest < matlab.unittest.TestCase
    % SolverTest tests solutions to the quadratic equation
    % a*x^2 + b*x + c = 0

    methods (Test)
        function testVehicleSimple(testCase)
            % Simulation time
            T = 6;                      % Total simulation time [s]
            resol = 50;                 % Resolution
            TSPAN = 0:T/resol:T;        % Time span [s]

            %% Tire parameters
            % Chosen tire: <TirePacejka1989.html TirePacejka1989.m>.
            %

            a0 = 1.002806;
            a1 = 2.014156;
            a2 = 710.5013;
            a3 = 5226.341;
            a4 = 78.87699;
            a5 = 0.01078379;
            a6 = -0.004759443;
            a7 = 0.6704447;
            a8 = 0;
            a9 = 0;
            a10 = 0;
            a11 = 0;
            a12 = 0;
            a13 = 0;
            TireModel = VehicleDynamicsLateral.TirePacejka1989(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13);

            %% Vehicle parameters
            % Chosen Vehicle: <VehicleSimpleNonlinear3DOF.html VehicleSimpleNonlinear3DOF.m>.

            mF0 = 700;                  % mass over front axle [kg]
            mR0 = 600;                  % mass over rear axle [kg]
            IT = 10000;                 % moment of inertia [kg*m2]
            deltaf = 0;                 % front axle steering [rad]
            lT = 3.550;                 % distance between axles [m]
            nF = 2;                     % number of tires in the front axle
            nR = 2;                     % number of tires in the rear axle
            wT = 2;                     % width [m]
            muy = 0.8;                  % coefficient of operation friction
            System = VehicleDynamicsLateral.VehicleSimpleNonlinear3DOF(IT, mF0, mR0, deltaf, lT, nF, nR, wT, muy, TireModel);

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
