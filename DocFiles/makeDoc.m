%% Documentation Generator
%
% This script generates the whole documentation of the package.
%
%% Description
%
% The documentation of the package is written within the code of all .m files as comments.
%
% Running this script all the .html files are generated through the command <http://www.mathworks.com/help/matlab/ref/publish.html publish> and saved in the folder "../Vehicle-Dynamics-Lateral-Documentation/".
%
%% Code start

clear all                   % Clear workspace
close all                   % Closing figures
clc

docPath = '../Documentation/html/';                      % Folder where html doc files are saved
apiDocPath = strcat(docPath, 'api/');

%% Adding paths
% Adding the folder of all files to the Matlab path (only the ones that 'evalCode' is true)
%

% Examples
addpath('../Examples/KalmanFilter/')
addpath('../Examples/SimulinkApplication/')
addpath('../Examples/SinusoidalSteering/')
addpath('../Examples/SkidPad/')
addpath('../Examples/SkidPad4DOF/')
addpath('../Examples/SteeringControl/')
addpath('../Examples/TemplateArticulated/')
addpath('../Examples/TemplateSimple/')
addpath('../Examples/TireComparison/')

%% Publishing documentation
%

% Index
publish('../DocFiles/index.m', 'outputDir', '../Documentation', 'evalCode', true,'showCode',false);

% API
publish('../DocFiles/api.m', 'outputDir', apiDocPath,'evalCode', false);

% Tire models
publish('../DocFiles/DocTireLinear.m', 'outputDir', docPath, 'evalCode', false);
publish('../DocFiles/DocTirePolynomial.m', 'outputDir', docPath, 'evalCode', false);
publish('../DocFiles/DocTirePacejka.m', 'outputDir', docPath, 'evalCode', false);

% Vehicle models
publish('../DocFiles/DocVehicleSimpleLinear.m', 'outputDir', docPath, 'evalCode', false);
publish('../DocFiles/DocVehicleSimpleNonlinear.m', 'outputDir', docPath, 'evalCode', false);
publish('../DocFiles/DocVehicleSimpleNonlinear4DOF.m', 'outputDir', docPath, 'evalCode', false);
publish('../DocFiles/DocVehicleArticulatedLinear.m', 'outputDir', docPath, 'evalCode', false);
publish('../DocFiles/DocVehicleArticulatedNonlinear.m', 'outputDir', docPath, 'evalCode', false);

% Graphics
publish('../DocFiles/DocGraphics.m', 'outputDir', docPath, 'evalCode', false);

% API
publish('../+VehicleDynamicsLateral/@VehicleSimple/VehicleSimple.m', 'outputDir', apiDocPath, 'evalCode', false);
publish('../+VehicleDynamicsLateral/@VehicleArticulated/VehicleArticulated.m', 'outputDir', apiDocPath, 'evalCode', false);

publish('../+VehicleDynamicsLateral/@VehicleSimpleLinear/VehicleSimpleLinear.m', 'outputDir', apiDocPath, 'evalCode', false);
publish('../+VehicleDynamicsLateral/@VehicleSimpleNonlinear/VehicleSimpleNonlinear.m', 'outputDir', apiDocPath, 'evalCode', false);
publish('../+VehicleDynamicsLateral/@VehicleSimpleNonlinear4DOF/VehicleSimpleNonlinear4DOF.m', 'outputDir', apiDocPath, 'evalCode', false);
publish('../+VehicleDynamicsLateral/@VehicleArticulatedLinear/VehicleArticulatedLinear.m', 'outputDir', apiDocPath, 'evalCode', false);
publish('../+VehicleDynamicsLateral/@VehicleArticulatedNonlinear/VehicleArticulatedNonlinear.m', 'outputDir', apiDocPath, 'evalCode', false);

publish('../+VehicleDynamicsLateral/@Tire/Tire.m', 'outputDir', apiDocPath, 'evalCode', false);
publish('../+VehicleDynamicsLateral/@TireLinear/TireLinear.m', 'outputDir', apiDocPath, 'evalCode', false);
publish('../+VehicleDynamicsLateral/@TirePolynomial/TirePolynomial.m', 'outputDir', apiDocPath, 'evalCode', false);
publish('../+VehicleDynamicsLateral/@TirePacejka/TirePacejka.m', 'outputDir', apiDocPath, 'evalCode', false);

publish('../+VehicleDynamicsLateral/@Simulator/Simulator.m', 'outputDir', apiDocPath, 'evalCode', false);

publish('../+VehicleDynamicsLateral/@Graphics/Graphics.m', 'outputDir', apiDocPath, 'evalCode', false);

% Examples
% TemplateSimple
publish('TemplateSimple.m', 'outputDir', docPath, 'evalCode', true);
g.Animation('../Documentation/illustrations/AnimationSimple');
close all
clearvars -except docPath apiDocPath

% TemplateArticulated
publish('TemplateArticulated.m', 'outputDir', docPath, 'evalCode', true);
g.Animation('../Documentation/illustrations/AnimationArticulated');
close all
clearvars -except docPath apiDocPath

% Simulink Application
publish('VehicleSystem.m', 'outputDir', docPath, 'evalCode', false);
publish('SimulinkApplication.m', 'outputDir', docPath, 'evalCode', true);
g.Animation('../Documentation/illustrations/AnimationSimulinkApplication');
close all
clearvars -except docPath apiDocPath

% Sinusoidal Steering
publish('SinusoidalSteering.m', 'outputDir', docPath, 'evalCode', true);
g.Animation('../Documentation/illustrations/AnimationSinusoidal');
close all
clearvars -except docPath apiDocPath

% Kalman Filter
publish('KalmanFilter.m', 'outputDir', docPath, 'evalCode', true);
close all
clearvars -except docPath apiDocPath

% TireComparison
publish('TireComparison.m', 'outputDir', docPath, 'evalCode', true);
close all
clearvars -except docPath apiDocPath

% SteeringControl
publish('SteeringControl.m', 'outputDir', docPath, 'evalCode', true,'showCode',false);
g.Animation('../Documentation/illustrations/SteeringControlAnimation');
close all
clearvars -except docPath apiDocPath

% Skid Pad
publish('SkidPad.m', 'outputDir', docPath, 'evalCode', true,'showCode',true);
g.Animation('../Documentation/illustrations/AnimationSkidPad');
close all
clearvars -except docPath apiDocPath

% Skid Pad 4DOF
publish('SkidPad4DOF.m', 'outputDir', docPath, 'evalCode', true,'showCode',true);
g.Animation('../Documentation/illustrations/AnimationSkidPad4DOF');
close all
clearvars -except docPath apiDocPath

% DocGen
publish('makeDoc', 'outputDir', docPath, 'evalCode', false);

%% Code end
clear all                   % Clear workspace
close all                   % Closing figures
clc                         % Clear command window

%% See Also
%
% <../index.html index>
%
