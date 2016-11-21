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

addpath('/home/andre/Repos/Vehicle-Dynamics/Vehicle-Dynamics-Lateral')

docPath = '../../Vehicle-Dynamics-Lateral-Documentation/';
apiDocPath = strcat(docPath, 'api/');

%% Index
publish('../index.m', 'outputDir', docPath, 'evalCode', false);
publish('../api.m', 'outputDir', apiDocPath, 'evalCode', false);

%% Tire models
publish('DocTireLinear.m', 'outputDir', docPath, 'evalCode', false);
publish('DocTirePolynomial.m', 'outputDir', docPath, 'evalCode', false);
publish('DocTirePacejka.m', 'outputDir', docPath, 'evalCode', false);

%% Vehicle models
publish('DocVehicleSimpleLinear.m', 'outputDir', docPath, 'evalCode', false);
publish('DocVehicleSimpleNonlinear.m', 'outputDir', docPath, 'evalCode', false);
publish('DocVehicleSimpleNonlinear4DOF.m', 'outputDir', docPath, 'evalCode', false);
publish('DocVehicleArticulatedLinear.m', 'outputDir', docPath, 'evalCode', false);
publish('DocVehicleArticulatedNonlinear.m', 'outputDir', docPath, 'evalCode', false);

%% Graphics
publish('DocGraphics.m', 'outputDir', docPath, 'evalCode', false);

cd ../+VehicleDynamicsLateral/

% API
publish('@VehicleSimple/VehicleSimple.m', 'outputDir', apiDocPath, 'evalCode', false);
publish('@VehicleArticulated/VehicleArticulated.m', 'outputDir', apiDocPath, 'evalCode', false);

publish('@VehicleSimpleLinear/VehicleSimpleLinear.m', 'outputDir', apiDocPath, 'evalCode', false);
publish('@VehicleSimpleNonlinear/VehicleSimpleNonlinear.m', 'outputDir', apiDocPath, 'evalCode', false);
publish('@VehicleSimpleNonlinear4DOF/VehicleSimpleNonlinear4DOF.m', 'outputDir', apiDocPath, 'evalCode', false);
publish('@VehicleArticulatedLinear/VehicleArticulatedLinear.m', 'outputDir', apiDocPath, 'evalCode', false);
publish('@VehicleArticulatedNonlinear/VehicleArticulatedNonlinear.m', 'outputDir', apiDocPath, 'evalCode', false);

publish('@Tire/Tire.m', 'outputDir', apiDocPath, 'evalCode', false);
publish('@TireLinear/TireLinear.m', 'outputDir', apiDocPath, 'evalCode', false);
publish('@TirePolynomial/TirePolynomial.m', 'outputDir', apiDocPath, 'evalCode', false);
publish('@TirePacejka/TirePacejka.m', 'outputDir', apiDocPath, 'evalCode', false);

publish('@Simulator/Simulator.m', 'outputDir', apiDocPath, 'evalCode', false);

publish('@Graphics/Graphics.m', 'outputDir', apiDocPath, 'evalCode', false);

cd ../Examples/TemplateSimple
%% Examples
% TemplateSimple
publish('TemplateSimple.m', 'outputDir', '../../../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', true);
close all
g.Animation('../../../Vehicle-Dynamics-Lateral-Documentation/illustrations/AnimationSimple');

cd ../TemplateArticulated
%%
% TemplateArticulated
publish('TemplateArticulated.m', 'outputDir', '../../../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', true);
close all
g.Animation('../../../Vehicle-Dynamics-Lateral-Documentation/illustrations/AnimationArticulated');

cd ../SinusoidalSteering
%%
% Sinusoidal Steering
publish('SinusoidalSteering.m', 'outputDir', '../../../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', true);
close all
g.Animation('../../../Vehicle-Dynamics-Lateral-Documentation/illustrations/AnimationSinusoidal');

cd ../KalmanFilter
%%
% Kalman Filter
publish('KalmanFilter.m', 'outputDir', '../../../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', true);

cd ../TireComparison
%%
% TireComparison
publish('TireComparison.m', 'outputDir', '../../../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', true);

cd ../SteeringControl
%%
% SteeringControl
publish('SteeringControl.m', 'outputDir', '../../../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', true,'showCode',false);
close all
g.Animation('../../../Vehicle-Dynamics-Lateral-Documentation/illustrations/SteeringControlAnimation');

cd ../SkidPad
%%
% Skid Pad
publish('SkidPad.m', 'outputDir', '../../../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', true,'showCode',true);
close all
g.Animation('../../../Vehicle-Dynamics-Lateral-Documentation/illustrations/AnimationSkidPad');

cd ../SkidPad4DOF
%%
% Skid Pad
publish('SkidPad4DOF.m', 'outputDir', '../../../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', true,'showCode',true);
close all
g.Animation('../../../Vehicle-Dynamics-Lateral-Documentation/illustrations/AnimationSkidPad4DOF');


% % DocGen
% publish('DocGen', 'outputDir', docPath, 'evalCode', false);

cd ../../Documentation

%% Code end
clear all                   % Clear workspace
close all                   % Closing figures
clc                         % Clear command window

%% See Also
%
% <index.html Index>
%
