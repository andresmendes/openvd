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
clc                         % Clear command window

%% Doc generation

% exKalman
publish('ExKalman', 'outputDir', '../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', true);

% % DocGen
% publish('DocGen', 'outputDir', '../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', false);
%
% % Index
% publish('index', 'outputDir', '../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', false);
%
% % Templates
% publish('TemplateSimple', 'outputDir', '../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', true);
% publish('TemplateArticulated', 'outputDir', '../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', true);
%
% % Tire model
% publish('+VehicleDynamicsLateral/@Tire/Tire', 'outputDir', '../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', false);
%     publish('+VehicleDynamicsLateral/@TireLinear/TireLinear', 'outputDir', '../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', false);
%     publish('+VehicleDynamicsLateral/@TirePolynomial/TirePolynomial', 'outputDir', '../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', false);
%     publish('+VehicleDynamicsLateral/@TirePacejka/TirePacejka', 'outputDir', '../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', false);
% publish('TireComparison', 'outputDir', '../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', true);
%
% % Vehicle model
% publish('+VehicleDynamicsLateral/@VehicleSimple/VehicleSimple', 'outputDir', '../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', false);
%     publish('+VehicleDynamicsLateral/@VehicleSimpleLinear/VehicleSimpleLinear', 'outputDir', '../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', false);
%     publish('+VehicleDynamicsLateral/@VehicleSimpleNonlinear/VehicleSimpleNonlinear', 'outputDir', '../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', false);
% publish('+VehicleDynamicsLateral/@VehicleArticulated/VehicleArticulated', 'outputDir', '../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', false);
%     publish('+VehicleDynamicsLateral/@VehicleArticulatedNonlinear/VehicleArticulatedNonlinear', 'outputDir', '../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', false);
%
% % Graphics
% publish('+VehicleDynamicsLateral/@Graphics/Graphics', 'outputDir', '../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', false);

%% Code end
clear all                   % Clear workspace
close all                   % Closing figures
clc                         % Clear command window

%% See Also
%
% <index.html Index>
%
