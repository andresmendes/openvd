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

%% Index
publish('../index.m', 'outputDir', '../../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', false);

%% Tire models
publish('DocTireLinear.m', 'outputDir', '../../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', false);
publish('DocTirePolynomial.m', 'outputDir', '../../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', false);
publish('DocTirePacejka.m', 'outputDir', '../../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', false);

%% Vehicle models
publish('DocVehicleSimpleLinear.m', 'outputDir', '../../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', false);
publish('DocVehicleSimpleNonlinear.m', 'outputDir', '../../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', false);
publish('DocVehicleArticulatedLinear.m', 'outputDir', '../../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', false);
publish('DocVehicleArticulatedNonlinear.m', 'outputDir', '../../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', false);

%% Graphics
publish('DocGraphics.m', 'outputDir', '../../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', false);

cd ../Examples/TemplateSimple
%% Examples
% TemplateSimple
publish('TemplateSimple.m', 'outputDir', '../../../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', true);

cd ../TemplateArticulated
%%
% TemplateArticulated
publish('TemplateArticulated/TemplateArticulated.m', 'outputDir', '../../../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', true);

cd ../KalmanFilter
%%
% Kalman Filter
publish('KalmanFilter/KalmanFilter.m', 'outputDir', '../../../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', true);

cd ../TireComparison
%%
% TireComparison
publish('TireComparison/TireComparison.m', 'outputDir', '../../../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', false);

% % DocGen
% publish('DocGen', 'outputDir', '../../Vehicle-Dynamics-Lateral-Documentation/', 'evalCode', false);

cd ../../Documentation

%% Code end
clear all                   % Clear workspace
close all                   % Closing figures
clc                         % Clear command window

%% See Also
%
% <index.html Index>
%
