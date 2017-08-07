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

% Going to openvd root
cd ../..
% Getting openvd path
openvdPath = pwd;
% Going back to DocFiles
cd docs/DocFiles
% Folder where html doc files are saved
docPath = strcat(openvdPath,'/docs/html/');
apiDocPath = strcat(openvdPath,'/docs/html/api/');

%% Adding paths
% Adding the folder of all necessary files to the Octave/Matlab path

% Path main.m function
addpath(strcat(openvdPath,'/docs/DocFiles'))

% Path of the publishOVD function
addpath(strcat(openvdPath,'/docs/DocFiles/publishOVD'))

% Path of the package
addpath(strcat(openvdPath,'/inst/'))

% API
addpath(strcat(openvdPath,'/inst/@VehicleSimple/'))
addpath(strcat(openvdPath,'/inst/@VehicleArticulated/'))
addpath(strcat(openvdPath,'/inst/@VehicleSimpleLinear/'))
addpath(strcat(openvdPath,'/inst/@VehicleSimpleNonlinear/'))
addpath(strcat(openvdPath,'/inst/@VehicleSimpleNonlinear4DOF/'))
addpath(strcat(openvdPath,'/inst/@VehicleArticulatedLinear/'))
addpath(strcat(openvdPath,'/inst/@VehicleArticulatedNonlinear/'))
addpath(strcat(openvdPath,'/inst/@Tire/'))
addpath(strcat(openvdPath,'/inst/@TireLinear/'))
addpath(strcat(openvdPath,'/inst/@TirePolynomial/'))
addpath(strcat(openvdPath,'/inst/@TirePacejka/'))
addpath(strcat(openvdPath,'/inst/@Simulator/'))
addpath(strcat(openvdPath,'/inst/@Graphics/'))

% % Examples
addpath(strcat(openvdPath,'/docs/Examples/KalmanFilter/'))
addpath(strcat(openvdPath,'/docs/Examples/SinusoidalSteering/'))
addpath(strcat(openvdPath,'/docs/Examples/SkidPad/'))
addpath(strcat(openvdPath,'/docs/Examples/SkidPad4DOF/'))
addpath(strcat(openvdPath,'/docs/Examples/SteeringControl/'))
addpath(strcat(openvdPath,'/docs/Examples/TemplateArticulated/'))
addpath(strcat(openvdPath,'/docs/Examples/TemplateArticulatedSimulink/'))
addpath(strcat(openvdPath,'/docs/Examples/TemplateSimple/'))
addpath(strcat(openvdPath,'/docs/Examples/TemplateSimpleSimulink/'))
addpath(strcat(openvdPath,'/docs/Examples/TireComparison/'))
%
% %% Deleting
% % Deleting old documentation
%
% % Old gifs
% delete('../Documentation/illustrations/*.gif')
%
% % Old html
% delete('../Documentation/html/*.*')
% delete('../Documentation/html/api/*.*')
%
% %% Publishing documentation
% %
%
% Index
options.evalCode = false;
options.outputDir = strcat(openvdPath,'/docs/');
publishOVD('main.m', options);
movefile(strcat(openvdPath,'/docs/main.html'),strcat(openvdPath,'/docs/index.html'))
%
% Tire models
options.evalCode = false;
options.outputDir = docPath;
publishOVD('DocTireLinear.m',options);
publishOVD('DocTirePacejka.m',options);
publishOVD('DocTirePolynomial.m',options);
%
% Vehicle models
publishOVD('DocVehicleArticulatedLinear.m',options);
publishOVD('DocVehicleArticulatedNonlinear.m',options);
publishOVD('DocVehicleSimpleLinear.m',options);
publishOVD('DocVehicleSimpleNonlinear.m',options);
publishOVD('DocVehicleSimpleNonlinear4DOF.m',options);
%
% % Graphics
publishOVD('DocGraphics.m',options);
%
% API
options.outputDir = apiDocPath;

publishOVD('api.m',options);

% Going to package folder
cd ../../inst/

publishOVD('@VehicleSimple/VehicleSimple.m',options);
publishOVD('@VehicleArticulated/VehicleArticulated.m',options);

publishOVD('@VehicleSimpleLinear/VehicleSimpleLinear.m',options);
publishOVD('@VehicleSimpleNonlinear/VehicleSimpleNonlinear.m',options);
publishOVD('@VehicleSimpleNonlinear4DOF/VehicleSimpleNonlinear4DOF.m',options);
publishOVD('@VehicleArticulatedLinear/VehicleArticulatedLinear.m',options);
publishOVD('@VehicleArticulatedNonlinear/VehicleArticulatedNonlinear.m',options);

publishOVD('@Tire/Tire.m',options);
publishOVD('@TireLinear/TireLinear.m',options);
publishOVD('@TirePolynomial/TirePolynomial.m',options);
publishOVD('@TirePacejka/TirePacejka.m',options);

publishOVD('@Simulator/Simulator.m',options);

publishOVD('@Graphics/Graphics.m',options);

% Going back to DocFiles
cd ../docs/DocFiles
%
% % EXAMPLES
% % Kalman Filter
% publish('KalmanFilter.m', 'outputDir', docPath, 'evalCode', true);
% close all
% clearvars -except docPath apiDocPath
%
% % Sinusoidal Steering
graphics_toolkit gnuplot            % svg quality is higher with gnuplot
SinusoidalSteering
paperPos = [0 0 10 8];
set(f1,'Paperunits','centimeters','PaperPosition',paperPos)
print(f1, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/SinusoidalSteeringFig1.svg'))
g.Frame('scalefig',3);
print(gcf, '-dsvg', strcat(openvdPath,'/docs/illustrations/frame/SinusoidalSteeringFrame.svg'))
graphics_toolkit qt
publishOVD(strcat(openvdPath,'/docs/Examples/SinusoidalSteering/SinusoidalSteering.m'),options);
g.Animation('savepath',strcat(openvdPath,'/docs/illustrations/animation/SinusoidalSteeringAnimation'),'scalefig',3);
close all

%% Skid Pad
%
graphics_toolkit gnuplot            % svg quality is higher with gnuplot
SkidPad
paperPos = [0 0 10 8];
set(f1,'Paperunits','centimeters','PaperPosition',paperPos)
print(f1, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/SkidPadFig1.svg'))
set(f2,'Paperunits','centimeters','PaperPosition',paperPos)
print(f2, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/SkidPadFig2.svg'))
set(f3,'Paperunits','centimeters','PaperPosition',paperPos)
print(f3, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/SkidPadFig3.svg'))
set(f4,'Paperunits','centimeters','PaperPosition',paperPos)
print(f4, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/SkidPadFig4.svg'))
set(f5,'Paperunits','centimeters','PaperPosition',paperPos)
print(f5, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/SkidPadFig5.svg'))
set(f6,'Paperunits','centimeters','PaperPosition',paperPos)
print(f6, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/SkidPadFig6.svg'))
g.Frame();
print(gcf, '-dsvg', strcat(openvdPath,'/docs/illustrations/frame/SkidPadFrame.svg'))
graphics_toolkit qt
publishOVD(strcat(openvdPath,'/docs/Examples/SkidPad/SkidPad.m'),options);
g.Animation('savepath',strcat(openvdPath,'/docs/illustrations/animation/SkidPadAnimation'));
close all

%% Skid Pad 4DOF
%
graphics_toolkit gnuplot            % svg quality is higher with gnuplot
SkidPad4DOF
paperPos = [0 0 10 8];
set(f1,'Paperunits','centimeters','PaperPosition',paperPos)
print(f1, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/SkidPad4DOFFig1.svg'))
set(f2,'Paperunits','centimeters','PaperPosition',paperPos)
print(f2, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/SkidPad4DOFFig2.svg'))
set(f3,'Paperunits','centimeters','PaperPosition',paperPos)
print(f3, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/SkidPad4DOFFig3.svg'))
set(f4,'Paperunits','centimeters','PaperPosition',paperPos)
print(f4, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/SkidPad4DOFFig4.svg'))
set(f5,'Paperunits','centimeters','PaperPosition',paperPos)
print(f5, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/SkidPad4DOFFig5.svg'))
set(f6,'Paperunits','centimeters','PaperPosition',paperPos)
print(f6, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/SkidPad4DOFFig6.svg'))
set(f7,'Paperunits','centimeters','PaperPosition',paperPos)
print(f7, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/SkidPad4DOFFig7.svg'))
set(f8,'Paperunits','centimeters','PaperPosition',paperPos)
print(f8, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/SkidPad4DOFFig8.svg'))
set(f9,'Paperunits','centimeters','PaperPosition',paperPos)
print(f9, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/SkidPad4DOFFig9.svg'))
set(f10,'Paperunits','centimeters','PaperPosition',paperPos)
print(f10, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/SkidPad4DOFFig10.svg'))
g.Frame();
print(gcf, '-dsvg', strcat(openvdPath,'/docs/illustrations/frame/SkidPad4DOFFrame.svg'))
graphics_toolkit qt
publishOVD(strcat(openvdPath,'/docs/Examples/SkidPad4DOF/SkidPad4DOF.m'),options);
g.Animation('savepath',strcat(openvdPath,'/docs/illustrations/animation/SkidPad4DOFAnimation'));
close all


%% SteeringControl
%

pkg load control

graphics_toolkit gnuplot            % svg quality is higher with gnuplot
SteeringControl
paperPos = [0 0 10 8];
set(f1,'Paperunits','centimeters','PaperPosition',paperPos)
print(f1, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/SteeringControlFig1.svg'))
set(f2,'Paperunits','centimeters','PaperPosition',paperPos)
print(f2, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/SteeringControlFig2.svg'))
set(f3,'Paperunits','centimeters','PaperPosition',paperPos)
print(f3, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/SteeringControlFig3.svg'))
g.Frame('scalefig',3);
print(gcf, '-dsvg', strcat(openvdPath,'/docs/illustrations/frame/SteeringControlFrame.svg'))
graphics_toolkit qt
publishOVD(strcat(openvdPath,'/docs/Examples/SteeringControl/SteeringControl.m'),options);
g.Animation('savepath',strcat(openvdPath,'/docs/illustrations/animation/SteeringControlAnimation'),'scalefig',3);
close all

%% TemplateArticulated
%
graphics_toolkit gnuplot            % svg quality is higher with gnuplot
TemplateArticulated
g.Frame();
print(gcf, '-dsvg', strcat(openvdPath,'/docs/illustrations/frame/TemplateArticulatedFrame.svg'))
graphics_toolkit qt
options.outputDir = docPath;
publishOVD(strcat(openvdPath,'/docs/Examples/TemplateArticulated/TemplateArticulated.m'),options);
g.Animation('savepath',strcat(openvdPath,'/docs/illustrations/animation/TemplateArticulatedAnimation'));
close all

%% Template Articulated Simulink
%
options.outputDir = docPath;
publish(strcat(openvdPath,'/docs/Examples/TemplateArticulatedSimulink/TemplateArticulatedSimulink.m'),options);
publish(strcat(openvdPath,'/docs/Examples/TemplateArticulatedSimulink/ArticulatedVehicleSFunction.m'),options);
close all

%% TemplateSimple
%
graphics_toolkit gnuplot            % svg quality is higher with gnuplot
TemplateSimple
paperPos = [0 0 10 8];
set(f1,'Paperunits','centimeters','PaperPosition',paperPos)
print(f1, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/TemplateSimpleFig1.svg'))
set(f2,'Paperunits','centimeters','PaperPosition',paperPos)
print(f2, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/TemplateSimpleFig2.svg'))
set(f3,'Paperunits','centimeters','PaperPosition',paperPos)
print(f3, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/TemplateSimpleFig3.svg'))
set(f4,'Paperunits','centimeters','PaperPosition',paperPos)
print(f4, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/TemplateSimpleFig4.svg'))
set(f5,'Paperunits','centimeters','PaperPosition',paperPos)
print(f5, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/TemplateSimpleFig5.svg'))
set(f6,'Paperunits','centimeters','PaperPosition',paperPos)
print(f6, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/TemplateSimpleFig6.svg'))
g.Frame();
print(gcf, '-dsvg', strcat(openvdPath,'/docs/illustrations/frame/TemplateSimpleFrame.svg'))
graphics_toolkit qt
options.outputDir = docPath;
publishOVD(strcat(openvdPath,'/docs/Examples/TemplateSimple/TemplateSimple.m'),options);
g.Animation('savepath',strcat(openvdPath,'/docs/illustrations/animation/TemplateSimpleAnimation'));
close all



%% Template Simple Simulink
%
options.outputDir = docPath;
publishOVD(strcat(openvdPath,'/docs/Examples/TemplateSimpleSimulink/TemplateSimpleSimulink.m'),options);
publish(strcat(openvdPath,'/docs/Examples/TemplateArticulatedSimulink/SimpleVehicleSFunction.m'),options);
close all


%% TireComparison
%
graphics_toolkit gnuplot            % svg quality is higher with gnuplot
TireComparison
paperPos = [0 0 10 8];
set(f1,'Paperunits','centimeters','PaperPosition',paperPos)
print(f1, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/TireComparisonFig1.svg'))
set(f2,'Paperunits','centimeters','PaperPosition',paperPos)
print(f2, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/TireComparisonFig2.svg'))
publishOVD(strcat(openvdPath,'/docs/Examples/TireComparison/TireComparison.m'),options);
close all


%% DocGen
%
publish('makeDoc.m',options);

%% Code end
% clear all                   % Clear workspace
% close all                   % Closing figures
% clc                         % Clear command window

%% See Also
%
% <../index.html Home>
%
