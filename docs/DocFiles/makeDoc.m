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
cd doc/DocFiles
% Folder where html doc files are saved
docPath = strcat(openvdPath,'/doc/html/');
apiDocPath = strcat(openvdPath,'/doc/html/api/');

%% Adding paths
% Adding the folder of all necessary files to the Octave/Matlab path

% Path of the publishOVD function
addpath(strcat(openvdPath,'/doc/DocFiles/publishOVD'))

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
addpath(strcat(openvdPath,'/doc/Examples/KalmanFilter/'))
addpath(strcat(openvdPath,'/doc/Examples/SinusoidalSteering/'))
addpath(strcat(openvdPath,'/doc/Examples/SkidPad/'))
addpath(strcat(openvdPath,'/doc/Examples/SkidPad4DOF/'))
addpath(strcat(openvdPath,'/doc/Examples/SteeringControl/'))
addpath(strcat(openvdPath,'/doc/Examples/TemplateArticulated/'))
addpath(strcat(openvdPath,'/doc/Examples/TemplateArticulatedSimulink/'))
addpath(strcat(openvdPath,'/doc/Examples/TemplateSimple/'))
addpath(strcat(openvdPath,'/doc/Examples/TemplateSimpleSimulink/'))
addpath(strcat(openvdPath,'/doc/Examples/TireComparison/'))
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
options.outputDir = strcat(openvdPath,'/doc/');
publishOVD('main.m', options);
movefile(strcat(openvdPath,'/doc/main.html'),strcat(openvdPath,'/doc/index.html'))
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
cd ../doc/DocFiles
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
print(f1, '-dsvg', strcat(openvdPath,'/doc/illustrations/plot/SinusoidalSteeringFig1.svg'))
g.Frame('scalefig',3);
print(gcf, '-dsvg', strcat(openvdPath,'/doc/illustrations/frame/SinusoidalSteeringFrame.svg'))
graphics_toolkit qt
publishOVD(strcat(openvdPath,'/doc/Examples/SinusoidalSteering/SinusoidalSteering.m'),options);
g.Animation('savepath',strcat(openvdPath,'/doc/illustrations/animation/SinusoidalSteeringAnimation'),'scalefig',3);
close all


% publish('SinusoidalSteering.m', 'outputDir', docPath, 'evalCode', true);
% g.Animation('../Documentation/illustrations/AnimationSinusoidalSteering');
% close all
% clearvars -except docPath apiDocPath
%

%% Skid Pad
%
graphics_toolkit gnuplot            % svg quality is higher with gnuplot
SkidPad
paperPos = [0 0 10 8];
set(f1,'Paperunits','centimeters','PaperPosition',paperPos)
print(f1, '-dsvg', strcat(openvdPath,'/doc/illustrations/plot/SkidPadFig1.svg'))
set(f2,'Paperunits','centimeters','PaperPosition',paperPos)
print(f2, '-dsvg', strcat(openvdPath,'/doc/illustrations/plot/SkidPadFig2.svg'))
set(f3,'Paperunits','centimeters','PaperPosition',paperPos)
print(f3, '-dsvg', strcat(openvdPath,'/doc/illustrations/plot/SkidPadFig3.svg'))
set(f4,'Paperunits','centimeters','PaperPosition',paperPos)
print(f4, '-dsvg', strcat(openvdPath,'/doc/illustrations/plot/SkidPadFig4.svg'))
set(f5,'Paperunits','centimeters','PaperPosition',paperPos)
print(f5, '-dsvg', strcat(openvdPath,'/doc/illustrations/plot/SkidPadFig5.svg'))
set(f6,'Paperunits','centimeters','PaperPosition',paperPos)
print(f6, '-dsvg', strcat(openvdPath,'/doc/illustrations/plot/SkidPadFig6.svg'))
g.Frame();
print(gcf, '-dsvg', strcat(openvdPath,'/doc/illustrations/frame/SkidPadFrame.svg'))
graphics_toolkit qt
publishOVD(strcat(openvdPath,'/doc/Examples/SkidPad/SkidPad.m'),options);
g.Animation('savepath',strcat(openvdPath,'/doc/illustrations/animation/SkidPadAnimation'));
close all

%% Skid Pad 4DOF
%
graphics_toolkit gnuplot            % svg quality is higher with gnuplot
SkidPad4DOF
paperPos = [0 0 10 8];
set(f1,'Paperunits','centimeters','PaperPosition',paperPos)
print(f1, '-dsvg', strcat(openvdPath,'/doc/illustrations/plot/SkidPad4DOFFig1.svg'))
set(f2,'Paperunits','centimeters','PaperPosition',paperPos)
print(f2, '-dsvg', strcat(openvdPath,'/doc/illustrations/plot/SkidPad4DOFFig2.svg'))
set(f3,'Paperunits','centimeters','PaperPosition',paperPos)
print(f3, '-dsvg', strcat(openvdPath,'/doc/illustrations/plot/SkidPad4DOFFig3.svg'))
set(f4,'Paperunits','centimeters','PaperPosition',paperPos)
print(f4, '-dsvg', strcat(openvdPath,'/doc/illustrations/plot/SkidPad4DOFFig4.svg'))
set(f5,'Paperunits','centimeters','PaperPosition',paperPos)
print(f5, '-dsvg', strcat(openvdPath,'/doc/illustrations/plot/SkidPad4DOFFig5.svg'))
set(f6,'Paperunits','centimeters','PaperPosition',paperPos)
print(f6, '-dsvg', strcat(openvdPath,'/doc/illustrations/plot/SkidPad4DOFFig6.svg'))
set(f7,'Paperunits','centimeters','PaperPosition',paperPos)
print(f7, '-dsvg', strcat(openvdPath,'/doc/illustrations/plot/SkidPad4DOFFig7.svg'))
set(f8,'Paperunits','centimeters','PaperPosition',paperPos)
print(f8, '-dsvg', strcat(openvdPath,'/doc/illustrations/plot/SkidPad4DOFFig8.svg'))
set(f9,'Paperunits','centimeters','PaperPosition',paperPos)
print(f9, '-dsvg', strcat(openvdPath,'/doc/illustrations/plot/SkidPad4DOFFig9.svg'))
set(f10,'Paperunits','centimeters','PaperPosition',paperPos)
print(f10, '-dsvg', strcat(openvdPath,'/doc/illustrations/plot/SkidPad4DOFFig10.svg'))
g.Frame();
print(gcf, '-dsvg', strcat(openvdPath,'/doc/illustrations/frame/SkidPad4DOFFrame.svg'))
graphics_toolkit qt
publishOVD(strcat(openvdPath,'/doc/Examples/SkidPad4DOF/SkidPad4DOF.m'),options);
g.Animation('savepath',strcat(openvdPath,'/doc/illustrations/animation/SkidPad4DOFAnimation'));
close all


%% SteeringControl
%
graphics_toolkit gnuplot            % svg quality is higher with gnuplot
SteeringControl
paperPos = [0 0 10 8];
set(f1,'Paperunits','centimeters','PaperPosition',paperPos)
print(f1, '-dsvg', strcat(openvdPath,'/doc/illustrations/plot/SteeringControlFig1.svg'))
set(f2,'Paperunits','centimeters','PaperPosition',paperPos)
print(f2, '-dsvg', strcat(openvdPath,'/doc/illustrations/plot/SteeringControlFig2.svg'))
set(f3,'Paperunits','centimeters','PaperPosition',paperPos)
print(f3, '-dsvg', strcat(openvdPath,'/doc/illustrations/plot/SteeringControlFig3.svg'))
g.Frame('scalefig',3);
print(gcf, '-dsvg', strcat(openvdPath,'/doc/illustrations/frame/SteeringControlFrame.svg'))
graphics_toolkit qt
publishOVD(strcat(openvdPath,'/doc/Examples/SteeringControl/SteeringControl.m'),options);
g.Animation('savepath',strcat(openvdPath,'/doc/illustrations/animation/SteeringControlAnimation'),'scalefig',3);
close all

%% TemplateArticulated
%
graphics_toolkit gnuplot            % svg quality is higher with gnuplot
TemplateArticulated
g.Frame();
print(gcf, '-dsvg', strcat(openvdPath,'/doc/illustrations/frame/TemplateArticulatedFrame.svg'))
graphics_toolkit qt
options.outputDir = docPath;
publishOVD(strcat(openvdPath,'/doc/Examples/TemplateArticulated/TemplateArticulated.m'),options);
g.Animation('savepath',strcat(openvdPath,'/doc/illustrations/animation/TemplateArticulatedAnimation'));
close all

% % Template Articulated Simulink
% publish('ArticulatedVehicleSFunction.m', 'outputDir', docPath, 'evalCode', false);
% publish('TemplateArticulatedSimulink.m', 'outputDir', docPath, 'evalCode', true);
% g.Animation('../Documentation/illustrations/AnimationTemplateArticulatedSimulink');
% close all
% clearvars -except docPath apiDocPath
%


%% TemplateSimple
%
graphics_toolkit gnuplot            % svg quality is higher with gnuplot
TemplateSimple
paperPos = [0 0 10 8];
set(f1,'Paperunits','centimeters','PaperPosition',paperPos)
print(f1, '-dsvg', strcat(openvdPath,'/doc/illustrations/plot/TemplateSimpleFig1.svg'))
set(f2,'Paperunits','centimeters','PaperPosition',paperPos)
print(f2, '-dsvg', strcat(openvdPath,'/doc/illustrations/plot/TemplateSimpleFig2.svg'))
set(f3,'Paperunits','centimeters','PaperPosition',paperPos)
print(f3, '-dsvg', strcat(openvdPath,'/doc/illustrations/plot/TemplateSimpleFig3.svg'))
set(f4,'Paperunits','centimeters','PaperPosition',paperPos)
print(f4, '-dsvg', strcat(openvdPath,'/doc/illustrations/plot/TemplateSimpleFig4.svg'))
set(f5,'Paperunits','centimeters','PaperPosition',paperPos)
print(f5, '-dsvg', strcat(openvdPath,'/doc/illustrations/plot/TemplateSimpleFig5.svg'))
set(f6,'Paperunits','centimeters','PaperPosition',paperPos)
print(f6, '-dsvg', strcat(openvdPath,'/doc/illustrations/plot/TemplateSimpleFig6.svg'))
g.Frame();
print(gcf, '-dsvg', strcat(openvdPath,'/doc/illustrations/frame/TemplateSimpleFrame.svg'))
graphics_toolkit qt
options.outputDir = docPath;
publishOVD(strcat(openvdPath,'/doc/Examples/TemplateSimple/TemplateSimple.m'),options);
g.Animation('savepath',strcat(openvdPath,'/doc/illustrations/animation/TemplateSimpleAnimation'));
close all



%% Template Simple Simulink
%
publishOVD(strcat(openvdPath,'/doc/Examples/TemplateSimpleSimulink/TemplateSimpleSimulink.m'),options);


%% TireComparison
%
graphics_toolkit gnuplot            % svg quality is higher with gnuplot
TireComparison
paperPos = [0 0 10 8];
set(f1,'Paperunits','centimeters','PaperPosition',paperPos)
print(f1, '-dsvg', strcat(openvdPath,'/doc/illustrations/plot/TireComparisonFig1.svg'))
set(f2,'Paperunits','centimeters','PaperPosition',paperPos)
print(f2, '-dsvg', strcat(openvdPath,'/doc/illustrations/plot/TireComparisonFig2.svg'))
publishOVD(strcat(openvdPath,'/doc/Examples/TireComparison/TireComparison.m'),options);
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
