%% Make PAF (Plot, Animation and Frame)
% This script generates the documentation and some illustrations of the package.
%
%% Description
%
% Part of the documentation of the package is written within the code of all .m files as comments.
%
% Running this script all the .html files are generated through the command <http://www.mathworks.com/help/matlab/ref/publish.html publish> and saved in the folder "../Vehicle-Dynamics-Lateral-Documentation/".
%
%% Instructions
% Run this file from /DocFiles
%
%% Code start
%

clear all                   % Clear workspace
close all                   % Closing figures
clc

% Going to openvd root
cd ../..
% Getting openvd path
openvdPath = pwd;
% Going back to DocFiles
cd docs/DocFiles

% Path of the examples doc
docPath = strcat(openvdPath,'/docs/html/');

%% Adding paths
% Adding the folder of all necessary files to the Octave/Matlab path

% Path main functions
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

% examples
addpath(strcat(openvdPath,'/docs/examples/KalmanFilter/'))
addpath(strcat(openvdPath,'/docs/examples/SinusoidalSteering/'))
addpath(strcat(openvdPath,'/docs/examples/SkidPad/'))
addpath(strcat(openvdPath,'/docs/examples/SkidPad4DOF/'))
addpath(strcat(openvdPath,'/docs/examples/SteeringControl/'))
addpath(strcat(openvdPath,'/docs/examples/TemplateArticulated/'))
addpath(strcat(openvdPath,'/docs/examples/TemplateArticulatedSimulink/'))
addpath(strcat(openvdPath,'/docs/examples/TemplateSimple/'))
addpath(strcat(openvdPath,'/docs/examples/TemplateSimpleSimulink/'))
addpath(strcat(openvdPath,'/docs/examples/TireComparison/'))


%% Deleting old doc and illustrations
%
% Old plot
delete('../illustrations/plot/*.svg')
% Old animation
delete('../illustrations/animation/*.gif')
% Old frame
delete('../illustrations/frame/*.svg')

%% Generating illustrations
%

% Kalman Filter
graphics_toolkit gnuplot            % svg quality is higher with gnuplot
KalmanFilter
set(gcf,'nextplot','replace')
paperPos = [0 0 10 8];
gPlant.Frame('scalefig',2);
print(gcf, '-dsvg', strcat(openvdPath,'/docs/illustrations/frame/KalmanFilterFrame1.svg'))

gModel.Frame('scalefig',2);
print(gcf, '-dsvg', strcat(openvdPath,'/docs/illustrations/frame/KalmanFilterFrame2.svg'))

gPlant.Frame('scalefig',2);
hold on
gModel.Frame('scalefig',2);
print(gcf, '-dsvg', strcat(openvdPath,'/docs/illustrations/frame/KalmanFilterFrame3.svg'))

gPlant.Frame('scalefig',2);
hold on
gKalman.Frame('scalefig',2);
print(gcf, '-dsvg', strcat(openvdPath,'/docs/illustrations/frame/KalmanFilterFrame4.svg'))

paperPos = [0 0 10 8];
set(f1,'Paperunits','centimeters','PaperPosition',paperPos)
print(f1, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/KalmanFilterFig1.svg'))
set(f2,'Paperunits','centimeters','PaperPosition',paperPos)
print(f2, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/KalmanFilterFig2.svg'))
set(f3,'Paperunits','centimeters','PaperPosition',paperPos)
print(f3, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/KalmanFilterFig3.svg'))
set(f4,'Paperunits','centimeters','PaperPosition',paperPos)
print(f4, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/KalmanFilterFig4.svg'))
set(f5,'Paperunits','centimeters','PaperPosition',paperPos)
print(f5, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/KalmanFilterFig5.svg'))
set(f6,'Paperunits','centimeters','PaperPosition',paperPos)
print(f6, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/KalmanFilterFig6.svg'))
set(f7,'Paperunits','centimeters','PaperPosition',paperPos)
print(f7, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/KalmanFilterFig7.svg'))
set(f8,'Paperunits','centimeters','PaperPosition',paperPos)
print(f8, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/KalmanFilterFig8.svg'))
set(f9,'Paperunits','centimeters','PaperPosition',paperPos)
print(f9, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/KalmanFilterFig9.svg'))
set(f10,'Paperunits','centimeters','PaperPosition',paperPos)
print(f10, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/KalmanFilterFig10.svg'))
set(f11,'Paperunits','centimeters','PaperPosition',paperPos)
print(f11, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/KalmanFilterFig11.svg'))
set(f12,'Paperunits','centimeters','PaperPosition',paperPos)
print(f12, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/KalmanFilterFig12.svg'))
set(f13,'Paperunits','centimeters','PaperPosition',paperPos)
print(f13, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/KalmanFilterFig13.svg'))
set(f14,'Paperunits','centimeters','PaperPosition',paperPos)
print(f14, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/KalmanFilterFig14.svg'))
set(f15,'Paperunits','centimeters','PaperPosition',paperPos)
print(f15, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/KalmanFilterFig15.svg'))
close all


% Sinusoidal Steering
graphics_toolkit gnuplot            % svg quality is higher with gnuplot
SinusoidalSteering
paperPos = [0 0 10 8];
set(f1,'Paperunits','centimeters','PaperPosition',paperPos)
print(f1, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/SinusoidalSteeringFig1.svg'))
g.Frame('scalefig',3);
print(gcf, '-dsvg', strcat(openvdPath,'/docs/illustrations/frame/SinusoidalSteeringFrame.svg'))
close all
graphics_toolkit qt
g.Animation('savepath',strcat(openvdPath,'/docs/illustrations/animation/SinusoidalSteeringAnimation'),'scalefig',3);
close all

% Skid Pad
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
close all
graphics_toolkit qt
g.Animation('savepath',strcat(openvdPath,'/docs/illustrations/animation/SkidPadAnimation'));
close all

% Skid Pad 4DOF
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
close all
graphics_toolkit qt
g.Animation('savepath',strcat(openvdPath,'/docs/illustrations/animation/SkidPad4DOFAnimation'));
close all

% SteeringControl
graphics_toolkit gnuplot            % svg quality is higher with gnuplot
pkg load control                    % loading the control package
SteeringControl
paperPos = [0 0 10 8];
set(f1,'Paperunits','centimeters','PaperPosition',paperPos)
print(f1, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/SteeringControlFig1.svg'))
print(999, '-dsvg', strcat(openvdPath,'/docs/illustrations/frame/SteeringControlFrame.svg'))
close all
graphics_toolkit qt
g.Animation('savepath',strcat(openvdPath,'/docs/illustrations/animation/SteeringControlAnimation'),'scalefig',3);
close all

% TemplateArticulated
graphics_toolkit gnuplot            % svg quality is higher with gnuplot
TemplateArticulated
g.Frame();
print(gcf, '-dsvg', strcat(openvdPath,'/docs/illustrations/frame/TemplateArticulatedFrame.svg'))
close all
graphics_toolkit qt
g.Animation('savepath',strcat(openvdPath,'/docs/illustrations/animation/TemplateArticulatedAnimation'));
close all

% TemplateSimple
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
close all
graphics_toolkit qt
g.Animation('savepath',strcat(openvdPath,'/docs/illustrations/animation/TemplateSimpleAnimation'));
close all

% TireComparison
graphics_toolkit gnuplot            % svg quality is higher with gnuplot
TireComparison
paperPos = [0 0 10 8];
set(f1,'Paperunits','centimeters','PaperPosition',paperPos)
print(f1, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/TireComparisonFig1.svg'))
set(f2,'Paperunits','centimeters','PaperPosition',paperPos)
print(f2, '-dsvg', strcat(openvdPath,'/docs/illustrations/plot/TireComparisonFig2.svg'))
close all
