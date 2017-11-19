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
docPath = strcat(openvdPath,'/doc/Documentation/html/');
apiDocPath = strcat(openvdPath,'/doc/Documentation/html/api/');

%% Adding paths
% Adding the folder of all necessary files to the Octave/Matlab path

% Path of the publishOVD function
addpath(strcat(openvdPath,'/doc/DocFiles/publishOVD'))

% Path of the package
addpath(strcat(openvdPath,'/inst/'))

% % examples
addpath(strcat(openvdPath,'/doc/examples/KalmanFilter/'))
addpath(strcat(openvdPath,'/doc/examples/TemplateArticulatedSimulink/'))
addpath(strcat(openvdPath,'/doc/examples/TemplateSimpleSimulink/'))

%% Template Simple Simulink
TemplateSimpleSimulink
g.Frame();
print(gcf, '-dsvg', strcat(openvdPath,'/doc/Documentation/illustrations/frame/TemplateSimpleSimulinkFrame.svg'))
g.Animation('savepath',strcat(openvdPath,'/doc/Documentation/illustrations/animation/TemplateSimpleSimulinkAnimation'));
close all
