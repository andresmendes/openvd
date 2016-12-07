%% Vehicle Dynamics - Lateral: Open Source Simulation Package for MATLAB
% This repository provides a collection of functions destinated to lateral dynamics simulations of ground vehicles.
%
%% Installation
% The first thing you have to do is install the package according to the following steps:
%
% * Download the Vehicle Dynamics - Lateral repository by clicking <https://github.com/andresmendes/Vehicle-Dynamics-Lateral/archive/master.zip here>
% * Save the package (folder "+VehicleDynamicsLateral") in the MATLAB(R) _path_ or add your current path to the _paths list_. More details in <http://www.mathworks.com/help/matlab/ref/path.html  help path>.
%
%% Dependencies
% Dependencies of the package (Only the files in "+VehicleDynamicsLateral"):
%

packageFilesList = getAllFiles('../+VehicleDynamicsLateral');
[packageFilesDep,packageProductDep] = matlab.codetools.requiredFilesAndProducts(packageFilesList);

packageDepNumber = length(packageProductDep);

for i = 1:packageDepNumber

    disp(strcat(packageProductDep(i).Name,', v',packageProductDep(i).Version));

end

%%
% Dependencies of the examples (Only the files in "Examples"):
%

examplesFilesList = getAllFiles('../Examples');
[examplesFilesDep,examplesProductDep] = matlab.codetools.requiredFilesAndProducts(examplesFilesList);

examplesDepNumber = length(examplesProductDep);

for i = 1:examplesDepNumber

    disp(strcat(examplesProductDep(i).Name,', v',examplesProductDep(i).Version));

end

%% Getting started
%
% Run and explore the files <html/TemplateSimple.html TemplateSimple.m> and <html/TemplateArticulated.html TemplateArticulated.m>.
%
%% Description
%
% <<illustrations/fluxograma.svg>>
%
%% Examples
% The templates simulate vehicle systems according to the flowchart above.
%
% * <html/TemplateSimple.html Template Simple> - Simple vehicle simulation.
% * <html/TemplateArticulated.html Template Articulated> - Articulated vehicle simulation.
% * <html/TireComparison.html Tire Comparison> - Comparison of tire models.
% * <html/SimulinkApplication.html Simulink Application> - Simulate the vehicle models in Simulink.
% * <html/SinusoidalSteering.html Sinusoidal Steering> - Maneuver with sinusoidal steering angle input.
% * <html/SteeringControl.html Steering Control> - Double lane change maneuver.
% * <html/KalmanFilter.html Kalman Filter> - Kalman Filter application
% * <html/SkidPad.html Skid Pad> - Simple vehicle moving in circle
% * <html/SkidPad4DOF.html Skid Pad 4DOF> - Simple vehicle with roll dynamics moving in circle
%
%% Tire model
%
% * <html/DocTireLinear.html Tire linear>
% * <html/DocTirePolynomial.html Tire polynomial>
% * <html/DocTirePacejka.html Tire Pacejka>
%
%% Vehicle model
%
% * <html/DocVehicleSimpleLinear.html Vehicle Simple Linear>
% * <html/DocVehicleSimpleNonlinear.html Vehicle Simple Nonlinear>
% * <html/DocVehicleSimpleNonlinear4DOF.html Vehicle Simple Nonlinear 4DOF>
% * <html/DocVehicleArticulatedLinear.html Vehicle Articulated Linear>
% * <html/DocVehicleArticulatedNonlinear.html Vehicle Articulated Nonlinear>
%
%% Graphics
% <html/DocGraphics.html Graphics> - Functions for graphics generation.
%
%% API Documentation
% API Documentation is <html/api/api.html here>. Help and documentation on-the-fly are available through the "doc" and "help" commands, as usual.
%
%% Contributing
% Steps:
%
% * Add and/or improve Matlab files (package or examples) with codes and publishable comments.
% * Add the publish command of the new files to <html/makeDoc.html DocFiles/makeDoc.m>.
% * Create the apropriate links between the documentation pages. Ex: "See Also", "Examples", ...
% * Update index.m and README.md.
% * Run <html/makeDoc.html makeDoc.m>.
% * Copy the files from directory "Documentation" to the gh-pages branch of the repository. One easy way is using <https://github.com/davisp/ghp-import ghp-import>.
% * Commit and push.
%
%% See Also
%
% <https://github.com/andresmendes/Vehicle-Dynamics-Lateral GitHub Page>
