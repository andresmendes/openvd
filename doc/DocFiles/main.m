%% OpenVD: Open Vehicle Dynamics
% Open source simulation package for Octave/Matlab.
%
% <<illustrations/animation/TemplateArticulatedAnimation.gif>>
%
% This package is an open source initiative that provides vehicle models and graphics features for vehicle dynamics simulation of simple and articulated vehicles.
%
%% Installation
%
% The first thing you have to do is download the OpenVD repository by clicking <https://github.com/andresmendes/openvd/archive/master.zip here>
%
% *Octave users*
%
% * Save the zip file in your package _path_ and run

pkg install openvd-master.zip

%%
% * Load the OpenVD package with

pkg load openvd

%%
% *Matlab users*
%
% * Save the package (folder "openvd/inst") in the Matlab _path_ or add your current path to the _paths list_. More details in <http://www.mathworks.com/help/matlab/ref/path.html  help path>.
%
%% Dependencies
% To run the functions of the package and the examples from the repository a minimal Octave/Matlab version and/or Package/Toolbox is required.
%
% Dependencies of the class files:
%
% * Octave 4.0 or Matlab v8.5
%
% Dependencies of the examples:
%
% * Control System Toolbox, v9.9
% * MATLAB, v8.5
% * Robust Control Toolbox, v5.3
% * Symbolic Math Toolbox, v6.2
% * Simulink, v8.5
% * SystemTest, v2.6.9
%
%% Overview
% The general structure of the package is illustrated below. All the classes of the package are categorized into Vehicle model, Tire model and Graphics. One Vehicle model and one Tire model are combined to form the System. The integration of the System, with the apropriate parameters and initial conditions, is performed through the standard <https://www.mathworks.com/help/matlab/ref/ode45.html ode45> function of Octave/Matlab. The resulting data can be ploted as Frame and Animation with the Graphics features.
%
% This package uses an object-oriented programming architecture. For more details see <https://www.mathworks.com/discovery/object-oriented-programming.html Object-Oriented Programming in MATLAB>
%
% <<illustrations/fluxograma.svg>>
%
% The links to the description page of the available models and graphics are listed below.
%
% *Tire model*
%
% * <html/DocTireLinear.html Tire linear>
% * <html/DocTirePacejka.html Tire Pacejka>
% * <html/DocTirePolynomial.html Tire polynomial>
%
% *Vehicle model*
%
% The theoretical foundation of vehicle models can be found in: <theory/vehicleSimple.pdf TheoryVehicleSimple>, <theory/vehicleSimple4DOF.pdf TheoryVehicleSimple4DOF> and <theory/vehicleArticulated.pdf TheoryVehicleArticulated>.
%
% * <html/DocVehicleArticulatedLinear.html Vehicle Articulated Linear>
% * <html/DocVehicleArticulatedNonlinear.html Vehicle Articulated Nonlinear>
% * <html/DocVehicleSimpleLinear.html Vehicle Simple Linear>
% * <html/DocVehicleSimpleNonlinear.html Vehicle Simple Nonlinear>
% * <html/DocVehicleSimpleNonlinear4DOF.html Vehicle Simple Nonlinear 4DOF>
%
% *Graphics*
%
% * <html/DocGraphics.html Graphics>
%
%% Getting started
% To make the first steps easier, two template scripts are available covering the simulation of simple and articulated vehicles. We encourage the users to run and explore the examples <html/TemplateSimple.html TemplateSimple.m> and <html/TemplateArticulated.html TemplateArticulated.m>.
%
% Alternatively, for Matlab users familiar with Simulink, two template applications are available for running the models of the package in Simulink. Run and explore the examples <html/TemplateSimpleSimulink.html TemplateSimpleSimulink.m> and <html/TemplateArticulatedSimulink.html TemplateArticulatedSimulink.m>.
%
%% Examples
% This section presents a series of studies with the successful use of the package.
%
%
%
% <html>
% <table border=1 width="97%">
%   <tr>
%     <th>Name</th>
%     <th>Description</th>
%   </tr>
%   <tr>
%     <td><a href="html/KalmanFilter.html"> Kalman Filter </a></td>
%     <td>Kalman Filter application.</td>
%   </tr>
%   <tr>
%     <td><a href="html/SinusoidalSteering.html"> Sinusoidal Steering </a></td>
%     <td>Maneuver with sinusoidal steering angle input.</td>
%   </tr>
%   <tr>
%     <td><a href="html/SkidPad.html"> Skid Pad </a></td>
%     <td>Simple vehicle moving in circle.</td>
%   </tr>
%   <tr>
%     <td><a href="html/SkidPad4DOF.html"> Skid Pad 4DOF </a></td>
%     <td>Simple vehicle with roll dynamics moving in circle.</td>
%   </tr>
%   <tr>
%     <td><a href="html/SteeringControl.html"> Steering Control </a></td>
%     <td>Double lane change maneuver.</td>
%   </tr>
%   <tr>
%     <td><a href="html/TemplateArticulated.html"> Template Articulated </a></td>
%     <td>Articulated vehicle simulation.</td>
%   </tr>
%   <tr>
%     <td><a href="html/TemplateArticulatedSimulink.html"> Template Articulated Simulink </a></td>
%     <td>Simulate the articulated vehicle model in Simulink.</td>
%   </tr>
%   <tr>
%     <td><a href="html/TemplateSimple.html"> Template Simple </a></td>
%     <td>Simple vehicle simulation.</td>
%   </tr>
%   <tr>
%     <td><a href="html/TemplateSimpleSimulink.html"> Template Simple Simulink </a></td>
%     <td>Simulate the simple vehicle model in Simulink.</td>
%   </tr>
%   <tr>
%     <td><a href="html/TireComparison.html"> Tire Comparison </a></td>
%     <td>Comparison of tire models.</td>
%   </tr>
% </table>
% </html>
%
%% API Documentation
% API Documentation is <html/api/api.html here>. Help and documentation on-the-fly are available through the "doc" and "help" commands, as usual.
%
%% Contributing
% There are several ways to contribute to open source projects (<https://guides.github.com/activities/contributing-to-open-source/ Contributing to open source>).
%
% To push your contribution see the following steps:
%
% * Add and/or improve the .m files (package or examples) with codes and publishable comments.
% * Add the publish command of the new files to <html/makeDoc.html DocFiles/makeDoc.m>.
% * Create the apropriate links between the documentation pages. Ex: "See Also", "Examples", ...
% * Update main.m and api.m.
% * Run <html/makeDoc.html makeDoc.m>.
% * Copy the files from directory "Documentation" to the gh-pages branch of the repository. One easy way is using <https://github.com/davisp/ghp-import ghp-import>.
% * Commit and push.
%
%% Publications
%
% * MENDES, A. S.; ACKERMANN, M. ; LEONARDI, F. ; FLEURY, A. T. . Yaw stability analysis of articulated vehicles using phase trajectory method. Diname, 2017
%
% * <http://papers.sae.org/2016-36-0115/ MENDES, A. S.; MENEGHETTI, D. R. ; ACKERMANN, M. ; FLEURY, A. T. .  Vehicle Dynamics - Lateral:  Open Source Simulation Package for MATLAB. In:  Congresso SAE Brasil, 2016, São Paulo. SAE Technical Paper Series, 2016.>
%
% * <https://www.researchgate.net/publication/309567064_Analise_de_estabilidade_em_guinada_de_veiculos_articulados?_iepl%5BviewId%5D=rvwMarESwXGWiY30vN0dF13h&_iepl%5BprofilePublicationItemVariant%5D=default&_iepl%5Bcontexts%5D%5B0%5D=prfpi&_iepl%5BtargetEntityId%5D=PB%3A309567064&_iepl%5BinteractionType%5D=publicationTitle MENDES, A. S. . Análise de estabilidade em guinada de veículos articulados (In english: Yaw stability analysis of articulated vehicles) Centro Universitário FEI, 2016>
%
%% See Also
%
% <https://github.com/andresmendes/openvd GitHub Page>
