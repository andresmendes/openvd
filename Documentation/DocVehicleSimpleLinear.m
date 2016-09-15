%% Vehicle Simple Linear
% Linear bicycle model with 3 degrees of freedom.
%
% <html>
% <script src='https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML'></script>
% </html>
%
%
%% Theory
% <theory/vehicleSimple.pdf Simple equations of motion>
%
%% Sintax
% |dx = _VehicleModel_.Model(~,estados)|
%
%% Arguments
% The following table describes the input arguments:
%
% <html> <table border=1 width="97%">
% <tr> <td width="30%"><tt>estados</tt></td> <td width="70%">Estados do modelo: [dPSI ALPHAT PSI XT YT VEL]</td> </tr>
% </table> </html>
%
%% Description
% The center of gravity of the vehicle is located at the point \(T\). The front and rear axles are located ate the points \(F\) and \(R\), respectively. The constant \(a\) measures the distance of point \(F\) to \(T\) and \(b\) the distance of point \(T\) to \(R\). The angles \(\alpha_F\) e \(\alpha_R\) are the front and rear slip angles, respectively. \(\alpha_T\) is the vehicle side slip angle and \(\psi\) is the vehicle yaw angle. \(\delta\) is the steering angle.
%
% <<illustrations/modelSimple.svg>>
%
%% See Also
%
% <index.html Index>
%
