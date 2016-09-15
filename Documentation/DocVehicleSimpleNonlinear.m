%% Vehicle Simple Nonlinear
% Bicycle model nonlinear with 3 degrees of freedom.
%
% <html>
% <script src='https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML'></script>
% </html>
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
% O centro de massa do veculo  dado pelo ponto \(T\) e os eixos dianteiro e traseiro so dados pelos pontos \(F\) e \(R\), respectivamente. A constante \(a\) mede a distncia do ponto \(F\) ao \(T\) e \(b\) a distncia do ponto \(T\) ao \(R\). Os ngulos \(\alpha_F\) e \(\alpha_R\) so os ngulos de deriva nos eixos dianteiro e traseiro. \(\alpha_T\) is the vehicle side slip angle and \(\psi\) is the vehicle yaw angle. Por fim, \(\delta\)  o ngulo de esteramento.
%
% <<illustrations/modelSimple.svg>>
%
%% See Also
%
% <index.html Index>
%
