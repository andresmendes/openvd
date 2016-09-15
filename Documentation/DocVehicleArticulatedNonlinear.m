%% Vehicle Articulated Nonlinear
%
% <html>
% <script src='https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML'></script>
% </html>
%
%% Theory
% <theory/vehicleArticulated.pdf Articulated equations of motion>
%
%% Sintax
% |dx = _VehicleModel_.Model(~,estados)|
%
%% Arguments
% The following table describes the input arguments:
%
% <html> <table border=1 width="97%">
% <tr> <td width="30%"><tt>estados</tt></td> <td width="70%">Estados do modelo: [dPSI ALPHAT dPHI VEL PHI PSI XT YT]</td> </tr>
% </table> </html>
%
%% Description
% O ngulo \(\psi\) define a orientao do caminho-trator em relao ao referencial inercial. O estado \(\phi\)  o ngulo formado entre o caminho-trator e o semirreboque. O ngulo \(\alpha_T\)  o ngulo de deriva do mdulo dianteiro e  formado pelo vetor velocidade do centro de massa e a linha longitudinal do caminho-trator. Por fim, \(v\)  o mdulo do vetor velocidade do centro de massa do caminho-trator. Os pontos \(T\) e \(S\) so coincidentes com os centros de massa do caminho-trator e semirreboque, respectivamente. Os pontos F e R so coincidentes com os eixos dianteiro e traseiro do caminho-trator, respectivamente. M  o ponto que representa o eixo do semirreboque e A  o ponto de articulao ente as duas unidades. As grandezas a, b e c da unidade motora so as distncias entre os pontos F-T, T-R e R-A, respectivamente. Na unidade movida, d e e definem as distncias entre os pontos A-S e S-M, respectivamente.
%
% <<illustrations/modelArticulated.svg>>
%
% Este modelo  escrito na forma:
%
% \[ M(x) \dot{x} = f(x) \]
%
% Onde \(x\)  o vetor de estados, \(M(x)\)  a matriz de massa do sistema e \(f(x)\)  uma funo vetorial no linear. Logo,  necessria uma funo que permita a integrao do sistema com a matriz de massa escrita explicitamente. Uma opo  utilizar a funo _ode45_. Details: <http://www.mathworks.com/help/matlab/ref/ode45.html?searchHighlight=%22mass%20matrix%22 ode45 (Mass matrix)>
%

%% See Also
%
% <index.html Index>
%
