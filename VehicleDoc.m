%% Nonlinear 2 DOF Simple Vehicle
% Nonlinear bicycle model with 2 degrees of freedom.
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
% The center of gravity of the vehicle is located at the point $T$. The front and rear axles are located ate the points $F$ and $R$, respectively. The constant $a$ measures the distance of point $F$ to $T$ and $b$ the distance of point $T$ to $R$. The angles $\alpha_F$ e $\alpha_R$ are the front and rear slip angles, respectively. $\alpha_T$ is the vehicle side slip angle and $\psi$ is the vehicle yaw angle. $\delta$ is the steering angle.
%
% <<illustrations/modeloSimples.svg>>
%
%% Code
%
%% Nonlinear 3 DOF vehicle model
% Bicycle model nonlinear with 3 degrees of freedom.
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
% O centro de massa do ve�culo � dado pelo ponto $T$ e os eixos dianteiro e traseiro s�o dados pelos pontos $F$ e $R$, respectivamente. A constante $a$ mede a dist�ncia do ponto $F$ ao $T$ e $b$ a dist�ncia do ponto $T$ ao $R$. Os �ngulos $\alpha_F$ e $\alpha_R$ s�o os �ngulos de deriva nos eixos dianteiro e traseiro. $\alpha_T$ is the vehicle side slip angle and $\psi$ is the vehicle yaw angle. Por fim, $\delta$ � o �ngulo de ester�amento.
%
% <<illustrations/modeloSimples.svg>>
%
%% Code
%
%%  Nonlinear 4 DOF articulated vehicle model
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
% O �ngulo $\psi$ define a orienta��o do caminh�o-trator em rela��o ao referencial inercial. O estado $\phi$ � o �ngulo formado entre o caminh�o-trator e o semirreboque. O �ngulo $\alpha_T$ � o �ngulo de deriva do m�dulo dianteiro e � formado pelo vetor velocidade do centro de massa e a linha longitudinal do caminh�o-trator. Por fim, $v$ � o m�dulo do vetor velocidade do centro de massa do caminh�o-trator. Os pontos $T$ e $S$ s�o coincidentes com os centros de massa do caminh�o-trator e semirreboque, respectivamente. Os pontos F e R s�o coincidentes com os eixos dianteiro e traseiro do caminh�o-trator, respectivamente. M � o ponto que representa o eixo do semirreboque e A � o ponto de articula��o ente as duas unidades. As grandezas a, b e c da unidade motora s�o as dist�ncias entre os pontos F-T, T-R e R-A, respectivamente. Na unidade movida, d e e definem as dist�ncias entre os pontos A-S e S-M, respectivamente.
%
% <<illustrations/modeloArticulado.svg>>
%
% Este modelo � escrito na forma:
%
% $$ M(x) \dot{x} = f(x)$$
%
% Onde $x$ � o vetor de estados, $M(x)$ � a matriz de massa do sistema e $f(x)$ � uma fun��o vetorial n�o linear. Logo, � necess�ria uma fun��o que permita a integra��o do sistema com a matriz de massa escrita explicitamente. Uma op��o � utilizar a fun��o _ode45_. Details: <http://www.mathworks.com/help/matlab/ref/ode45.html?searchHighlight=%22mass%20matrix%22 ode45 (Mass matrix)>
%
%% Code
%
