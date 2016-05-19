%% Pacejka 1989 Tire
% Nonlinear relationship between tire lateral force and slip angle expressed by a semi-empirical model with experimental coefficients [1].
%
%% Sintax
% |Fy = _TireModel_.Characteristic(alpha, Fz, muy)|
%
%% Arguments
% The following table describes the input arguments
%
% <html> <table border=1 width="97%">
% <tr> <td width="30%"><tt>alpha</tt></td> <td width="70%">Tire slip angle [rad]</td> </tr>
% <tr> <td width="30%"><tt>Fz</tt></td> <td width="70%">Vertical force [N]</td> </tr>
% <tr> <td width="30%"><tt>muy</tt></td> <td width="70%">Friction coefficient [-]</td> </tr>
% </table> </html>
%
%% Description
% The lateral force can be written as
%
% $$ F_y = - \frac{\mu_y}{\mu_{y, n}} (F_{y, n}(\alpha_{eq}) + S_v)$$
%
% where $\alpha_{eq}$ is the equivalent slip angle
%
% $$ \alpha_{eq} = \frac{\mu_{y0}}{\mu_y} \frac{F_{z0}}{F_z} (\alpha + S_h)$$
%
% and $F_{y, n}$ is the reference function of the lateral force
%
% $$ F_{y, n} = D \sin(C \arctan(B \alpha - E (B \alpha - \arctan(B \alpha)))) $$
%
% The coefficients $B$, $C$, $D$ and $E$ can be written as
%
% $$ C = a_0 $$
%
% $$ D = \mu_{y, n} F_z = (a_1 F_z + a_2) F_z $$
%
% $$ B = \frac{B C D}{C D} = a_3 \sin \left\{ 2 \arctan \left( \frac{F_z}{a_4} \right) \right\} (1 - a_5 |\gamma|)$$
%
% $$ E = a_6 F_z + a_7$$
%
% The horizontal and vertical shifts of the curve are calculated as
%
% $$ S_h = a_8 \gamma + a_9 F_z + a_{10} $$
%
% $$ S_v = a_{11} F_z \gamma + a_{12} F_z + a_{13} $$
%
% The model implemented here converts the slip angle using the following equation
%
% _ALPHA = asin(sin(alpha));_
%
% This equation alters the slip angle in such way that the characteristic equation becames symmetric in relation to the vertical line at 90 degrees and the lateral force becomes zero at 180 degrees. The same analogy can be made with negative values of the slip angle
%
% *Hypothesis*
%
% * Nonlinear tire model
% * Covers the whole range of slip angles (-180 to 180 degrees)
%
%% Code
%

classdef TirePacejka1989 < VehicleDynamicsLateral.Tire
    methods
        % Constructor
        function self = TirePacejka1989()
            self.a0 = 1;
            self.a1 = 0;
            self.a2 = 700;
            self.a3 = 3000;
            self.a4 = 50;
            self.a5 = 0;
            self.a6 = 0;
            self.a7 = -1;
            self.a8 = 0;
            self.a9 = 0;
            self.a10 = 0;
            self.a11 = 0;
            self.a12 = 0;
            self.a13 = 0;
        end

        function Fy = Characteristic(self, alpha, Fz, muy)
            % Input
            % alpha - slip angle [rad]
            % Fz    - Load [N]
            % muy   - Lateral friction coefficient (*1000) [-]

            % Slip angle treatment
            ALPHA = asin(sin(alpha));          % [rad]
            ALPHA = 180 / pi * ALPHA;   % Conversion [rad] - [deg]
            % Nominal parameters
            a0 = self.a0;
            a1 = self.a1;
            a2 = self.a2;
            a3 = self.a3;
            a4 = self.a4;
            a5 = self.a5;
            a6 = self.a6;
            a7 = self.a7;
            a8 = self.a8;
            a9 = self.a9;
            a10 = self.a10;
            a11 = self.a11;
            a12 = self.a12;
            a13 = self.a13;

            Fz = Fz/1000;           % Conversion [N] - [kN]

            camber = 0;             % Camber angle

            C = a0;                 % Shape factor
            muy0 = a1 * Fz + a2;      % Lateral friction coefficient nominal [-]
            muy = muy * 1000;         % Lateral friction coefficient operacional
            D = muy0 * Fz;            % muy = lateral friction coefficient
            BCD = a3 * sin(2 * atan(Fz/a4))*(1-a5 * abs(camber)); % Cornering stiffness
            E = a6 * Fz + a7;         % Curvature factor
            B = BCD/(C * D);          % stiffness factor
            Sh = a8 * camber + a9 * Fz + a10;      % Horizontal shift
            Sv = a11 * Fz * camber + a12 * Fz + a13; % Vertical shift
            ALPHAeq = muy0/muy*(ALPHA + Sh);   % Equivalent slip angle
 % Reference characteristics
            fy = D * sin(C * atan(B * ALPHAeq - E*(B * ALPHAeq - atan(B * ALPHAeq))));
 % Lateral force
            Fy = -muy/muy0*(fy + Sv);
        end
    end

   %% Properties
   %

    properties
        a0 % Shape factor [-]
        a1 % Load dependency of lateral friction (*1000) [1/kN]
        a2 % Lateral friction level (*1000) [-]
        a3 % Maximum cornering stiffness [N/deg]
        a4 % Load at maximum cornering stiffness [kN]
        a5 % Camber sensitivity of cornering stiffness
        a6 % Load dependency of curvature factor
        a7 % Curvature factor level
        a8 % Camber sensitivity of horizontal shift
        a9 % Load dependency of horizontal shift
        a10 % Horizontal shift level
        a11 % Combined load and camber sensitivity of vertical shift
        a12 % Load dependency of vertical shift
        a13 % Vertical shift level
    end
end

%% References
% [1] BAKKER, E.; PACEJKA, H. B.; LIDNER, L. A new tire model with an application in vehicle dynamics studies. [S.l.], 1989
%
%% See Also
%
% <index.html Index> | <TireLinear.html Linear Tire> | <TirePolynomial.html Polynomial Tire>
%
