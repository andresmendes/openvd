%% Polynomial Tire
% Nonlinear relation between tire lateral force and slip angle expressed by a third order polinomial equation. This model is used by several authors [1] [2] [3].
%
%% Sintax
% |Fy = _TireModel_.Characteristic(alpha)|
%
%% Arguments
% The following table describes the input arguments:
%
% <html> <table border=1 width="97%">
% <tr> <td width="30%"><tt>alpha</tt></td> <td width="70%">Tire slip angle [rad]</td> </tr>
% </table> </html>
%
%% Description
%
% Model equation:
%
% $$ F_y = k_1 \alpha  - k_2\alpha^3 $$
%
% where $F_y$ is the lateral force and $\alpha$ is the tire slip angle. $k_1$ and $k_2$ are the model coefficients.
%
% *Hypothesis*
%
% * Nonlinear tire model
% * Valid up to the maximal lateral force (Tire saturation).
%
%% Code
%

classdef TirePolynomial < VehicleDynamicsLateral.Tire
    methods
        % Constructor
        function self = TirePolynomial(varargin)
            if nargin == 0
                % Tire parameters [1]
                Ca = 57300;
                k = 4.87;

                k1 = 2*Ca;
                k2 = 2*Ca*k;
                self.params = [k1 k2];
            else
                self.params = varargin{1};
            end
        end

        function Fy = Characteristic(self,alpha,~,~)
            % Polynomial model coefficients
            k1 = self.params(1);
            k2 = self.params(2);
            % Lateral force
            Fy = - (k1*alpha-k2*alpha.^3);
        end
    end

    %% Properties
    %

    properties
        params
    end
end

%% References
%
% [1] JOHNSON, D.; HUSTON, J. Nonlinear lateral stability analysis of road vehicles using Liapunov's second method. SAE Technical Paper, SAE International, 1984.
%
% [2] SADRI, S.; WU, C. Stability analysis of a nonlinear vehicle model in plane motion using the concept of lyapunov exponents. Vehicle System Dynamics, Taylor & Francis, v. 51, n. 6, p.906-924, 2013.
%
% [3] SAMSUNDAR, J.; HUSTON, J. C. Estimating lateral stability region of a nonlinear 2 degree-of-freedom vehicle. SAE Technical Paper, 1998.
%
%% See Also
%
% <index.html Index> | <TireLinear.html Tire linear> | <TirePacejka1989.html Tire Pacejka 1989>
%
