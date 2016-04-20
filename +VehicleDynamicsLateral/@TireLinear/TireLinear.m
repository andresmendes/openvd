%% Linear Tire
% Linear relationship between tire lateral force and slip angle.
%
%% Sintax
% |Fy = _TireModel_.Characteristic(alpha)|
%
%% Arguments
% The following table describes the input arguments
%
% <html> <table border=1 width="97%">
% <tr> <td width="30%"><tt>alpha</tt></td> <td width="70%">Tire slip angle [rad]</td> </tr>
% </table> </html>
%
%% Description
%
% The lateral force of the tire can be calculated as
%
% $$ F_y = K \alpha $$
%
% where $F_y$ is the lateral force, $K$ is the cornering stiffness and $\alpha$ is the tire slip angle.
%
% *Hypothesis*
%
% * Linear tire model
% * Valid only for small values of slip angle
%
%% Code
%

classdef TireLinear < VehicleDynamicsLateral.Tire
    methods
        % Constructor
        function self = TireLinear(varargin)
            if nargin == 0
                self.params = 1000;
            else
                self.params = varargin{1};
            end
        end

        function Fy = Characteristic(self,alpha)
            Fy = -self.params*alpha;
        end
    end

    %% Properties
    %

    properties
        params
    end
end

%% See Also
%
% <index.html Index> | <TirePolynomial.html Polynomial Tire> | <TirePacejka1989.html Pacejka 1989 Tire>
%
