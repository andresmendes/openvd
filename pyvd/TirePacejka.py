import Tire


class TirePacejka(Tire.Tire):
    '''TirePacejka tire model

    :var a0: Shape factor [-]
    :var a1: Load dependency of lateral friction (1000) [1/kN]
    :var a2: Lateral friction level (1000) [-]
    :var a3: Maximum cornering stiffness [N/deg]
    :var a4: Load at maximum cornering stiffness [kN]
    :var a5: Camber sensitivity of cornering stiffness
    :var a6: Load dependency of curvature factor
    :var a7: Curvature factor level
    :var a8: Camber sensitivity of horizontal shift
    :var a9: Load dependency of horizontal shift
    :var a10: Horizontal shift level
    :var a11: Combined load and camber sensitivity of vertical shift
    :var a12: Load dependency of vertical shift
    :var a13: Vertical shift level

    '''

    def __init__(self):
        super(TirePacejka, self).__init__()
        self.a0 = 1
        self.a1 = 0
        self.a2 = 800
        self.a3 = 3000
        self.a4 = 50
        self.a5 = 0
        self.a6 = 0
        self.a7 = -1
        self.a8 = 0
        self.a9 = 0
        self.a10 = 0
        self.a11 = 0
        self.a12 = 0
        self.a13 = 0

    def PlotTire(self, Fz, muy):
        # % Returns the handle of the curve
        # alpha = (0:0.1:15)*pi/180;
        # Fy = - self.Characteristic(alpha, Fz, muy);
        # p = plot(alpha*180/pi,Fy);
        # grid on; box on;
        # xlabel('Slip angle [deg]')
        # ylabel('Lateral force [N]')
        pass

    def Characteristic(self, alpha, Fz, muy):
        #            % Input
        #            % alpha - slip angle [rad]
        #            % Fz    - Load [N]
        #            % muy   - Lateral friction coefficient (*1000) [-]
        #
        #            % Slip angle treatment
        #            ALPHA = asin(sin(alpha)); % [rad]
        #            ALPHA = 180 / pi * ALPHA; % Conversion [rad] - [deg]
        #            % Nominal parameters
        #            a0 = self.a0;
        #            a1 = self.a1;
        #            a2 = self.a2;
        #            a3 = self.a3;
        #            a4 = self.a4;
        #            a5 = self.a5;
        #            a6 = self.a6;
        #            a7 = self.a7;
        #            a8 = self.a8;
        #            a9 = self.a9;
        #            a10 = self.a10;
        #            a11 = self.a11;
        #            a12 = self.a12;
        #            a13 = self.a13;
        #
        #            Fz = Fz/1000;           % Conversion [N] - [kN]
        #
        #            camber = 0;             % Camber angle
        #
        #            C = a0;                 % Shape factor
        #            muy0 = a1 * Fz + a2;      % Lateral friction coefficient nominal [-]
        #            muy = muy * 1000;         % Lateral friction coefficient operacional
        #            D = muy0 * Fz;            % muy = lateral friction coefficient
        #            BCD = a3 * sin(2 * atan(Fz/a4))*(1-a5 * abs(camber)); % Cornering stiffness
        #            E = a6 * Fz + a7;         % Curvature factor
        #            B = BCD/(C * D);          % stiffness factor
        #            Sh = a8 * camber + a9 * Fz + a10;      % Horizontal shift
        #            Sv = a11 * Fz * camber + a12 * Fz + a13; % Vertical shift
        #            ALPHAeq = muy0/muy*(ALPHA + Sh);   % Equivalent slip angle
        # % Reference characteristics
        #            fy = D * sin(C * atan(B * ALPHAeq - E*(B * ALPHAeq - atan(B * ALPHAeq))));
        # % Lateral force
        #            Fy = -muy/muy0*(fy + Sv);
        pass
