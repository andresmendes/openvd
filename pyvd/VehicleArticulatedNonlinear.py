import VehicleArticulated


class VehicleArticulatedNonlinear(VehicleArticulated.VehicleArticulated):
    ''' Tell my details
    '''

    def __init__(self):
        super(TireLinear, self).__init__()
     # Tire.Tire.__init__(self)

    def PlotTire(self):
        ''' Returns the handle of the curve'''
        # alpha = (0:0.1:15)*pi/180
        # Fy = - self.Characteristic(alpha)
        # p = plot(alpha*180/pi,Fy)
        # grid on; box on;
        # xlabel('Slip angle [deg]')
        # ylabel('Lateral force [N]')

    def Characteristic(self, alpha, varargin):
        # ''' Returns the handle of the curve'''
        # Fy = - self.k * alpha
        pass
