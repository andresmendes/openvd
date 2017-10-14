import Tire


class TireLinear(Tire.Tire):
    '''TireLinear Linear tire model

    :var k: Cornering stiffness [N/rad]
    :var q: Cornering stiffness 2 [N/rad]

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
