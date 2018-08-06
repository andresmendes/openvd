class VehicleSimple(object):
    '''Vehicle Absract class 
    This class must be inherited by Vehicle models that will be used with the simulator. 

    :var mT:  Mass of the car (tractor) (kg)
    :var IT:  Moment of inertia the car (tractor) [kg * m2]
    :var a:  Distance from front axle of the car (tractor) to the center of mass of the car (tractor) [m]
    :var b:  Distance from center of mass of the car (tractor) to the front axle of the car (tractor) [m]
    :var mF0:  Mass over the front axle [kg]
    :var mR0:  Mass over the rear axle [kg]
    :var lT:  Wheelbase [m]
    :var nF:  Number of front tires
    :var nR:  Number of rear tires
    :var wT: Track of the car (tractor)  [m]
    :var muy: Operational friction coefficient
    :var tire: Tire model
    :var deltaf: Steering angle [rad]
    :var Fxf:  Longitudinal force at F [rad]
    :var Fxr:  Longitudinal force at R [rad]
    '''

    def __init__(self):
        self.param = 10

    def Characteristic(self, alpha, varargin):
        '''Tell my details.'''
        pass
