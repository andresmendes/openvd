class Simulator(object):
    ''' Simulator Vehicle dynamics simulator
        The simulator receives a vehicle object that 
        inherits from VehicleSimple, simulates its behavior during a given 
        time span and provides its behavior during time via its properties. Each property is a (timespan, 1) vector in which each value represents that parameter's value in time.
  
    :var Vehicle: Vehicle model to be used inthe simulation
    :var TSpan: a vector indicating the intervals in which the simulation steps will be conducted
    :var X0: Initial CG horizontal position [m] (init to 0)
    :var Y0: Initial CG vertical position [m] (init to 0)
    :var PSI0:  : Initial yaw angle [rad] (init to 0) 
    :var PHI0: Initial articulation angle [rad] (init to 0)
    :var THETA0: Initial roll angle [rad] (init to 0)
    :var V0: Initial CG velocity [m/s] (init to 20)
    :var ALPHAT0: Initial side slip angle [rad] (init to 0)
    :var dPSI0: Initial yaw rate [rad/s] (init to 0)
    :var dPHI0: Initial articulation rate [rad/s] (init to 0)
    :var dTHETA0: Initial roll rate [rad/s] (init to 0)
    :var XT: CG horizontal position [m]
    :var YT: CG vertical position [m]
    :var PSI: Yaw angle [rad]
    :var PHI: Relative yaw angle [rad]
    :var THETA: Roll angle [rad]
    :var VEL: CG velocity [m/s]
    :var ALPHAT: Side slip angle [rad]
    :var dPSI: Yaw rate [rad/s]
    :var dPHI: Relative yaw rate [rad/s]
    :var dTHETA: Roll rate [rad/s]
    '''

    def __init__(self):
        ''' 
        Requires
        :var Vehicle: Vehicle model to be used inthe simulation
        :var tspan: a vector indicating the intervals in which the simulation steps will be conducted

        Depending on the type of vehicle the initialisation is different.
        '''


    def Simulate(self):
        '''integration
        '''
        pass

    def getInitialState(self):
        ''' Transforms properties into a vector so it can be used by the integrator
   
        '''
        pass