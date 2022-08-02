from breezyslam.vehicles import WheeledVehicle
from breezyslam.sensors import URG04LX
from typing import *
import math

class KheperRobot(WheeledVehicle):
    def __init__(self, x:float, y:float, orientation:'radians', filename:str):
        self.initX = x
        self.initY = y
        self.initO = orientation
        self.filename = filename
        WheeledVehicle.__init__(self, 21, 52.7)
        
        self.ticks_per_cycle = 250

    def __str__(self):
        return 'Robot| Initial Location {},{}| Initial Orientation {} radians| Filename {}.dat'.format(self.initX, 
                                                                                     self.initY,
                                                                                     self.initO,
                                                                                     self.filename)
    def computePoseChange(self, timestamp,odometry):
        
        #return WheeledVehicle.computePoseChange(self, odometry[0], odometry[1], odometry[2])
        '''
        Computes pose change based on odometry.
        
        Parameters:
        
          timestamp          time stamp, in whatever units your robot uses       
          leftWheelOdometry  odometry for left wheel, in whatever form your robot uses       
          rightWheelOdometry odometry for right wheel, in whatever form your robot uses
        
        Returns a tuple (dxyMillimeters, dthetaDegrees, dtSeconds)
        
          dxyMillimeters     forward distance traveled, in millimeters
          dthetaDegrees change in angular position, in degrees
          dtSeconds     elapsed time since previous odometry, in seconds
        '''              
        leftWheelOdometry = odometry[0]
        rightWheelOdometry = odometry[1]
        dxyMillimeters = 0
        dthetaDegrees = 0
        dtSeconds = 0
                       
        timestampSecondsCurr, leftWheelDegreesCurr, rightWheelDegreesCurr = \
            self.extractOdometry(timestamp, leftWheelOdometry, rightWheelOdometry)
            
        if self.timestampSecondsPrev != None:  
            
            leftDiffDegrees = leftWheelDegreesCurr - self.leftWheelDegreesPrev
            rightDiffDegrees = rightWheelDegreesCurr - self.rightWheelDegreesPrev
            
            dxyMillimeters =  self.wheelRadiusMillimeters * \
                    (math.radians(leftDiffDegrees) + math.radians(rightDiffDegrees))
               
            dthetaDegrees =  (float(self.wheelRadiusMillimeters) / self.halfAxleLengthMillimeters) * \
                    (rightDiffDegrees - leftDiffDegrees)
                
            dtSeconds = timestampSecondsCurr - self.timestampSecondsPrev
                
        # Store current odometry for next time
        self.timestampSecondsPrev = timestampSecondsCurr        
        self.leftWheelDegreesPrev = leftWheelDegreesCurr
        self.rightWheelDegreesPrev = rightWheelDegreesCurr

        # Return linear velocity, angular velocity, time difference
        return dxyMillimeters, dthetaDegrees, dtSeconds 

    def extractOdometry(self, timestamp, leftWheel, rightWheel):
                
        # Convert microseconds to seconds, ticks to angles        
        return timestamp / 1e6, \
               self._ticks_to_degrees(leftWheel), \
               self._ticks_to_degrees(rightWheel)
               
    def _ticks_to_degrees(self, ticks):
        
        return ticks * (180. / self.ticks_per_cycle)        
