'''
mines.py - classes for the SLAM apparatus used at Paris Mines Tech
             
For details see

    @inproceedings{coreslam-2010,
      author    = {Bruno Steux and Oussama El Hamzaoui},
      title     = {CoreSLAM: a SLAM Algorithm in less than 200 lines of C code},
      booktitle = {11th International Conference on Control, Automation, 
                   Vehicleics and Vision, ICARCV 2010, Singapore, 7-10 
                   December 2010, Proceedings},
      pages     = {1975-1979},
      publisher = {IEEE},
      year      = {2010}
    }
                 
Copyright (C) 2014 Simon D. Levy

This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.

This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http://www.gnu.org/licenses/>.
'''

from breezyslam.vehicles import WheeledVehicle
from breezyslam.sensors import URG04LX
import math



# Method to load all from file ------------------------------------------------
# Each line in the file has the format:
#
#  TIMESTAMP  ... Q1  Q1 ... Distances
#  (usec)                    (mm)
#  0          ... 2   3  ... 24 ... 
#  
#where Q1, Q2 are odometry values

def load_data(datadir, dataset):
    
    filename = '%s/%s.dat' % (datadir, dataset)
    print('Loading data from %s...' % filename)
    
    fd = open(filename, 'rt')
    
    timestamps = []
    scans = []
    odometries = []
    
    while True:  
        
        s = fd.readline()
        
        if len(s) == 0:
            break       
            
        toks = s.split()[0:-1] # ignore ''

        timestamp = int(toks[0])

        odometry = timestamp, int(toks[2]), int(toks[3])
                        
        lidar = [int(tok) for tok in toks[24:]]

        timestamps.append(timestamp)
        scans.append(lidar)
        odometries.append(odometry)
        
    fd.close()
        
    return timestamps, scans, odometries

class MinesLaser(URG04LX):
    
    def __init__(self):
        
        URG04LX.__init__(self, 70, 145)
        
# Class for MinesRover custom robot ------------------------------------------

class Rover(WheeledVehicle):
    
    def __init__(self):
        
        WheeledVehicle.__init__(self, 21, 52.7)
        
        self.ticks_per_cycle = 180.*10
                        
    def __str__(self):
        
        return '<%s ticks_per_cycle=%d>' % (WheeledVehicle.__str__(self), self.ticks_per_cycle)
        
    '''
    def computePoseChange(self, odometry):
        dxyMillimeters, dthetaDegrees, dtSeconds = WheeledVehicle.computePoseChange(self, odometry[0], odometry[1], odometry[2])
        print('here', dthetaDegrees)
        return dxyMillimeters, dthetaDegrees, dtSeconds
    '''

    def extractOdometry(self, timestamp, leftWheel, rightWheel):
                
        # Convert microseconds to seconds, ticks to angles        
        return timestamp / 1e6, \
               self._ticks_to_degrees(leftWheel), \
               self._ticks_to_degrees(rightWheel)
               
    def odometryStr(self, odometry):
        
        return '<timestamp=%d usec leftWheelTicks=%d rightWheelTicks=%d>' % \
               (odometry[0], odometry[1], odometry[2])
               
    def _ticks_to_degrees(self, ticks):
        
        return ticks * (180. / self.ticks_per_cycle)

    def computePoseChange(self, timestamp, leftWheelOdometry, rightWheelOdometry):
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
        dxyMillimeters = 0
        dthetaDegrees = 0
        dtSeconds = 0
                       
        timestampSecondsCurr, leftWheelDegreesCurr, rightWheelDegreesCurr = \
            self.extractOdometry(timestamp, leftWheelOdometry, rightWheelOdometry)

        turnLeft = False
        turnRight = False
            
        if self.timestampSecondsPrev != None:  
            
            leftDiffDegrees = leftWheelDegreesCurr - self.leftWheelDegreesPrev
            rightDiffDegrees = rightWheelDegreesCurr - self.rightWheelDegreesPrev
            if leftDiffDegrees < 0:
                turnLeft = True
            if rightDiffDegrees <0:
                turnRight = True
            
            dxyMillimeters =  self.wheelRadiusMillimeters * \
                    (math.radians(leftDiffDegrees) + math.radians(rightDiffDegrees))
               
            dthetaDegrees =  (float(self.wheelRadiusMillimeters) / self.halfAxleLengthMillimeters) * \
                    (rightDiffDegrees - leftDiffDegrees)
            if turnRight:
                dthetaDegrees = -1*dthetaDegrees
            if turnLeft or turnRight:
                print('here', dthetaDegrees, leftDiffDegrees, rightDiffDegrees, turnLeft, turnRight)
            dtSeconds = timestampSecondsCurr - self.timestampSecondsPrev
                
        # Store current odometry for next time
        self.timestampSecondsPrev = timestampSecondsCurr        
        self.leftWheelDegreesPrev = leftWheelDegreesCurr
        self.rightWheelDegreesPrev = rightWheelDegreesCurr

        # Return linear velocity, angular velocity, time difference
        return dxyMillimeters, dthetaDegrees, dtSeconds
        
        
        
        
        