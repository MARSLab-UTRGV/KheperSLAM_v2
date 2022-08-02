##!/usr/bin/env python3
'''
Breezy Slam implementation for multiple KheperaIV robots
The initial position and orientation is needed for each robot.
'''

import argparse
import sys
from time import time

from breezyslam.algorithms import Deterministic_SLAM, RMHC_SLAM
from mines import load_data, MinesLaser
from pgm_utils import pgm_save
from KheperRobot import *

# Map size, scale
MAP_SIZE_PIXELS          = 800
MAP_SIZE_METERS          =  32

def mm2pix(mm):
        
    return int(mm / (MAP_SIZE_METERS * 1000. / MAP_SIZE_PIXELS))  
    
      

def handle_args() -> 'parser object':
    parser = argparse.ArgumentParser()
    parser.add_argument('--path', '-p', type=str, help='path to kheper_data.dat files')
    parser.add_argument('--number','-n', type=int, help='number of robots')
    parser.add_argument('--location', '-l', type=float, nargs='*', help='initial locations [(x1,y1),(x2,y2)...]')
    parser.add_argument('--orientation', '-o', type = float, nargs='*', help='initial orientations [rads1, rads2, ...]') 
    args = parser.parse_args()
    if len(args.location) != 2*args.number:
        print("Error, x/y coordinates not available for each robot")
        sys.exit()
    if len(args.orientation) != args.number:
        print("Error, initial orientation not available for all robots")

    return args

class MapGenerator:
    def __init__(self, args):
        print("MapGenerator created", args)
        self.robotLst = []
        self.lidarLst = []
        self.odomLst = []
        self.timestampLst = []
        self.slamLst = []
        m = 0
        for i in range(args.number):
            print(args.location, i)
            self.robotLst.append(KheperRobot(args.location[m], 
                                             args.location[m+1], 
                                             args.orientation[i],
                                             'kheper_data_'+str(i+1)))
            self.lidarLst.append([])
            self.odomLst.append([])
            self.timestampLst.append([])
            self.slamLst.append(RMHC_SLAM(MinesLaser(), MAP_SIZE_PIXELS, MAP_SIZE_METERS))
            m+=2

    def loadDataLists(self):
        for i in range(len(self.robotLst)):
            self.timestampLst[i], self.lidarLst[i], self.odomLst[i] = load_data('.', self.robotLst[i].filename)

    def __str__(self)->str:
        ret = ''
        for i in range(len(self.robotLst)):
            ret+= str(i) + str(self.robotLst[i]) + '\n'
        return ret

    def generate(self):
        velocityLst = []
        trajectoryLst = []
        for i in range(len(self.robotLst)):
            velocityLst.append([])
            trajectoryLst.append([])

        for i in range(len(self.robotLst)):
            for scanno in range(len(self.lidarLst[i])):
                velocity = self.robotLst[i].computePoseChange(self.timestampLst[i][scanno],self.odomLst[i][scanno])
                self.slamLst[i].update(self.lidarLst[i][scanno], velocity)
                x_mm, y_mm, theta_degrees = self.slamLst[i].getpos() #get new position
                trajectoryLst[i].append((x_mm, y_mm)) #add pos to trajectory

        #gen maps
        for i in range(len(self.robotLst)):
            mapbytes = bytearray(MAP_SIZE_PIXELS*MAP_SIZE_PIXELS)
            self.slamLst[i].getmap(mapbytes)
            #add path of bot
            for coords in trajectoryLst[i]:
                x_mm, y_mm = coords
                x_pix = mm2pix(x_mm)
                y_pix = mm2pix(y_mm)
                print("mapbytes len ", len(mapbytes), mapbytes[y_pix * MAP_SIZE_PIXELS + x_pix], x_pix, y_pix)
                mapbytes[y_pix * MAP_SIZE_PIXELS + x_pix] = 0
            #save map of each robot, need to do fusion for one map at some point
            pgm_save('{}dm.pgm'.format(self.robotLst[i].filename), mapbytes, (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS))





if __name__ == '__main__':
    args = handle_args()
    print(args.number, args.path)
    print('working')
    mg = MapGenerator(args)
    mg.loadDataLists()
    mg.generate()
    print(str(mg))
