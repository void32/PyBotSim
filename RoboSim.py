#!/usr/bin/python

#setup interrupt handler
import signal
import sys
def signal_handler(signal, frame):
    print "You pressed Ctrl+C!"
    sys.exit(0)

import pylab
import Image
import numpy as np
from numpy import linalg
from matplotlib import pyplot as plt
import matplotlib.cm as cm
import random
import time

from matplotlib import animation


stepSize = 0.25
totalNumberOfTicks = 4000

class World:
    def __init__(self, pictureForBitmap='world.png'):
        print "create world"
        self.image = np.array(Image.open('world.png').convert('L'))

    def draw(self,figure):
        figure.gca().imshow(self.image, cmap = cm.Greys_r)

    def centerX(self):
        return(self.image.shape[1]/2)

    def centerY(self):
        return(self.image.shape[0]/2)

class Robot:    
    def reset(self):
        self.posX=random.randint(world.centerX()-100, world.centerX()+100)
        self.posY=random.randint(world.centerY()-100, world.centerY()+100)
        
        self.angle = random.uniform(0,2*np.pi)

        self.linearVelocity = random.uniform(20,50)
        self.angularVelocity = 0.0

        self.distSensorAngles = [-45*(2*3.14/360),0,45*(2*3.14/360)] #angles in rand.
        #self.distSensorAngles = [45*(2*3.14/360),2*45*(2*3.14/360),3*45*(2*3.14/360),4*45*(2*3.14/360),5*45*(2*3.14/360),6*45*(2*3.14/360),7*45*(2*3.14/360),8*45*(2*3.14/360)] #angles in rand.
        self.distSenorsCollistionCoordinate = {} #(x,y)
        self.sensedDistances = {}


        self.sensorLength = 300

    def __init__(self, x,y):
        self.reset()
        self.callbacks=[]
        

    def draw(self, figure):
        figure.gca().plot(self.posX,self.posY,'ro') #draw the robot

        #draw distance sensors/lasers
        for ds in self.distSensorAngles: 
            scale = 25
            figure.gca().plot([self.posX, scale*np.cos(self.angle+ds)+self.posX], [ self.posY, scale*np.sin(self.angle+ds)+self.posY], 'r-', lw=2)
            if ds in self.distSenorsCollistionCoordinate:
                figure.gca().plot([self.posX, self.distSenorsCollistionCoordinate.get(ds)[0]], [self.posY, self.distSenorsCollistionCoordinate.get(ds)[1]], 'g-',lw=1) 

    def move(self):
        movement = self.linearVelocity * stepSize 
        self.encodersim = np.array([(movement * np.cos(self.angle)), (movement * np.sin(self.angle))])        
        self.posX = self.posX + self.encodersim[0]
        self.posY = self.posY + self.encodersim[1]
        self.angle = self.angle + (self.angularVelocity * stepSize)
        
    def deltaPos(self):
        return self.encodersim

    def bitMapCollision(self):
        print("robot collided with a wall")
        self.reset()
        for f in self.callbacks:
            f()
            
    def addCallback(self, func):
        self.callbacks.append(func)

    def useDistSensors(self):
        self.sensedDistances.clear();
        self.distSenorsCollistionCoordinate.clear();
        for sensorAngle in self.distSensorAngles:
            r=col.rayCast(self.posX, self.posY,self.angle+sensorAngle,self.sensorLength)
            #r raycast result (whether it hit, (X, Y) )            
            if (r[0] == True):
                dist = np.sqrt((r[1][0]-self.posX)**2 + (r[1][1]-self.posY)**2)
                self.sensedDistances[sensorAngle]= dist # the sensor date
                self.distSenorsCollistionCoordinate[sensorAngle] = r[1] #used for drawing

class collisionManager:
    def __init__(self):
        self.collidables = []

    def setBitMap(self, bitMap, threshold=128):
        self.bitMap = bitMap
        self.threshold = threshold
        

    def addCollidable(self, drawable):
        self.collidables.append(drawable)

    def bitMapCollisionCheck(self, x, y):
        #if out of bound, we assume it to be not traversable
        try:
            return self.bitMap[y,x] < self.threshold            
        except IndexError:
            return True #This should be handled with if-statements no a try

    def detectionCollisions(self):
        #collidables (objects) against bit map
        for c in self.collidables:
            if(self.bitMapCollisionCheck(c.posX,c.posY)):
                return(c.bitMapCollision())

    #list of all the points crossed by the line from (x0, y0) to (x1, y1)
    def BresenhamAlg(self, x0, y0, x1, y1):
        result = []
        steep = np.abs(y1 - y0) > np.abs(x1 - x0);        
        if steep:
            x0, y0 = y0, x0 #swap(x0, y0)
            x1, y1 = y1, x1 #swap(x1, y1)

        if x0 > x1:
            x0, x1 = x1, x0 #swap(x0, x1)
            y0, y1 = y1, y0 #swap(y0, y1)

        deltaX = x1 - x0
        deltaY = np.abs(y1 - y0)
        error = 0
        y = y0
        if y0 < y1:
            yStep=1
        else:
            yStep=-1

        for x in range(x0,x1+1):
            if steep:
                result.append( (y,x) )
            else:
                result.append( (x,y) )

            error += deltaY
            if 2*error >= deltaX:
                y += yStep
                error -= deltaX
        return result


    def BresenhamRayCast(self, originX, originY, directionAngle, length):
        endX = int(length*np.cos(directionAngle)+originX)
        endY = int(length*np.sin(directionAngle)+originY)
        pixels = self.BresenhamAlg(int(originX),int(originY),endX,endY)

        l = [p for p in pixels if self.bitMapCollisionCheck(p[0],p[1])]

        try:
            minHit = min(np.sqrt((t[0]-originX)**2 + (t[1]-originY)**2) for t in l)
            firstHit = None
            for t in l:
                if minHit == np.sqrt((t[0]-originX)**2 + (t[1]-originY)**2):
                    firstHit = t
                    break
            
            if len(l)>0:
                return (True, firstHit)
        except:
            return (False, None)

    def rayCast(self, originX, originY, directionAngle, length):
        return self.BresenhamRayCast(originX, originY, directionAngle, length)

class Visualiser:
    def keyPress(self, event):
        if (event.key == 'escape'):
            print 'Exiting Sokoban...'
            exit()

    def __init__(self):
        self.drawables = []
        plt.ion()
        self.figure = plt.figure('Robot simulator plot')

    def add(self, drawable):
        self.drawables.append(drawable)

    def draw(self):
        self.figure.clf()
        for d in self.drawables:
            d.draw(self.figure)

        #self.figure.canvas.draw()     
        manager.canvas.draw()

##Use case example
world = World()
robot = Robot(world.centerX(), world.centerY())

col = collisionManager()
col.setBitMap(world.image)
col.addCollidable(robot)

vis = Visualiser()
vis.add(world)
vis.add(robot)

manager = pylab.get_current_fig_manager()


"""
def mainLogicCallback():
    robot.move()
    robot.useDistSensors()    
    col.detectionCollisions()
    vis.draw()

#Rather than using a loop we use a timed callback from the figure 
#this way we can interact with the window 
timer = vis.figure.canvas.new_timer(interval=100)
timer.add_callback(mainLogicCallback)
timer.start()
plt.show(block=True) #keep the window open
"""
