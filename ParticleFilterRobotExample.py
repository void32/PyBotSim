#todo: propragate the paritcles using the movement of the robot

#evt. add randomness to sensors, color the particles after resample, visualize the gaussian distribution, interactive ui...

from RoboSim import *
import random
import matplotlib.pyplot as plt

numberOfParticles = 26
numberParticlesToResample = 8
additivCovarianceMatrix = np.array([[150,0],[0,150]])

class Particle(Robot):
    def __init__(self, x=world.centerX(), y=world.centerY(),angle=0): 
        Robot.__init__(self, x,y)
        self.posX = x
        self.posY = y
        self.angle = angle

    def draw(self, figure):
        figure.gca().plot(self.posX,self.posY,'b+') #draw the robot

        #draw distance sensors/lasers
        for ds in self.distSensorAngles: 
            scale = 10
            figure.gca().plot([self.posX, scale*np.cos(self.angle+ds)+self.posX], [ self.posY, scale*np.sin(self.angle+ds)+self.posY], 'b-', lw=2)


    def distToTarget(self,target):
        self.useDistSensors()
        suma = 0
        for key in target.sensedDistances.keys():
            try:
                if key in self.sensedDistances.keys():
                    suma += (self.sensedDistances[key]-target.sensedDistances[key])**2
                else:
                    suma = float("inf")
                    break
            except KeyError:
                print self.sensedDistances.keys()
                print target.sensedDistances.keys()
                exit()
        return -suma

    def updateWeight(self,target):
        self.weight = self.distToTarget(target)

    def bitMapCollision(self):
        print("Particle collided with a wall")

    #less than method    
    def __lt__(self, other):
        return self.weight < other.weight

    #tostring method
    def __str__(self):
        return("("+str(self.posX)+","+str(self.posY)+")")
        
class ParticleFilter():
    # Underlying steps:
    #  1. update the weights using the measurements
    #  2. resample with respect to the weights
    #  3. Propagate the particles in time using a model

    def updateWeights(self):
        self.target.useDistSensors()
        for p in self.particles:
            p.updateWeight(self.target)

    def resample(self):
        self.particles.sort(reverse=True) #select the lightest particles
        likelyParticles = self.particles[:numberParticlesToResample] #[p for p in self.particles[:numberParticlesToResample] if p.weight != -float("inf")]
        a = np.array([[p.posX,p.posY] for p in likelyParticles])
        self.mu = np.mean(a,axis=0)
        self.sigma = np.cov(a.T)+additivCovarianceMatrix

    def propagateParticles(self):
        self.mu += self.target.deltaPos()
        
        pass

    def reset(self):
        print("reset ParticleFilter")

        self.particles = [Particle(random.randint(world.centerX()-150, world.centerX()+150),random.randint(world.centerY()-150, world.centerY()+150), self.target.angle) for k in range(numberOfParticles)] 
        self.mu = np.array([0,0])
        self.sigma = None
        pass

    def __init__(self,target):
        self.target = target        
        self.reset()
        pass

    def draw(self, figure):    
        for p in self.particles:
            p.draw(figure)
        figure.gca().plot(self.mu[0],self.mu[1],'go') #draw the robot
        pass

    def genNewParticles(self):
        #perfect dist and angle sensors (no noise)  
        self.particles = [Particle(p[0],p[1],self.target.angle) for p in np.random.multivariate_normal(self.mu,self.sigma,numberOfParticles)]

pf = ParticleFilter(robot)
vis.add(pf)
robot.addCallback(pf.reset)

def mainLogicCallback():
    vis.draw()
    robot.move()
    robot.useDistSensors()
    if not col.detectionCollisions():
        pf.updateWeights()
        pf.resample()
        pf.propagateParticles()
        pf.genNewParticles()


#Rather than using a loop we use a timed callback from the figure 
#this way we can interact with the window 
timer = vis.figure.canvas.new_timer(interval=1)
timer.add_callback(mainLogicCallback)
timer.start()
plt.show(block=True) #keep the window open
