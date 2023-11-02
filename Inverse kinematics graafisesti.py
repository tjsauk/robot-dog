#Based on project by Christopher M. Locke and others
#https://github.com/cguweb-com/Arduino-Projects/tree/main/Nova-SM3
#https://leon70sml.blogspot.com/
#completement of movements and translation to python By Teemu Saukkio

''''Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files 
(the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, 
publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, 
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES 
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE 
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.'''



import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation 
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider, Button
import numpy as np
from vispy import scene, app, visuals
from vispy.scene.cameras import TurntableCamera
from vispy.visuals.transforms import STTransform
from vispy.scene.visuals import Markers, Line
#from vispy.scene.widgets import Button
from vispy.scene.widgets import Widget

#This works as configuration file that is used in the implementation of this code.
#It is here to make this code as similar as the implementation as possible
myconfig = {"COAX_SIZE" : 70,'FEMUR_SIZE' : 106.5, "TIBIA_SIZE" : 136,
            "BODY_LENGTH" : 282 , "BODY_WIDTH" : 78, "STEP_SIZE" : 80,
            "STEP_STANCE_TIME_PERCENTAGE" : 0.75, "STEP_SWING_HEIGHT_FRONT": 35, "STEP_STANCE_HEIGHT_FRONT" : 0,
            "STEP_STANCE_HEIGHT_REAR" : 0, "STEP_SWING_HEIGHT_REAR" : 35,
            "WALK_POSITION_FRONT_X" : 0, "WALK_POSITION_REAR_X" : 0,
            "WALK_POSITION_FRONT_Y" : 170, "WALK_POSITION_REAR_Y" :170,
            "COM_X_OFFSET" : -20,"COM_WEIGHT" : 80,
            "MAX_TURN_RADIUS" : 4000}#com paino grammoissa
#     


class Angle:
    def __init__(self, coax_degree, femur_degree, tibia_degree):
        self.coax_degree = coax_degree
        self.femur_degree = femur_degree
        self.tibia_degree = tibia_degree

class coords:
    def __init__(self,x_c,y_c,z_c):
        self.x = x_c
        self.y = y_c
        self.z = z_c

class FootDelays:
    def __init__(self):
        self.frontLeft = 0
        self.frontRight = 0
        self.rearLeft = 0
        self.rearRight = 0

class MyAsyncServo:
    def __init__(self):
        #contains all the same information as implemented code
        self.mCurrentDegree = -1
        self.mReverse = False
        self.mName = None
        self.initialized = False
        self.mHome = None
        self.mCorrection = None
        self.mZero = None
        self.mDegreeRange = None
        self.mLimitLow = 0.0
        self.mLimitHigh = None
    

    def setName(self, owner, name):
        #sets a name for one motor
        self.mName = f"{owner}-{name}"
    
    def setReverse(self, reverse):
        self.mReverse = reverse

    def Attach(self, srv_type, home, correction, zero):
        #initialises one servo with needed calibration information
        if not self.initialized:
            self.initialized = True

        
        self.mHome = home#this is the starting position on boot up
        self.mCorrection = correction#correction term used for aligning
        self.mZero = zero#this is degrees where motor is straight down
        #not used in this version but used in the implementation
        if srv_type == 'SERVO_20KG':
            self.mPwmMin = 120
            self.mPwmMax = 590
            self.mDegreeRange = 270
        else:  # SERVO_35KG SERVO_20KG
            self.mPwmMin = 140
            self.mPwmMax = 590
            self.mDegreeRange = 270.0

    def SetLimits(self, low, high):
        #is used to set maximum limits for motor to not to prake it. 
        self.mLimitLow = low
        self.mLimitHigh = high

    def cutoff_degrees(self, degree):
        # is used to check that motor doesnt go over limits
        if degree < self.mLimitLow:
            return self.mLimitLow
        elif degree > self.mLimitHigh:
            return self.mLimitHigh
        else:
            return degree


        
#these are for certain positionons or general movement.
    def go_home(self):
        #this is so called dum position so it doesnt know or care about heights. It only goes to certain position
        self.start_movement( self.mHome)

    def go_center(self):
        self.start_movement( self.mDegreeRange / 2.0)

    def start_movement(self,  degree):
        degree = self.cutoff_degrees(degree)
        
        self.mCurrentDegree = degree
        self.mIsMoving = True

class MyLeg:
    global myconfig
    def __init__(self, left, front):
        self.mFront = front#sets the position of the leg
        self.mLeft = left
        self.mName = None
        self.mTibia = MyAsyncServo()#assigns new motor for every leg
        self.mFemur = MyAsyncServo()
        self.mCoax = MyAsyncServo()
        self.mPositionPaw = coords(0,0,0)#current position
        self.mPositionKnee = coords(0,0,0)
        self.mPositionShoulder = coords(0,0,0)
        self.mPosition = coords(0,0,0)#offset position from leg origin for walking
        if self.mFront:#on etujalka
            self.mPosition.x = myconfig["WALK_POSITION_FRONT_X"]
            self.mPosition.y = myconfig["WALK_POSITION_FRONT_Y"]
            self.mPosition.z = 0
        else:#on takajalka
            self.mPosition.x = myconfig["WALK_POSITION_REAR_X"]
            self.mPosition.y = myconfig["WALK_POSITION_REAR_Y"]
            self.mPosition.z = 0
    
    def getPoints(self):
        #Returns all joint coordinates of a leg in hib joint origin
        
        x_points = [self.mPositionPaw.x,self.mPositionKnee.x,self.mPositionShoulder.x]
        y_points = [self.mPositionPaw.y,self.mPositionKnee.y,self.mPositionShoulder.y]
        if self.mLeft:
            z_points = [self.mPositionPaw.z+myconfig["COAX_SIZE"],self.mPositionKnee.z,self.mPositionShoulder.z]
        else:
            z_points = [self.mPositionPaw.z-myconfig["COAX_SIZE"],self.mPositionKnee.z,self.mPositionShoulder.z]
        return x_points, y_points, z_points

    def setName(self, name):
        self.mName = name 
        self.mTibia.setName(self.mName, "Tibia")
        self.mFemur.setName(self.mName, "Femur")
        self.mCoax.setName(self.mName, "Coax")

    def setReverse(self, value):
        self.mTibia.setReverse(value)
        self.mFemur.setReverse(value)
        self.mCoax.setReverse(value)

    def quaternion_multiply(self,v1, v2):
        q0,q1,q2,q3 = v1
        p0,p1,p2,p3 = v2
        
        result = [
            q0*p0-q1*p1-q2*p2-q3*p3,
            q1*p0+q0*p1-q3*p2+q2*p3,
            q2*p0+q3*p1+q0*p2-q1*p3,
            q3*p0-q2*p1+q1*p2+q0*p3
            ]

        return result
    def quaternion_conjugate(self, q):
        w, x, y, z = q
        return w, -x, -y, -z
    
    def rotateAxis(self, angle):#sin polvi kelt olkapää
        #this rotates the shoulder and knee points using quaternions
        #is called when leg position is updated 
        #quaternions for x rotation
        deg_rad = (2*math.pi)/360
        #oikealla puolella positiivinen kulma kääntää edestä katsottuna vastapäivään 0,0 kohdan ympäri. kulma radiaanenina
        unit_quaternion_x = [math.cos((self.mCoax.mZero-self.mCoax.mCurrentDegree)*deg_rad/2),math.sin((self.mCoax.mZero-self.mCoax.mCurrentDegree)*deg_rad/2),0,0]#self.mCoax.mCurrentDegree-45)*deg_rad
        
        #oikealla puolella oikealta katsottuna positiivinen kulma kääntää myötäpäivään
        unit_quaternion_z = [math.cos((self.mFemur.mZero-self.mFemur.mCurrentDegree)*deg_rad/2),0,0,math.sin((self.mFemur.mZero-self.mFemur.mCurrentDegree)*deg_rad/2)]#- self.mFemur.mZero ((self.mFemur.mCurrentDegree-85)*deg_rad)
        
        #point quaternion for shoulder. Allways starts at zero position
        #origin point of rotation is hip joint and for the leg its shoulder so compensate coordinates
        #vasemmalla positiivinen z
        if self.mLeft:
            shoulder_quaternion = [0,0,0,myconfig["COAX_SIZE"]]
            knee_quaternion = [0,0,myconfig["FEMUR_SIZE"],myconfig["COAX_SIZE"]]
        else:
            shoulder_quaternion = [0,0,0,-myconfig["COAX_SIZE"]]
            knee_quaternion = [0,0,myconfig["FEMUR_SIZE"],-myconfig["COAX_SIZE"]]
        #coordinate object 
        a = coords(0,0,0)
        b = coords(0,0,0)
        # Apply the rotation to the point of shoulder by applying p'=q^-1pq using x axis as rotation axis
        w,x,y,z = self.quaternion_multiply(self.quaternion_multiply(unit_quaternion_x, shoulder_quaternion), self.quaternion_conjugate(unit_quaternion_x))
        a.x = x
        a.y = y
        a.z =z
        self.mPositionShoulder = a
        
        #next rotate knee firs over x axis
        rotated_quaternion = self.quaternion_multiply(self.quaternion_multiply(unit_quaternion_z, knee_quaternion), self.quaternion_conjugate(unit_quaternion_z))
        #then over z axis
        w,x,y,z = self.quaternion_multiply(self.quaternion_multiply(unit_quaternion_x, rotated_quaternion), self.quaternion_conjugate(unit_quaternion_x))
        
        b.x = x
        b.y =y#+
    
        b.z =z
        #print("polvi koordinaatit "+self.mName + " "+str(b.x) + " "  +str(b.y) + " "  +str(b.z))
        self.mPositionKnee = b


    def home(self):
       #this is so called dum position so it doesnt know or care about heights. It only goes to certain position
        self.mTibia.go_home()
        self.mFemur.go_home()
        self.mCoax.go_home()

    def go_center(self):
        self.mTibia.go_center()
        self.mFemur.go_center()
        self.mCoax.go_center()

    def IKSolver(self,x, y, z):#https://community.robotshop.com/blog/show/mechdog-kinematics-trajectory-synchronization-gaits
        #solves the angles needed for motors of one leg
        M_PI = math.pi  # Pi constant
        MAX_LEN = (myconfig["FEMUR_SIZE"]+myconfig["TIBIA_SIZE"])

 
        z = z + myconfig["COAX_SIZE"]

        d1 = math.sqrt(z**2 + y**2 - myconfig["COAX_SIZE"]**2)
        d2 = math.sqrt(d1**2 + x**2)#etäisyys origon ja tassun pään välillä xy tasossa
        d3 = (d2**2- myconfig["FEMUR_SIZE"]**2 -myconfig["TIBIA_SIZE"]**2) / (2 * myconfig["FEMUR_SIZE"] * myconfig["TIBIA_SIZE"])#(d2**2- myconfig["FEMUR_SIZE"]**2 -myconfig["TIBIA_SIZE"]**2)
        theta1 = -math.atan2(y, z) - math.atan2(d1, -myconfig["COAX_SIZE"])
        #print(x,y,z,d3)
        theta3 = math.acos(d3)
        #print("IK solver "+ str(x)+" "+str(y)+" "+str(z)+" "+str(theta1)+" "+str(theta3)+" "+str(d3))
        theta2 = math.atan2(x, d1) - math.atan2(myconfig["TIBIA_SIZE"] * math.sin(theta3),myconfig["FEMUR_SIZE"] + myconfig["TIBIA_SIZE"] * math.cos(theta3))

   
        a = Angle(0,0,0)
    
        a.coax_degree = 180+self.mCoax.mZero+theta1*180/M_PI
        a.femur_degree = self.mFemur.mZero+theta2*180/M_PI
        a.tibia_degree = self.mTibia.mZero+theta3*180/M_PI

        return a

    def step_curve(self, percentage, rotation,y_rot, x,z):
        #y_steer = rot_point saa arvoja 1 ja -1 väliltä. Vasemmalle käännös on 1
        #this takes the walk cycle percentage of this leg and gets the coordinates for it using interpolation
        #askelpituus on maksimissaan 1
        step_lenght = 0#tätä pitää muuttaa siten että säde jalkaan/säde keskelle kasvattaa tai pienetää
        x_compensate = 0
        z_compensate = 0
        #koska jalat eivät ole kehon keskipisteen mukaan koordinaaterissa tarvitaan kompensaatio kertoimet
        #kehoon nähden z positiivinen on vasemmalla ja ax positiivinen edessä
        if self.mLeft:
            z_compensate = myconfig["BODY_WIDTH"]/2
        else:
            z_compensate = -myconfig["BODY_WIDTH"]/2
        if self.mFront:
            x_compensate = myconfig["BODY_LENGTH"]/2
        else:
            x_compensate = -myconfig["BODY_LENGTH"]/2

        if x == 0 and z == 0:#jos pyöritään paikallaan
            step_lenght = myconfig["STEP_SIZE"] * y_rot/2
        elif y_rot != 0:#käännytään
            #kääntösäteen origon etäisyys kehon keskeltä
            D0 = (rotation[0]**2+rotation[1]**2)**0.5
            #jalan etäisyys kääntösäteen origosta
            D1 = ((rotation[0]-self.mPositionPaw.x)**2+(rotation[1]-self.mPositionPaw.y)**2)**0.5
            #kompensoidaan aseklta pienemmäksi ja isommaksi riippuen siitä missä jalka on
            step_lenght = myconfig["STEP_SIZE"] *((x)**2+(z)**2)**0.5/2*(D1/D0)
        else:#kävellään suoraan
            step_lenght = myconfig["STEP_SIZE"] *(x**2+z**2)**0.5/2
        
        #lasketaan vastainen viereinen ja hypotenuusa
        viereinen = (rotation[0]-self.mPositionPaw.x+x_compensate)#z=viereinen=kääntöpisteen x - jalan kehon keskipisteen suhteen x   
        vastainen = (rotation[1]-self.mPositionPaw.z+z_compensate)
        hypotenuusa = (viereinen**2+vastainen**2)**0.5

        step_size_x = 0
        step_size_z = 0
        if y_rot == 0:#ei käännytä ollenkaan jolloin mennään vaan x tai z suuntaan tasaisesti
            step_size_x = myconfig["STEP_SIZE"] * x/2#näihin pitää ehkä laittaa jotain muuta kun tämä esim askelvektori
            step_size_z = myconfig["STEP_SIZE"] * z/2
            #print("Askelkoot x ja z suuntaan ei käänny " + self.mName + " " + str(step_size_x) + " " + str(step_size_z))
        else:#käännytään ja kävellään
            step_size_z = step_lenght*(viereinen/hypotenuusa)#näihin pitää ehkä laittaa jotain muuta kun tämä esim askelvektori
            step_size_x = step_lenght*(vastainen/hypotenuusa)
            #print("Askelkoot x ja z suuntaan kääntyy " + self.mName + " " + str(step_size_x) + " " + str(step_size_z))
        #kun käännytään paikallaan vasemmalle etu vasen pitäis olla negatiivinen x positiivinen z
        #etu oikea pitäis olla positiivinen x positiivinen z
        #taka vasen pitäisi olla negatiivinen x negatiivinen z taka oikea positiivinen x negatiivinen z

        
        step_stance_h = 0  # Replace with the actual value
        step_swing_h = 0  # Replace with the actual value
        stance_percentage = myconfig["STEP_STANCE_TIME_PERCENTAGE"]  # Replace with the actual value
        
       

        if self.mFront:
            step_swing_h = myconfig["STEP_SWING_HEIGHT_FRONT"]
            step_stance_h = myconfig["STEP_STANCE_HEIGHT_FRONT"]
        else:
            step_swing_h = myconfig["STEP_SWING_HEIGHT_REAR"]
            step_stance_h = myconfig["STEP_STANCE_HEIGHT_FRONT"]

        coord = coords(0,0,0)

        if percentage <= stance_percentage:
            perc = percentage / stance_percentage

            # Coordinates of 4 points to interpolate
            x0 = step_size_x / 2
            y0 = 0
            z0 = step_size_z / 2

            x1 = step_size_x / 2 - step_size_x / 3
            y1 = 0
            z1 = step_size_z / 2 - step_size_z / 3

            x2 = step_size_x / 2 - step_size_x * 2 / 3
            y2 = 0
            z2 = step_size_z / 2 - step_size_z * 2 / 3

            x3 = step_size_x / 2 - step_size_x
            y3 = 0
            z3 = step_size_z / 2 - step_size_z

            coord.x = ((1 - perc) * ((1 - perc) * ((1 - perc) * x0 + perc * x1) + perc * ((1 - perc) * x1 + perc * x2)) +
                    perc * ((1 - perc) * ((1 - perc) * x1 + perc * x2) + perc * ((1 - perc) * x2 + perc * x3)))
            coord.y = -(((1 - perc) * ((1 - perc) * ((1 - perc) * y0 + perc * y1) + perc * ((1 - perc) * y1 + perc * y2)) +
                        perc * ((1 - perc) * ((1 - perc) * y1 + perc * y2) + perc * ((1 - perc) * y2 + perc * y3))))
            coord.z = ((1 - perc) * ((1 - perc) * ((1 - perc) * z0 + perc * z1) + perc * ((1 - perc) * z1 + perc * z2)) +
                    perc * ((1 - perc) * ((1 - perc) * z1 + perc * z2) + perc * ((1 - perc) * z2 + perc * z3)))

            return coord
        else:
            perc = (percentage - stance_percentage) / (1.0 - stance_percentage)

        # Coordinates of 4 points to interpolate
            x0 = step_size_x / 2 - step_size_x
            y0 = 0
            z0 = step_size_z / 2 - step_size_z

            x1 = step_size_x / 2 - step_size_x + step_size_x / 5
            y1 = step_swing_h * 4 / 3
            z1 = step_size_z / 2 - step_size_z + step_size_z / 5

            x2 = step_size_x / 2 - step_size_x / 5
            y2 = step_swing_h * 4 / 3
            z2 = step_size_z / 2 - step_size_z / 5

            x3 = step_size_x / 2
            y3 = 0
            z3 = step_size_z / 2
            coord.x = ((1 - perc) * ((1 - perc) * ((1 - perc) * x0 + perc * x1) + perc * ((1 - perc) * x1 + perc * x2)) +
                    perc * ((1 - perc) * ((1 - perc) * x1 + perc * x2) + perc * ((1 - perc) * x2 + perc * x3)))
            coord.y = -(((1 - perc) * ((1 - perc) * ((1 - perc) * y0 + perc * y1) + perc * ((1 - perc) * y1 + perc * y2)) +
                        perc * ((1 - perc) * ((1 - perc) * y1 + perc * y2) + perc * ((1 - perc) * y2 + perc * y3))))
            coord.z = ((1 - perc) * ((1 - perc) * ((1 - perc) * z0 + perc * z1) + perc * ((1 - perc) * z1 + perc * z2)) +
                    perc * ((1 - perc) * ((1 - perc) * z1 + perc * z2) + perc * ((1 - perc) * z2 + perc * z3)))

            return coord



    def GoPosition(self,x,y,z):#Sets leg to certain position with out knowing the last position
        #sets paw to certain xyz position relative to its zero position
        self.mPosition.x = x
        self.mPosition.y = y
        self.mPosition.z = z
        a = self.IKSolver(x, y,z)
        
        self.mPositionPaw.x = x
        self.mPositionPaw.y = y
        self.mPositionPaw.z = z
        self.rotateAxis(a)
        #print("Suoraan asentoon ",str(x),str(y),str(z))
        self.mTibia.start_movement(a.tibia_degree)
        self.mFemur.start_movement( a.femur_degree)
        self.mCoax.start_movement( a.coax_degree)
    
    def BackToPosition(self):#returns to current non walking position
        #sets paw to certain xyz position relative to its zero position
        x= self.mPosition.x 
        y = self.mPosition.y 
        z = self.mPosition.z 
        #print(x,y,z)
        a = self.IKSolver(x, y,z)
        
        self.mPositionPaw.x = x
        self.mPositionPaw.y = y
        self.mPositionPaw.z = z
        #print(self.mPositionPaw.y)
        #print("Suoraan asentoon ",str(x),str(y),str(z))
        self.mTibia.start_movement(a.tibia_degree)
        self.mFemur.start_movement( a.femur_degree)
        self.mCoax.start_movement( a.coax_degree)
        self.rotateAxis(a)
        

    def WalkWithDelta(self, delta):
        #sets paw to certain xyz position relative to its standing position position
        #print("deltalla asentoon sento ennen "+ str(self.mPositionPaw.x) + " " + str(self.mPositionPaw.y) + " " + str(self.mPositionPaw.z))

        self.mPositionPaw.x= self.mPosition.x +delta.x
        self.mPositionPaw.y=self.mPosition.y+delta.y
        self.mPositionPaw.z=self.mPosition.z+delta.z

    def addToWalk(self, x,y,z):
        #adds to upcoming walk coordinates to adjust in case of COM not being in the right place
        #print("deltalla asentoon sento ennen "+ str(self.mPositionPaw.x) + " " + str(self.mPositionPaw.y) + " " + str(self.mPositionPaw.z))

        self.mPositionPaw.x+=x
        self.mPositionPaw.y+=y
        self.mPositionPaw.z+=z

    def updateWalkPosition(self):
        #rotates the motors to the current temperary position
        #print("deltalla asentoon sento ennen "+ str(self.mPositionPaw.x) + " " + str(self.mPositionPaw.y) + " " + str(self.mPositionPaw.z))
        new_x = self.mPositionPaw.x
        new_y = self.mPositionPaw.y
        new_z = self.mPositionPaw.z
        #print("Asento jälkeen "+ str(self.mPositionPaw.x) + " " + str(self.mPositionPaw.y) + " " + str(self.mPositionPaw.z))
        a = self.IKSolver(new_x, new_y, new_z)

    
        self.mTibia.start_movement(a.tibia_degree)
        self.mFemur.start_movement( a.femur_degree)
        self.mCoax.start_movement(a.coax_degree)
        self.rotateAxis(a)
        
    def PositionWithDelta(self, x,y,z):
        #updates the current the current standing position
        self.mPosition.x+=x
        self.mPosition.y+=y
        self.mPosition.z+=z

    def updateStandPosition(self):
        a = self.IKSolver(self.mPosition.x,self.mPosition.y, self.mPosition.z)
    
        self.mTibia.start_movement(a.tibia_degree)
        self.mFemur.start_movement( a.femur_degree)
        self.mCoax.start_movement(a.coax_degree)
        self.mPositionPaw.x=self.mPosition.x
        self.mPositionPaw.y=self.mPosition.y
        self.mPositionPaw.z=self.mPosition.z
        self.rotateAxis(a)
        

    def DoStep(self, percentage, rot_point,y_rot, x, z):
        #calculates the needed position in legs local coordinates
        #steer angles are directions to turn to and coordinates are directions to walk to
        #print("DoStep inputs " + str(y_rot) + " " + str(x) + " " + str(z))
        delta = self.step_curve(percentage, rot_point,y_rot, x ,z)
        #print("DoStep deltas " + str(delta.x) + " " + str(delta.y) + " " + str(delta.z))
        self.WalkWithDelta(delta)


class MyBody:
    global myconfig
    def __init__(self):
        self.mLeftFront = MyLeg(True,True)#left front
        self.mLeftRear = MyLeg(True,False)
        self.mRightFront = MyLeg(False,True)
        self.mRightRear = MyLeg(False,False)

        self.mRightFront.setReverse(True)
        self.mRightRear.setReverse(True)

        self.mRightRear.setName("R rear")
        self.mRightFront.setName("R front")
        self.mLeftRear.setName("L rear")
        self.mLeftFront.setName("L front")
        self.mFootDelays = FootDelays()
        self.mWalkStep = 0
        #values used for steering
        self.sVelocity = 0
        self.sDegree_x = 0
        self.sDegree_y = 0
        self.sDegree_z = 0
        self.sDirection_x = 0
        self.sDirection_y = 0
        self.sDirection_z = 0
        self.sRamp = 0
        #this is for bodys wireframe coordinates relative to center of pody
        #y positive is down, x positive if forward , z positive is ????
        self.wfLeftFront = coords(0,0,0)
        self.wfLeftRear = coords(0,0,0)
        self.wfRightFront = coords(0,0,0)
        self.wfRightRear = coords(0,0,0)
        #set the initial wire frame position to straight in xz plane and y zero
        self.wfLeftFront.x = myconfig["BODY_LENGTH"]/2
        self.wfRightFront.x = myconfig["BODY_LENGTH"]/2
        self.wfLeftRear.x = -myconfig["BODY_LENGTH"]/2
        self.wfRightRear.x = -myconfig["BODY_LENGTH"]/2
        #set the initial wireframe positions offset from center in z aaxis
        self.wfLeftFront.z = myconfig["BODY_WIDTH"]/2
        self.wfRightFront.z = -myconfig["BODY_WIDTH"]/2
        self.wfLeftRear.z = myconfig["BODY_WIDTH"]/2
        self.wfRightRear.z = -myconfig["BODY_WIDTH"]/2
        #initial wireframeposition y component wireframe y should be 0 if back and front are in same height
        self.wfLeftFront.y = (myconfig["WALK_POSITION_REAR_Y"]-myconfig["WALK_POSITION_FRONT_Y"])/2#if initialised with a difference between front and back
        self.wfRightFront.y = (myconfig["WALK_POSITION_REAR_Y"]-myconfig["WALK_POSITION_FRONT_Y"])/2#if initialised with a difference between front and back
        self.wfLeftRear.y = (myconfig["WALK_POSITION_FRONT_Y"]-myconfig["WALK_POSITION_REAR_Y"])/2#if initialised with a difference between front and back
        self.wfRightRear.y = (myconfig["WALK_POSITION_FRONT_Y"]-myconfig["WALK_POSITION_REAR_Y"])/2#if initialised with a difference between front and back

    #support triangle corners
        self.wfSupportTriangle = []
        self.wfCOM = coords(myconfig["COM_X_OFFSET"],0,0)#jos keho kääntyy käännä tätä

    def setGait(self, mode):#setGait("GAIT_WALK")
        if mode == "GAIT_WALK":
            self.mFootDelays.frontLeft = 0.00
            self.mFootDelays.frontRight = 0.50
            self.mFootDelays.rearLeft = 0.75
            self.mFootDelays.rearRight = 0.25
        elif mode == "GAIT_TROT":
            self.mFootDelays.frontLeft = 0.00
            self.mFootDelays.frontRight = 0.50
            self.mFootDelays.rearLeft = 0.50
            self.mFootDelays.rearRight = 0.00
        elif mode == "JACK_SPARROW":
            self.mFootDelays.frontLeft = 0.00
            self.mFootDelays.frontRight = 0.75
            self.mFootDelays.rearLeft = 0.25
            self.mFootDelays.rearRight = 0.50
        elif mode == "GAIT_PACE":
            self.mFootDelays.frontLeft = 0.00
            self.mFootDelays.frontRight = 0.50
            self.mFootDelays.rearLeft = 0.00
            self.mFootDelays.rearRight = 0.50
        elif mode == "GAIT_CANTER":
            self.mFootDelays.frontLeft = 0.00
            self.mFootDelays.frontRight = 0.30
            self.mFootDelays.rearLeft = 0.70
            self.mFootDelays.rearRight = 0.00
        else:
            self.mFootDelays.frontLeft = 0.00
            self.mFootDelays.frontRight = 0.00
            self.mFootDelays.rearLeft = 0.50
            self.mFootDelays.rearRight = 0.50

    def Home(self):
       #this is so called dum position so it doesnt know or care about heights. It only goes to certain position
        self.mLeftFront.home()
        self.mLeftRear.home()
        self.mRightFront.home()
        self.mRightRear.home()


    #def GoTo(self, mills, duration, degree_x, degree_y, degree_z, x, y, z, ramp):#make sure that it doesnt work if in home position
        #lf, rf, lr, rr = Coords(), Coords(), Coords(), Coords()
        #IK_Solver(MYDEG_TO_RAD(degree_x), MYDEG_TO_RAD(degree_y), MYDEG_TO_RAD(degree_z), x, y, z, lf, rf, lr, rr)
        #self.mLeftFront.GoPosition(mills, lf, duration, ramp)
        #self.mLeftRear.GoPosition(mills, lr, duration, ramp)
        #self.mRightFront.GoPosition(mills, rf, duration, ramp)
        #self.mRightRear.GoPosition(mills, rr, duration, ramp)

    def getLegPoints(self, leg):
        #arguments LF RF LR RR indicating the leg 
        #return coordinates relative to body origin
        if leg =="LF":
            x_LF,y_LF,z_LF = self.mLeftFront.getPoints()#paw knee shoulder
            for i in range(3):
                x_LF[i] += self.wfLeftFront.x#adds body measurements for paw knee and shoulder point
                y_LF[i] += self.wfLeftFront.y
                z_LF[i] += self.wfLeftFront.z
            return x_LF,y_LF,z_LF
        elif leg =="RF":
            x_RF,y_RF,z_RF = self.mRightFront.getPoints()
            for i in range(3):
                x_RF[i] += self.wfRightFront.x
                y_RF[i] += self.wfRightFront.y
                z_RF[i] += self.wfRightFront.z
            return x_RF,y_RF,z_RF
        elif leg =="LR":
            x_LR,y_LR,z_LR = self.mLeftRear.getPoints()
            for i in range(3):
                x_LR[i] += self.wfLeftRear.x
                y_LR[i] += self.wfLeftRear.y
                z_LR[i] += self.wfLeftRear.z
            return x_LR,y_LR,z_LR
        elif leg =="RR":
            x_RR,y_RR,z_RR = self.mRightRear.getPoints()
            for i in range(3):
                x_RR[i] += self.wfRightRear.x
                y_RR[i] += self.wfRightRear.y
                z_RR[i] += self.wfRightRear.z
            return x_RR,y_RR,z_RR
            
            
    def WalkDirection(self, velocity, degree_x, degree_y, degree_z, x, y, z, ramp):
        #new values used in movement
        self.sVelocity = velocity
        self.sDegree_x = degree_x#pyörähdys x akselin ympäri
        self.sDegree_y = degree_y#pyörähdys y akselin ympäri
        self.sDegree_z = degree_z#pyörähdys z akselin ympäri
        self.sDirection_x = x#translaatio suunnat
        self.sDirection_y = y
        self.sDirection_z = z
        self.sRamp = ramp
    
    def getSupportTriangle(self,perc_LF,perc_RF,perc_LR,perc_RR):
        #order of return is the one leg that is back or front alone, Leg that is rising next, final leg
        if perc_LF >=0.74:#etuvasen ylhäällä
           #paw knee shoulder
            x_LR,y_LR,z_LR = self.getLegPoints("LR")#[x_LR[0],y_LR[0],z_LR[0]]
            x_RF,y_RF,z_RF = self.getLegPoints("RF")#[x_RF[0],y_RF[0],z_RF[0]]
            x_RR,y_RR,z_RR = self.getLegPoints("RR")#[x_RR[0],y_RR[0],z_RR[0]]
            return [[x_RF[0],y_RF[0],z_RF[0]],[x_RR[0],y_RR[0],z_RR[0]],[x_LR[0],y_LR[0],z_LR[0]]]
        elif perc_RF >=0.74:#etuoikea ylhäällä
            x_LF,y_LF,z_LF = self.getLegPoints("LF")#[x_LF[0],y_LF[0],z_LF[0]]
            x_LR,y_LR,z_LR = self.getLegPoints("LR")#[x_LR[0],y_LR[0],z_LR[0]]
            x_RR,y_RR,z_RR = self.getLegPoints("RR")#[x_RR[0],y_RR[0],z_RR[0]]
            return [[x_LF[0],y_LF[0],z_LF[0]],[x_LR[0],y_LR[0],z_LR[0]],[x_RR[0],y_RR[0],z_RR[0]]]
            
        elif perc_RR >=0.74:#taka oikea ylhäällä
            x_LF,y_LF,z_LF = self.getLegPoints("LF")#[x_LF[0],y_LF[0],z_LF[0]]
            x_LR,y_LR,z_LR = self.getLegPoints("LR")#[x_LR[0],y_LR[0],z_LR[0]]
            x_RF,y_RF,z_RF = self.getLegPoints("RF")#[x_RF[0],y_RF[0],z_RF[0]]
            return [[x_LR[0],y_LR[0],z_LR[0]],[x_RF[0],y_RF[0],z_RF[0]],[x_LF[0],y_LF[0],z_LF[0]]]
            
        elif perc_LR >=0.74:#taka vasen ylhäällä
            x_LF,y_LF,z_LF = self.getLegPoints("LF")#[x_LF[0],y_LF[0],z_LF[0]]
            x_RF,y_RF,z_RF = self.getLegPoints("RF")#[x_RF[0],y_RF[0],z_RF[0]]
            x_RR,y_RR,z_RR = self.getLegPoints("RR")#[x_RR[0],y_RR[0],z_RR[0]]
            
            return [[x_RR[0],y_RR[0],z_RR[0]],[x_LF[0],y_LF[0],z_LF[0]],[x_RF[0],y_RF[0],z_RF[0]]]    

    def COMUpdateXY(self):
        x_LF,y_LF,z_LF = self.getLegPoints("LF")#[x_LF[0],y_LF[0],z_LF[0]]
        x_LR,y_LR,z_LR = self.getLegPoints("LR")#[x_LR[0],y_LR[0],z_LR[0]]
        x_RF,y_RF,z_RF = self.getLegPoints("RF")#[x_RF[0],y_RF[0],z_RF[0]]
        x_RR,y_RR,z_RR = self.getLegPoints("RR")#[x_RR[0],y_RR[0],z_RR[0]]
        #Massakeskipiste suhteessa pisteeseen 
        #Lasketaan jalkojen vaikutus suhteessa kehon keskipisteeseen COM_WEIGHT
        m = myconfig["COM_WEIGHT"]
        x_off = myconfig["COM_X_OFFSET"]
        #x_com = m_1x_1+m_2x_2/m_1+m_2
        x_com = (m*(x_LF[1]-x_off)+m*(x_LR[1]-x_off)+m*(x_RF[1]-x_off)+m*(x_RR[1]-x_off)+4*m*(x_off*(-2)))/(8*m)
        z_com = (m*z_LF[1]+m*z_LR[1]+m*z_RF[1]+m*z_RR[1])/(4*m)
        
        self.wfCOM.x = myconfig["COM_X_OFFSET"]+x_com
        self.wfCOM.z = z_com

    def quaternion_multiply(self,v1, v2):
        q0,q1,q2,q3 = v1
        p0,p1,p2,p3 = v2
        
        result = [
            q0*p0-q1*p1-q2*p2-q3*p3,
            q1*p0+q0*p1-q3*p2+q2*p3,
            q2*p0+q3*p1+q0*p2-q1*p3,
            q3*p0-q2*p1+q1*p2+q0*p3
            ]

        return result
    def quaternion_conjugate(self, q):
        w, x, y, z = q
        return w, -x, -y, -z
    def rotateBody(self, angle_x , angle_z):
    
        #rotates the bodys wire frame and returns leg delta coordinates
        deg_rad = (2*math.pi)/360
        #oikealla puolella positiivinen kulma kääntää edestä katsottuna vastapäivään 0,0 kohdan ympäri. kulma radiaanenina
        unit_quaternion_x = [math.cos((angle_x)*deg_rad/2),math.sin((angle_x)*deg_rad/2),0,0]#self.mCoax.mCurrentDegree-45)*deg_rad
        
        #oikealla puolella oikealta katsottuna positiivinen kulma kääntää myötäpäivään
        unit_quaternion_z = [math.cos((angle_z)*deg_rad/2),0,0,math.sin((angle_z)*deg_rad/2)]#- self.mFemur.mZero ((self.mFemur.mCurrentDegree-85)*deg_rad)
        #the corner points of the wire frame currently
        
        LF1 = [self.wfLeftFront.x,self.wfLeftFront.y,self.wfLeftFront.z]
        LR1 = [self.wfLeftRear.x,self.wfLeftRear.y,self.wfLeftRear.z]
        RR1 = [self.wfRightRear.x,self.wfRightRear.y,self.wfRightRear.z]
        RF1 = [self.wfRightFront.x,self.wfRightFront.y,self.wfRightFront.z]
        
        list1 = [LF1,LR1,RR1,RF1]
        list2 = [self.wfLeftFront,self.wfLeftRear,self.wfRightRear,self.wfRightFront]
        result = []
        #rotate all points with the given angles and add the result to list
        for i in range(4):
            quaternion = [0,list1[i][0],list1[i][1],list1[i][2]]
        #next rotate knee firs over x axis
            rotated_quaternion = self.quaternion_multiply(self.quaternion_multiply(self.quaternion_conjugate(unit_quaternion_z), quaternion), unit_quaternion_z)
        #then over z axis
            w,x,y,z = self.quaternion_multiply(self.quaternion_multiply(self.quaternion_conjugate(unit_quaternion_x), rotated_quaternion), unit_quaternion_x)
            #last step is to calculate the resulting delta values from the rotation by substracting the old coordinates from the new coordinates
            result.append([list1[i][0]-x,list1[i][1]-y,list1[i][2]-z])
            list2[i].x = x
            list2[i].y = y
            list2[i].z = z
            #rotate the COM allso
        quaternion = [0,self.wfCOM.x,self.wfCOM.y,self.wfCOM.z]
        #next rotate knee firs over x axis
        rotated_quaternion = self.quaternion_multiply(self.quaternion_multiply(self.quaternion_conjugate(unit_quaternion_z), quaternion), unit_quaternion_z)
        #then over z axis
        w,x,y,z = self.quaternion_multiply(self.quaternion_multiply(self.quaternion_conjugate(unit_quaternion_x), rotated_quaternion), unit_quaternion_x)
        self.wfCOM.x = x
        self.wfCOM.y = y
        self.wfCOM.z = z
            
        return result
            
        
        
    def distance_to_line(point, line_start, line_end):
    # Calculate the direction vector of the line segment
        line_dir = (line_end[0] - line_start[0], line_end[2] - line_start[2])

    # Calculate the vector from the line start to the point
        to_point = (point[0] - line_start[0], point[2] - line_start[2])

    # Calculate the magnitude of the line direction vector
        line_length = (line_dir[0] ** 2 + line_dir[2] ** 2) ** 0.5

    

    # Calculate the projection of the point onto the line
        t = (to_point[0] * line_dir[0] + to_point[2] * line_dir[2]) / (line_length ** 2)

    

    # Calculate the closest point on the line
        closest_point = (line_start[0] + t * line_dir[0], line_start[2] + t * line_dir[2])

    # Calculate the distance between the point and the closest point on the line
        distance = ((point[0] - closest_point[0]) ** 2 + (point[2] - closest_point[2]) ** 2) ** 0.5

        return distance    
        
        
    def PositionUpdate(self):#make sure that it doesnt work if in home position
        #this applies the set parameters for current valc cycle
        total_step_points = 50
        
        newDuration = myconfig["STEP_SIZE"] / self.sVelocity
        #new duration on yhden askel syklin kesto
        #siitä saadaan time delta joka on yhden sykli askeleen kesto
        time_delta = newDuration/50#this is the duration of time it takes to complete one servo movement
        perc_FL = ((self.mWalkStep + int(self.mFootDelays.frontLeft * total_step_points)) % total_step_points )/ total_step_points
        perc_FR = ((self.mWalkStep + int(self.mFootDelays.frontRight * total_step_points)) % total_step_points) / float(total_step_points)
        perc_RL = (self.mWalkStep + int(self.mFootDelays.rearLeft * total_step_points)) % total_step_points / total_step_points
        perc_RR = (self.mWalkStep + int(self.mFootDelays.rearRight * total_step_points)) % total_step_points / total_step_points
        if self.sDirection_x != 0 or self.sDirection_z != 0 or self.sDegree_y != 0:#halutaan kävellä tai kääntyä
            #print(self.mWalkStep)
            

            #keskipiste käännön määrittelevälle säteelle
            rot_point = [0,0]
            if self.sDegree_y != 0:#halutaan kääntyä kun liikutaan
                #käännöspiste lasketaan ajatuksella että y säätää sen etäisyyttä välillä maksimi ja minimi
                #käännöspiste on kohtisuorassa kulkusuuntaan ja koska kulkua ohjataan arvoilla -1,1 väliltä saadaan että z ohjaus on cos ja x ohjaus sin
                #tällöin saadaan yksikköympyrän piste jossa käännön keskipiste on kaavasta P = (cos,sin) ja kertomalla se kääntösäteellä R saadaan P = (R*cos,R*sin)=(R*z,R*x)
                rot_point = [(myconfig["BODY_LENGTH"]+myconfig["MAX_TURN_RADIUS"]*(1-self.sDegree_y))*self.sDirection_z,(myconfig["BODY_LENGTH"]+myconfig["MAX_TURN_RADIUS"]*(1-self.sDegree_y))*self.sDirection_x]#x ,z
                #print(rot_point)
            #pitäisi tässä kohtaa katsoa kuinka paljon pitäisi muuttaa kehon asentoa jotta ei kaaduta. Ja päivittää jalkojen vakio asentoa mPosition jotta 
            #kävelysyklin aikana asento muuttuu halutuksi sillä kävely askel kutsuu delta muunnosta jonka aikana voidaan päivittää myös muuta
            COM_cycle = coords(0,0,0)
            self.mRightFront.DoStep( perc_FR,rot_point,self.sDegree_y,self.sDirection_x,self.sDirection_z)
            self.mLeftFront.DoStep( perc_FL,rot_point,self.sDegree_y,self.sDirection_x,self.sDirection_z)
            self.mRightRear.DoStep( perc_RR,rot_point,self.sDegree_y,self.sDirection_x,self.sDirection_z)
            self.mLeftRear.DoStep( perc_RL,rot_point,self.sDegree_y,self.sDirection_x,self.sDirection_z)
          

            #this can be used to add something to walk position variables addToWalk(x,y,z) to apply com cycle
            #when walk starts the com should be close to right rear foot and go next to right front leg
            
            #Seuraavan askeleen koordinaatit on nyt paw muuttujassa eli väliaikaisessa asennossa

            #seuraavaksi pitäisi laskea kulmat jotta nähdään pitääkö comin takia tehä muita muutoksia 
            
            self.mRightFront.updateWalkPosition()#muutetaan dostep siten ettei moottoreita vielä ajeta vaan ne palauttaa moottorien kulmat
            self.mLeftFront.updateWalkPosition()#sillä asento saadaan laskettua ilmankin liikkumista
            self.mRightRear.updateWalkPosition()
            self.mLeftRear.updateWalkPosition()

            #sitten tarkistetaan com pisteen sijainti ja jso pitää muuttaa lasketaan uudet kulmat
            self.wfSupportTriangle = self.getSupportTriangle(perc_FL,perc_FR,perc_RL,perc_RR) 

            self.COMUpdateXY()
            #A,B,C = self.getSupportTriangle(perc_FL,perc_FR,perc_RL,perc_RR)
            #print(self.distance_to_line([self.wfCOM.x,self.wfCOM.y,self.wfCOM.z], A, C))
            #kun on com oikein ja lopulliset kulmat niin ajetaan moottorit pikeaan asentoon
            if False:
                with open("COM liike tarkkailu.txt", 'a') as file:
                    file.write(str(perc_FL) + "," + str(perc_FR) + ","+str(perc_RL) + ","+ str(perc_RR) + ","
                            +str(self.wfSupportTriangle[0][0]) + ","+str(self.wfSupportTriangle[0][1]) + ","+str(self.wfSupportTriangle[0][2]) + ","
                            +str(self.wfSupportTriangle[1][0]) + ","+str(self.wfSupportTriangle[1][1]) + ","+str(self.wfSupportTriangle[1][2]) +","
                            +str(self.wfSupportTriangle[2][0]) + ","+str(self.wfSupportTriangle[2][1]) + ","+str(self.wfSupportTriangle[2][2]) + ","
                                + str(self.wfCOM.x)+","+ str(self.wfCOM.y)+"," + str(self.wfCOM.z)+'\n') 
            self.mWalkStep += 1
        elif self.sDirection_y != 0 or self.sDegree_x != 0 or self.sDegree_z != 0:#halutaan muuttaa asentoa ilman kävelyä
            y_move_length = 40#this is the dimension that the robots body can move in y direction during one cycle period defined by newDuration
            rotation_len = 20 #this is the angle that can be changed during one cycle

            #add a check that makes sure nothing is over rotated or moved!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1

            #täällä tarkoitus on asettaa keholle asentoja muuttamalla jalkojen mPositio ta joka on "vakio asento tällähetkellä"
            #change in y is 0 or positive / negative. and proportional of the size of the input
            FL = coords(0,y_move_length/50*self.sDirection_y,0)
            FR = coords(0,y_move_length/50*self.sDirection_y,0)
            RR = coords(0,y_move_length/50*self.sDirection_y,0)
            RL = coords(0,y_move_length/50*self.sDirection_y,0)
            if self.sDegree_x != 0 or self.sDegree_z != 0:#rotation of the body
                deltas = self.rotateBody(rotation_len/50*self.sDegree_x , rotation_len/50*self.sDegree_z)
                FL.x += deltas[0][0]
                FL.y += deltas[0][1]
                FL.z += deltas[0][2]

                FR.x += deltas[3][0]
                FR.y += deltas[3][1]
                FR.z += deltas[3][2]

                RR.x += deltas[2][0]
                RR.y += deltas[2][1]
                RR.z += deltas[2][2]

                RL.x += deltas[1][0]
                RL.y += deltas[1][1]
                RL.z += deltas[1][2]
                
            self.mLeftFront.PositionWithDelta(FL.x,FL.y,FL.z)
            self.mRightFront.PositionWithDelta(FR.x,FR.y,FR.z)
            self.mRightRear.PositionWithDelta(RR.x,RR.y,RR.z)
            self.mLeftRear.PositionWithDelta(RL.x,RL.y,RL.z)
#PositionWithDelta(self, x,y,z) updateStandPosition(
            self.mLeftFront.updateStandPosition()
            self.mRightFront.updateStandPosition()
            self.mRightRear.updateStandPosition()
            self.mLeftRear.updateStandPosition()

            #päivittää COM pisteen sijainnin kehossa riippuen jalkojen sijainnista
            self.COMUpdateXY()
            
        else:#kävely seis eikä liikuta joten ollaan staattisesti vakio asennossa
            self.GoWalkPosition()
            self.COMUpdateXY()
        if self.mWalkStep > total_step_points:
            self.mWalkStep = 0

    def GoWalkPosition(self):
        #sets legs to preset positions for walking
        
        self.mLeftFront.BackToPosition()
        self.mRightFront.BackToPosition()
        self.mLeftRear.BackToPosition()
        self.mRightRear.BackToPosition()
        self.COMUpdateXY()
        #updaate referense wire frame

#tähän wireframen piirtäminen #  

def draw_frame(thisBody,save,file_name):
    # Extract coordinates for pody frame corners
    LF = [thisBody.wfLeftFront.x,thisBody.wfLeftFront.z,thisBody.wfLeftFront.y]
    LR = [thisBody.wfLeftRear.x,thisBody.wfLeftRear.z,thisBody.wfLeftRear.y]
    RF = [thisBody.wfRightFront.x,thisBody.wfRightFront.z,thisBody.wfRightFront.y]
    RR = [thisBody.wfRightRear.x,thisBody.wfRightRear.z,thisBody.wfRightRear.y]
   
    #points for leg joints
    x_LF,z_LF,y_LF = thisBody.getLegPoints("LF")
    x_LR,z_LR,y_LR = thisBody.getLegPoints("LR")
    x_RF,z_RF,y_RF = thisBody.getLegPoints("RF")
    x_RR,z_RR,y_RR = thisBody.getLegPoints("RR")
   
        #self.mPositionPaw = coords()
        #self.mPositionKnee = coords()
        #self.mPositionShoulder = coords()
# Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

# Plot dots for body frame
    ax.scatter(LF[0], LF[1], LF[2], c='black', marker='o')
    ax.scatter(LR[0], LR[1], LR[2], c='black', marker='o')
    ax.scatter(RF[0], RF[1], RF[2], c='black', marker='o')
    ax.scatter(RR[0], RR[1], RR[2], c='black', marker='o')
    #plot dots for legs
    colors = ['g', 'b', 'y']#paw knee shoulder
    
    for i in range(3):
        ax.scatter(x_RF[i], y_RF[i], z_RF[i], c=colors[i], marker='o')
        ax.scatter(x_RR[i], y_RR[i], z_RR[i], c=colors[i], marker='o')
        ax.scatter(x_LR[i], y_LR[i], z_LR[i], c=colors[i], marker='o')
        ax.scatter(x_LF[i], y_LF[i], z_LF[i], c=colors[i], marker='o')
        #draw lines of legs
    for i in range(2):
        ax.plot([x_RF[i],x_RF[i+1]], [y_RF[i],y_RF[i+1]], [z_RF[i],z_RF[i+1]], c='b')
        ax.plot([x_RR[i],x_RR[i+1]], [y_RR[i],y_RR[i+1]], [z_RR[i],z_RR[i+1]], c='b')
        ax.plot([x_LF[i],x_LF[i+1]], [y_LF[i],y_LF[i+1]], [z_LF[i],z_LF[i+1]], c='b')
        ax.plot([x_LR[i],x_LR[i+1]], [y_LR[i],y_LR[i+1]], [z_LR[i],z_LR[i+1]], c='b')
    #draw lines connecting frame and legs
    ax.plot([x_LR[2],LR[0]], [y_LR[2],LR[1]], [z_LR[2],LR[2]], c='b')
    ax.plot([x_LF[2],LF[0]], [y_LF[2],LF[1]], [z_LF[2],LF[2]], c='b')
    ax.plot([x_RR[2],RR[0]], [y_RR[2],RR[1]], [z_RR[2],RR[2]], c='b')
    ax.plot([x_RF[2],RF[0]], [y_RF[2],RF[1]], [z_RF[2],RF[2]], c='b')
    
# Plot lines for pody frame
    ax.plot([LR[0], LF[0]], [LR[1], LF[1]], [LR[2], LF[2]], c='b')
    ax.plot([RF[0], LF[0]], [RF[1], LF[1]], [RF[2], LF[2]], c='b')
    ax.plot([RF[0], RR[0]], [RF[1], RR[1]], [RF[2], RR[2]], c='b')
    ax.plot([LR[0], RR[0]], [LR[1], RR[1]], [LR[2], RR[2]], c='b')
#tulostetaan tukikolmio
    if len(thisBody.wfSupportTriangle) == 3:
        ax.plot([thisBody.wfSupportTriangle[0][0], thisBody.wfSupportTriangle[1][0]], [thisBody.wfSupportTriangle[0][2], thisBody.wfSupportTriangle[1][2]], [thisBody.wfSupportTriangle[0][1], thisBody.wfSupportTriangle[1][1]], c='g')
        ax.plot([thisBody.wfSupportTriangle[1][0], thisBody.wfSupportTriangle[2][0]], [thisBody.wfSupportTriangle[1][2], thisBody.wfSupportTriangle[2][2]], [thisBody.wfSupportTriangle[1][1], thisBody.wfSupportTriangle[2][1]], c='g')
        ax.plot([thisBody.wfSupportTriangle[2][0], thisBody.wfSupportTriangle[0][0]], [thisBody.wfSupportTriangle[2][2], thisBody.wfSupportTriangle[0][2]], [thisBody.wfSupportTriangle[2][1], thisBody.wfSupportTriangle[0][1]], c='g')

    ax.scatter(thisBody.wfCOM.x, thisBody.wfCOM.z, thisBody.wfCOM.y, c='r', marker='o')

    #coordinate systems
    #center of body
    ax.plot([0,30],[0,0],[0,0],c='g')
    ax.plot([0,0],[0,30],[0,0],c='r')
    ax.plot([0,0],[0,0],[0,30],c='b')
    #legs
    ax.plot([141,30+141],[109,109],[0,0],c='g')
    ax.plot([141,141],[109,30+109],[0,0],c='r')
    ax.plot([141,141],[109,109],[0,30],c='b')

    ax.plot([141,30+141],[-109,-109],[0,0],c='g')
    ax.plot([141,141],[-109,30-109],[0,0],c='r')
    ax.plot([141,141],[-109,-109],[0,30],c='b')

    ax.plot([-141,30-141],[-109,-109],[0,0],c='g')
    ax.plot([-141,-141],[-109,30-109],[0,0],c='r')
    ax.plot([-141,-141],[-109,-109],[0,30],c='b')

    ax.plot([-141,30-141],[109,109],[0,0],c='g')
    ax.plot([-141,-141],[109,30+109],[0,0],c='r')
    ax.plot([-141,-141],[109,109],[0,30],c='b')
    ax.set_xlabel('X')
    ax.set_ylabel('Z')
    ax.set_zlabel('Y')
    ax.set_aspect('equal')
    ax.invert_zaxis()
    
    if save:
        plt.savefig(file_name, format="png", dpi=1200)
    else:
        plt.show()

def main():
    #initialise everything like in the righ one
    body = MyBody()
    body.mRightFront.mTibia.Attach("SERVO_35KG",220,0,90)#Attach(self, srv_type, home, correction, zero) SERVO_35KG SERVO_20KG
    body.mRightFront.mFemur.Attach("SERVO_20KG", 100.0, 0.0,175.0)
    body.mRightFront.mCoax.Attach("SERVO_20KG", 135.0, 0.0,135.0)
 

    body.mRightRear.mTibia.Attach("SERVO_35KG",220,0,90)
    body.mRightRear.mFemur.Attach("SERVO_20KG", 100.0, 0.0,175.0)
    body.mRightRear.mCoax.Attach("SERVO_20KG", 135.0, 0.0,135.0)
  
  
    body.mLeftFront.mTibia.Attach("SERVO_35KG",220,0,90)
    body.mLeftFront.mFemur.Attach("SERVO_20KG", 100.0, 0.0,175.0)
    body.mLeftFront.mCoax.Attach("SERVO_20KG", 135.0, 0.0,135.0)
  
  
    body.mLeftRear.mTibia.Attach("SERVO_35KG",220,0,90)
    body.mLeftRear.mFemur.Attach("SERVO_20KG", 100.0, 0.0,175.0)
    body.mLeftRear.mCoax.Attach("SERVO_20KG", 135.0, 0.0,135.0)
 
    body.mLeftFront.mTibia.SetLimits(80.0, 270.0)
    body.mLeftRear.mTibia.SetLimits(80.0, 270.0) 
    body.mRightFront.mTibia.SetLimits(80.0, 270.0)
    body.mRightRear.mTibia.SetLimits(80.0, 270.0)
  
    body.mLeftFront.mFemur.SetLimits(0.0, 270.0)
    body.mRightFront.mFemur.SetLimits(0.0, 270.0)
    body.mLeftRear.mFemur.SetLimits(0.0, 270.0)
    body.mRightRear.mFemur.SetLimits(0.0, 270.0)

    body.mLeftFront.mCoax.SetLimits(50.0, 170.0)
    body.mRightFront.mCoax.SetLimits(50.0, 170.0)
    body.mLeftRear.mCoax.SetLimits(50.0, 170.0)
    body.mRightRear.mCoax.SetLimits(50.0, 170.0)

    body.Home()
   
    
    body.GoWalkPosition()
    
    body.setGait("JACK_SPARROW")
    body.WalkDirection( 0.2, 0, 0, 0, 0, 0, 0, 0.16)
    fig = plt.figure()
    
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Z')
    ax.set_zlabel('Y')
    ax.set_aspect('equal')
    ax.invert_zaxis()
    bootCounter = 0

    def update(frame):
        
        #body.WalkDirection( 0.2, 0, 0, 0, 1, 0, 1, 0.16)
        ax.clear()
        result = []
    # Extract coordinates for pody frame corners
        LF = [body.wfLeftFront.x,body.wfLeftFront.z,body.wfLeftFront.y]
        LR = [body.wfLeftRear.x,body.wfLeftRear.z,body.wfLeftRear.y]
        RF = [body.wfRightFront.x,body.wfRightFront.z,body.wfRightFront.y]
        RR = [body.wfRightRear.x,body.wfRightRear.z,body.wfRightRear.y]
   
    #points for leg joints
        x_LF,z_LF,y_LF = body.getLegPoints("LF")
        x_LR,z_LR,y_LR = body.getLegPoints("LR")
        x_RF,z_RF,y_RF = body.getLegPoints("RF")
        x_RR,z_RR,y_RR = body.getLegPoints("RR")
   
        #self.mPositionPaw = coords()
        #self.mPositionKnee = coords()
        #self.mPositionShoulder = coords()


# Plot dots for body frame
        
        result.append(ax.scatter(LF[0], LF[1], LF[2], c='r', marker='o'))
        result.append(ax.scatter(LR[0], LR[1], LR[2], c='r', marker='o'))
        result.append(ax.scatter(RF[0], RF[1], RF[2], c='r', marker='o'))
        result.append(ax.scatter(RR[0], RR[1], RR[2], c='r', marker='o'))
        result.append(ax.scatter(body.wfCOM.x, body.wfCOM.z, body.wfCOM.y+170, c='r', marker='o'))
    #plot dots for legs
        colors = ['g', 'b', 'y']#paw knee shoulder
    
        for i in range(3):
            result.append(ax.scatter(x_RF[i], y_RF[i], z_RF[i], c=colors[i], marker='o'))
            result.append(ax.scatter(x_RR[i], y_RR[i], z_RR[i], c=colors[i], marker='o'))
            result.append(ax.scatter(x_LR[i], y_LR[i], z_LR[i], c=colors[i], marker='o'))
            result.append(ax.scatter(x_LF[i], y_LF[i], z_LF[i], c=colors[i], marker='o'))
        #draw lines of legs
        for i in range(2):
            result.append(ax.plot([x_RF[i],x_RF[i+1]], [y_RF[i],y_RF[i+1]], [z_RF[i],z_RF[i+1]], c='b'))
            result.append(ax.plot([x_RR[i],x_RR[i+1]], [y_RR[i],y_RR[i+1]], [z_RR[i],z_RR[i+1]], c='b'))
            result.append(ax.plot([x_LF[i],x_LF[i+1]], [y_LF[i],y_LF[i+1]], [z_LF[i],z_LF[i+1]], c='b'))
            result.append(ax.plot([x_LR[i],x_LR[i+1]], [y_LR[i],y_LR[i+1]], [z_LR[i],z_LR[i+1]], c='b'))
    #draw lines connecting frame and legs
        result.append(ax.plot([x_LR[2],LR[0]], [y_LR[2],LR[1]], [z_LR[2],LR[2]], c='b'))
        result.append(ax.plot([x_LF[2],LF[0]], [y_LF[2],LF[1]], [z_LF[2],LF[2]], c='b'))
        result.append(ax.plot([x_RR[2],RR[0]], [y_RR[2],RR[1]], [z_RR[2],RR[2]], c='b'))
        result.append(ax.plot([x_RF[2],RF[0]], [y_RF[2],RF[1]], [z_RF[2],RF[2]], c='b'))
    
# Plot lines for pody frame
        result.append(ax.plot([LR[0], LF[0]], [LR[1], LF[1]], [LR[2], LF[2]], c='b'))
        result.append(ax.plot([RF[0], LF[0]], [RF[1], LF[1]], [RF[2], LF[2]], c='b'))
        result.append(ax.plot([RF[0], RR[0]], [RF[1], RR[1]], [RF[2], RR[2]], c='b'))
        result.append(ax.plot([LR[0], RR[0]], [LR[1], RR[1]], [LR[2], RR[2]], c='b'))
#tulostetaan tukikolmio
        if len(body.wfSupportTriangle) == 3:
            result.append(ax.plot([body.wfSupportTriangle[0][0], body.wfSupportTriangle[1][0]], [body.wfSupportTriangle[0][2], body.wfSupportTriangle[1][2]], [body.wfSupportTriangle[0][1], body.wfSupportTriangle[1][1]], c='g'))
            result.append(ax.plot([body.wfSupportTriangle[1][0], body.wfSupportTriangle[2][0]], [body.wfSupportTriangle[1][2], body.wfSupportTriangle[2][2]], [body.wfSupportTriangle[1][1], body.wfSupportTriangle[2][1]], c='g'))
            result.append(ax.plot([body.wfSupportTriangle[2][0], body.wfSupportTriangle[0][0]], [body.wfSupportTriangle[2][2], body.wfSupportTriangle[0][2]], [body.wfSupportTriangle[2][1], body.wfSupportTriangle[0][1]], c='g'))
        
        if frame > 3:
            
            body.PositionUpdate()
        
        
        result.extend(ax.get_children())
    
        ax.set_xlim([-300, 300])
        ax.set_ylim([-300, 300])
        ax.set_zlim([-300, 300])
        

        return result
    
    
    ani = FuncAnimation(fig, update, frames=30)

    # Define the initial values for your sliders
    initial_values1 = [0.0, 0.0, 0.0]
    initial_values2 = [0.0, 0.0, 0.0]
    slider_titles1 = ["x rot","y rot","z rot"]
    slider_titles2 = ["x move","y move","z move"]

# Create a list to hold the Slider widgets
    sliders = []
    reset_buttons = {}

# Create and add six sliders to the figure
    for i in range(3):
        ax_slider = plt.axes([0.1, 0.1 - 0.05 * i, 0.22, 0.03])
        slider = Slider(ax_slider, slider_titles1[i], -1.0, 1.0, valinit=initial_values1[i])
        sliders.append(slider)
        #käännökselle
        ax_reset_button = plt.axes([0.35, 0.1 - 0.05 * i, 0.1, 0.03])
        reset_button = Button(ax_reset_button, 'Reset')
        reset_buttons[reset_button] = slider
        ax_slider = plt.axes([0.55, 0.1 - 0.05 * i, 0.22, 0.03])
        slider = Slider(ax_slider, slider_titles2[i], -1.0, 1.0, valinit=initial_values2[i])
        sliders.append(slider)
        
        #liikkeelle
        ax_reset_button = plt.axes([0.80, 0.1 - 0.05 * i, 0.1, 0.03])
        reset_button = Button(ax_reset_button, 'Reset')
        reset_buttons[reset_button] = slider

# Define a function to be called when the sliders are updated
    def update(val):
    # You can access the current slider values using slider.val
        slider_values = [slider.val for slider in sliders]#rot move järjestyksessä
    # Update your program with the new slider values
        #print(slider_values)
        body.WalkDirection( 0.2, slider_values[0], slider_values[2], slider_values[4], slider_values[1], slider_values[3], slider_values[5], 0.16)
    
    def reset_slider(event, slider):
        slider.set_val(0)
# Connect the update function to the slider events
    for slider in sliders:
        slider.on_changed(update)
    for reset_button, slider in zip(reset_buttons, sliders):
        reset_button.on_clicked(lambda event, slider=slider: reset_slider(event, slider))

    
    plt.show()
    #draw_frame(body)
    
def oneframe():
    body = MyBody()
    body.mRightFront.mTibia.Attach("SERVO_35KG",220,0,90)#Attach(self, srv_type, home, correction, zero) SERVO_35KG SERVO_20KG
    body.mRightFront.mFemur.Attach("SERVO_20KG", 100.0, 0.0,175.0)
    body.mRightFront.mCoax.Attach("SERVO_20KG", 135.0, 0.0,135.0)
 

    body.mRightRear.mTibia.Attach("SERVO_35KG",220,0,90)
    body.mRightRear.mFemur.Attach("SERVO_20KG", 100.0, 0.0,175.0)
    body.mRightRear.mCoax.Attach("SERVO_20KG", 135.0, 0.0,135.0)
  
  
    body.mLeftFront.mTibia.Attach("SERVO_35KG",220,0,90)
    body.mLeftFront.mFemur.Attach("SERVO_20KG", 100.0, 0.0,175.0)
    body.mLeftFront.mCoax.Attach("SERVO_20KG", 135.0, 0.0,135.0)
  
  
    body.mLeftRear.mTibia.Attach("SERVO_35KG",220,0,90)
    body.mLeftRear.mFemur.Attach("SERVO_20KG", 100.0, 0.0,175.0)
    body.mLeftRear.mCoax.Attach("SERVO_20KG", 135.0, 0.0,135.0)
 
    body.mLeftFront.mTibia.SetLimits(80.0, 270.0)
    body.mLeftRear.mTibia.SetLimits(80.0, 270.0) 
    body.mRightFront.mTibia.SetLimits(80.0, 270.0)
    body.mRightRear.mTibia.SetLimits(80.0, 270.0)
  
    body.mLeftFront.mFemur.SetLimits(0.0, 270.0)
    body.mRightFront.mFemur.SetLimits(0.0, 270.0)
    body.mLeftRear.mFemur.SetLimits(0.0, 270.0)
    body.mRightRear.mFemur.SetLimits(0.0, 270.0)

    body.mLeftFront.mCoax.SetLimits(50.0, 170.0)
    body.mRightFront.mCoax.SetLimits(50.0, 170.0)
    body.mLeftRear.mCoax.SetLimits(50.0, 170.0)
    body.mRightRear.mCoax.SetLimits(50.0, 170.0)

    body.Home()   
    body.GoWalkPosition()
    body.setGait("GAIT_WALK")
    body.WalkDirection( 0.2, 0, 0, 0, 1, 0, 0, 0.16)
    body.PositionUpdate()
    draw_frame(body,False,"walk_cycle.png")
    
#oneframe()   
    

main()
