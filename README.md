# robot-dog
Code and the documentation pdf contain spelling errors.<br>
Comments in code are partly in english and partly in finnish.<br>

Other than that the simulator works as its supposed to.<br>


Folder muokatut contains some 3D models that I have modified. <br>
-NOVA SM3 Assembly_SM3_Frame_RightTibia muokattu <-- modified tibia for the robot, attachment point for  the servo is changed to better fit the servomotors used.<br>
-cover_updated_for_Chris_Shoulder_muokattu v4 <-- modified the bearing holes to fit the bearings I used.<br>
-SM3_Frame_FrontShoulderOuter paksunnettu <-- part is made thicker and bearing holes deeper to work with the modified frame.<br>
-SM3_Frame_RearShoulderMiddle Paksunnettu levy <-- same as above.<br>
-robottikoiran jetson runko v4 <-- modified main frame part for the robot. It is made to have room for jetson nano and the components in the original design.<br>
-Assembly_Cover_HeadPanel kamera upotus v8 <-- modified the head part so that it has mounting for IMX219-83 Stereo Camera.<br>
-Ass - Speaker v5 v1 <-- is modified so that it fits to the new shoulder frame parts.<br>
-tassumuotti <- mold for casting the paws from silicone

Original design can be found from here:<br>
https://github.com/cguweb-com/Arduino-Projects/tree/main/Nova-SM3<br>

Code is python version of the code that is eventually going to be in the robot. <br>

Python version works as a simulator for the behavior, so it would be easyer to implement new code with out breaking anything.<br>

dynamic_balancing_v0.1 contains the teensy 4.0 code that has all the basic functionality. It can communicate with the remote and move accordingly. The code does dynamic balancing while walking but the parameters of that are in a need of finetuning at the time this code was uploaded. <br>

To set up the code in your system follow these steps. <br>
Comment everything off from the void loop. Start by finding the zero position just by adjusting values in these lines <br>
body.mRightFront.mTibia.Attach(15, SERVO_35KG, 220.0, 10.0,89.3). Zero position is a position where coaxes are aligned and in 90 degree angle relative to the body and the axis of tibia and femur align in a straigh line going down from the body toward the ground. Basicly in zero position the robot stand with all its legs fully extended. Find this position by changing the first degree value in Attach commands. Then after the correct degrees are found its time to move those values in to the last position in the attach command. Then the first values are used to find good boot up position for the robot. <br>
The syntax of the attach goes like this body.mRightFront.mTibia.Attach(Pin id of this joint in the motor driver, Type of motor used, Home position in degrees, Home position offset,Zero position in degrees)<br>
After the zero and home is set up you can run the code inside the void loop. To make the robot move correctly you need to finetune the zero positions by putting the robot in to a walking position and compare the values in myconfig to the robots current position. Fine tune the degrees until the robot matches to the myconfig values as closely as possible, start from coax joints, continue with femurs and finally tibia. Tibia can be used to eaven out the weight distribution of the robot. <br>
To make the robot resbond to the remote you need to have the same remote or set up your own. The remote im using can be found here <br>
https://github.com/cguweb-com/Arduino-Projects/tree/cb5a4712f65acdd1623183609f027163f564fcbd/Nova-SM3/Remote%20Control <br>
My remote code v0.1 matches this teensy code so it should work with little effort. <br>

dynamic_balancing_v0.2 works with remote adjusted balancing parameters. Remote code 0.2 matches this version.
