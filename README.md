# TFC-NXP-Lebanon

A robotics competition involving autonomous vehicules.
Please check the documentation.pdf for further information about the challenges and the design decisions. 

The code has a commented part in it that was used according to the challenge in question. You will also find some commented parts that refer to other tuning parameters for the PID. these are experiments carried out by the team. 

## Folders description

Under the directory **Final** you can find the code that was used on the car while racing.
Under the directory **camera_test** you can find the code used on the car while testing the performance of the camera and tuning it according to the luminosity of the room (as this tuning couldn't be automised and requiered turning a special screw/potientometer in the camera)
Under the directory **sketch_camera_over_bluetooth** you can find the code that was used to recieve the sent images from the vehicule (via bluetooth) and display them. The program is written using the [Processing](https://processing.org/) environment.

## Quick explanation
The robot used was a standard [Alamak car](https://nxp.gitbook.io/nxp-cup/developer-guide/landzo-car-model/kit-contents/model-alamak), with a few modifications. Most notably: 
* Used 2 cameras instead of a single one (one for far & another for near view) (check the documentation)
* Infrared rotation counter usede for the PID controlled motors (check the documentation)

### 2 Cameras
The motivation for using 2 cameras was:
1. The existance of intersections, hence the existance of a blind spot while crossing the cross-road: the solution is to rely on the camera that is detecting a border
2. The need to detect signs on the road 
  * Stop at the end of the lap for the normal timed race
  * pedestrian walking lines detection for the road sign detection challenge

This technique proved helpful and the detection of road signs was highly reliable.

### PID on the motors
The motivation for using PID controlled Motors:
1. In case no cameras detected any road lines, we had the possibility to slow the down the car to a well controlled speed and not just cut of the power.
  * This is helpful in case the car got off the lap, so slowing down the car preveneted any damage on the Servo motor controlling the steering wheels.
  * While crossing a road, there might still be a gap in the detection of any lines, and slowing down the car will help avoid any problems related to stability of the PID (the PID was much more stable while driving slowly)
2. Having a PID controlled speed of the car meant that the PID tuning of the steering was much more simple and the equations can assumptions can be simplified.


  * Stop at the end of the lap for the normal timed race
  * pedestrian walking lines detection for the road sign detection challenge
