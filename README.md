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

