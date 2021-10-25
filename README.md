# RosUltrasonic11V10_V2

<h1>Requirements :</h1>

numpy==1.13.3 :
$pip install numpy==1.13.3

hidapi==0.10.1 :
$pip install hidapi==0.10.1

or directly 

pip install -r catkin_sinusoidal_rosiocard/src/sinusoidal/config/requirements.txt

<h2>Create udev Rules :</h2>

1 - At the root folder

2 - sudo nano /etc/udev/rules.d/99-rosiocard.rules 

3 - copy and paste udev rules : 
     
	   SUBSYSTEM=="input", GROUP="input", MODE="0666"

     SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5743", MODE="666", GROUP="plugdev"

     KERNEL=="hidraw*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5743", MODE:="0666", GROUP="plugdev"

4- ctrl+x and Y for save the udev rules.

5- sudo udevadm trigger or Reboot

<h1>Instalattion :</h1>

1 - git clone https://github.com/muratkayamurat/RosUltrasonic11V10_V2

2- cd catkin_sinusoidal_rosiocard

3- catkin_make

<h1>Run :</h1>

1- cd catkin_sinusoidal_rosiocard

2- source devel/setup.bash

3- roslaunch sinusoidal startsinusoidalrosiocard.launch
