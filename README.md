# SPLBallPerceptor
White Ball (official SPL Ball for RoboCup 2016) perceptor module. This module is developed as a patch for B-Human framework and has been tested in the 2013 release. The code has been used by MRL Biped Lab. (MRL-SPL Team) in RoboCup 2016 standard platform league (SPL).

To use this code you need to setup B-Human code release 2013. The later versions might also be working. The framework is accessible on:
http://b-human.de/

After installation of B-Human's code, replace the provided files with the original ones. That is all need to be done, unless there were modifications on original representations. Please note that, both upper and lower camera image has to be at same resulotion. So, please check the "upperCameraInfo" and "lowerCameraInfo" file at config to ensure they are running at same resulotion. One other thing to note, is to please set black color into orange label in B-Human's color lookup table.

Since the change in the SPL rule about the ball, an entirely approach needed for detecting the ball. Because the ball is no longer has an unique color. The approach represented in this release is finding circles in the image using Fast Random Hough Transform (FRHT), afterward filter them by trying to detect the black pattern on the ball. However this code is still under development and all feature might not be applicable right now.

Feel free to use, modify or re-publish this code. And please feel free to fork the code from Github and send pull requests. For more information you can visit my blog at http://arefmq.blogspot.com/ or mail me personally.

Report any comment or bugs to:
a.moqadam@mrl-spl.ir

Regards,
Aref Moqadam
Biped Lab. (MRL-SPL Team)
Mechatronics Research Laboratories (MRL)
