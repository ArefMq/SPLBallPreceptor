# SPLBallPreceptor
White Ball (official SPL Ball for RoboCup 2016) perceptor module. This module is developed as a patch for B-Human framework and has been tested in the 2013 release. The code has been used by MRL Biped Lab. (MRL-SPL Team) in RoboCup 2016 standard platform league (SPL).

To use this code you need to setup B-Human code release 2013. The later versions might also be working. The framework is accessible on:
http://b-human.de/

After installation of B-Human's code, replace the provided files with the original ones. This is all needs to done, unless there were modifications on original representations.

Since the change in the SPL rule about the ball, an entirely approach needed for detecting the ball. Because the ball is no longer has an unique color. The approach represented in this release is finding circles in the image using Random Hough Transform (RHT), afterward filter them by trying to detect the black pattern on the ball. However this code is still under development and all feature might not be applicable right now.

Feel free to use, modify or re-publish this code. And please feel free to fork the code from Github and send pull requests.

Report any comment or bugs to:
a.moqadam@mrl-spl.ir

Regards,
Aref Moqadam
Biped Lab. (MRL-SPL Team)
Mechatronics Research Laboratories (MRL)
