# ntrip_ros
Connects to a ntrip caster and publishes rtcm messages

Changes from the original repo: github.com/tilk/ntrip_ros
* Ported to Python 3.10 - Humble
* Patched httpresponse to allow connecting to u-center ntrip caster

Requires rtcm-msgs package: https://github.com/tilk/rtcm_msgs

I'm using this with https://github.com/KumarRobotics/ublox which recently has had rtcm message support added. Some system repos (robostack and ubuntu) don't have the latest version so you may need to build from source in your catkin workspace.

You can generate the require $GPGGA message at this site. https://www.nmeagen.org/ Set a point near where you want to run and click "Generate NMEA file". Cut and paste the $GPGGA message into the launch file.

