# Ubuntu-mate raspberry4

## rpi-update && raspi-config install

[https://gist.github.com/jgamblin/2441964a1266764ed71f3243f87bbeec](https://gist.github.com/jgamblin/2441964a1266764ed71f3243f87bbeec)

| sudo apt-get update                                                                                 |
| --------------------------------------------------------------------------------------------------- |
| sudo apt-get install lua5.1 alsa-utils triggerhappy curl libcurl3                                   |
| wget http://archive.raspberrypi.org/debian/pool/main/r/raspi-config/raspi-config\_20160322\_all.deb |
| wget http://archive.raspberrypi.org/debian/pool/main/r/rpi-update/rpi-update\_20140705\_all.deb     |
| dpkg -i raspi-config\_20160322\_all.deb                                                             |
| dpkg -i rpi-update\_20140705\_all.deb                                                               |
