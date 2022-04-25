-----25-04-2022-----
First commit
The files contain are a typical ROS publisher and Subscriber architecture.
The only files that are planed to be modified are in the nodes folder and they are called
the subscriber.py and the publisher.py
The files are to be found inside the src\Nodes\...

Important Comments (From New to Old)
*Some files were moved out of the correct structure (from SRC/Nodes to directly SRC/), 
this is to avoid problems with git, however when downloading and updating to the arduino
we need to copy ONLY SUBSCRIBERY.py AND PUBLISHER.PY
*If a code need to be updated inside the Raspberry, then we need to copy this files from the repo and
place them directly in the same folder inside the raspberry pi.
*The code is planed to run inside the Raspberry PI architecture. So actually no compilation is avaialble
from this repository, moreover, some libraries only work inside the Raspberrypi.



