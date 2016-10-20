# baxter_motion_planning
Motion planning prototype setup for Baxter robot using OMPL python bindings.


# Install Rospy
- add it to your shell path


# Run Merry:
- In all terminals with baxter nodes running:
- cd ~/projects/ros_ws/
- ./baxter.sh

- chmod +x start_planning.sh
- ./start_planning.sh

# add startup command to .bashrc (start_planning)
- nano ~/.bashrc
- add: alias start_planning='~/projects/ros_ws/src/potential_fields/start_planning.sh'


- For gazebo simulator: roslaunch cwru_baxter_sim baxter_world.launch
- roslaunch baxter_moveit_config demo_baxter.launch
- For motion planning: roslaunch baxter_moveit_config move_group.launch
- For visualization: rviz rviz

# Extract raw data from Kinect
- http://answers.ros.org/question/9803/extracting-raw-data-from-kinect/

now, in python you can:
from roslib import message
import sensor_msgs.point_cloud2 as pc2
import freenect
import pcl


# Install Django and Postgres (unnecessary)
- Install django: pip install Django==1.9.7
- pip install psycopg2 colour

- Use this link to setup postgres sql:
    https://www.digitalocean.com/community/tutorials/how-to-use-postgresql-with-your-django-application-on-ubuntu-14-04

    Use these commands to create the Postgres database:
    -    sudo apt-get update
    -    sudo apt-get install python-pip python-dev libpq-dev postgresql postgresql-contrib
    -    sudo su - postgres
    -    psql
    ``CREATE DATABASE merry;
    CREATE USER lab WITH PASSWORD 'merry';
    ALTER ROLE lab SET client_encoding TO 'utf8';
    ALTER ROLE lab SET default_transaction_isolation TO 'read committed';
    ALTER ROLE lab SET timezone TO 'UTC';
    GRANT ALL PRIVILEGES ON DATABASE merry TO lab;
    \q``
    -    exit
        
        
# Make migrations (first time and subsequently only if django server startup says it's necessary):

- cd ~/projects/ros_ws/potential_fields/merry_app/
- python manage.py makemigrations merry
- python manage.py migrate merry

# Create superuser:
- python manage.py createsuperuser
- username: lab
- pass: merryP@$$

# Run django server:
- python manage.py runserver 0.0.0.0:8000
- http://localhost:8000/admin
- login with superuser username and password




# install Kinect Libraries
- sudo apt-get install python-dev python-numpy
- opencv: https://help.ubuntu.com/community/OpenCV
- cd ~/libraries/
- sudo apt-get install python-freenect
- if above doesn't work, these are supposed to work but usually throw errors:
- -- git clone https://github.com/OpenKinect/libfreenect.git
- -- cd libfreenect/wrappers/python
- -- python setup.py build_ext --inplace


# install Cython, PCL for Python (doesn't seem to work due to improper installation)
- sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
- sudo apt-get update
- sudo apt-get install python-dev build-essential libpcl-all
- Download latest release from http://cython.org and untar it to ~/libraries
- cd ~/libraries/Cython-0.xx
- sudo python setup.py install
- cd ~/libraries/
- git clone https://github.com/strawlab/python-pcl.git
- cd python-pcl
- sudo python setup.py clean
- sudo make clean
- sudo make all
- sudo python setup.py install (make sure compiler for make and install are same!!!)


# Install kdl libraries
- cd ~/projects
- git clone https://github.com/ShaunHoward/baxter_pykdl.git
- git clone https://github.com/gt-ros-pkg/hrl-kdl.git 
- cd baxter_pykdl; python setup.py install
- cd ../hrl-kdl; python setup.py install

# Latest OMPL installation (via http://ompl.kavrakilab.org/installation.html)
Locate the OMPL install script in the project root and make the script executable:

chmod u+x install-ompl-ubuntu.sh

Next, there are three ways to run this script:

    ./install-ompl-ubuntu.sh will install OMPL without Python bindings
    ./install-ompl-ubuntu.sh --python will install OMPL with Python bindings
    ./install-ompl-ubuntu.sh --app will install OMPL.app with Python bindings

The script downloads and installs OMPL and all dependencies via apt-get & pip and from source. It will ask for your password to install things. The script has been tested on vanilla installs of Ubuntu 14.04 (Trusty), 15.10 (Wily), and 16.04 (Xenial). 

MAKE SURE to sudo apt-get install g++-5. 
ONLY OMPL.app requires ccd libraries and jade.

# install MoveIt! (not required)
- sudo add-apt-repository ppa:libccd-debs/ppa
- sudo apt-get update
- sudo apt-get install libccd-dev
    cd ~
    mkdir -p github.com/danfis
    cd github.com/danfis
    git clone https://github.com/danfis/libccd
    cd libccd
    #ignore their instructions
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
- sudo apt-get install ros-indigo-moveit-full
- cd ~/ros_ws/src
- git clone https://github.com/ros-planning/moveit_robots.git
- clone omplapp and ompl:
- sudo apt-get install build-essential libboost-all-dev cmake libccd-dev python-dev python-pyqt5.qtopengl python-opengl freeglut3-dev libassimp-dev libeigen3-dev libode-dev doxygen graphviz
- cd omplapp
- mkdir -p build/Release
- cd build/Release
- cmake -DCCD_LIBRARIES=/usr/local/lib/x86_64-linux-gnu/libccd.a -DCCD_INCLUDE_DIRS=/usr/local/include .
- Fix any errors or warnings after make (make sure ompl is built correctly)
- make -j 4 update_bindings
Optionally, generate the Python bindings with make -j 4 update_bindings.
Compile OMPL.app by typing make -j 4.
Optionally, run the test programs by typing make test.
Optionally, generate documentation by typing make doc.
If you need to install the library, you can type sudo make install.
The install location is specified by CMAKE_INSTALL_PREFIX.
If you install in a non-standard location, you have to set the environment variable PYTHONPATH to the
directory where the OMPL python module is installed (e.g., $HOME/lib/python2.7/site-packages).
(from http://ompl.kavrakilab.org/installation.html)

# From UNH AI Wiki (not necessary):
http://unh-ai.pbworks.com/w/page/105303567/OMPL
(if you're using ROS and want OMPL, see MoveIt!)

OMPL has become slightly trickier to install recently and has some outdated installation documentation, especially in terms of its dependencies and how to properly install them.

Assuming you're using Ubuntu and you're fine to use your home dir as your working directory...

Dependencies first:

----the easy stuff----
sudo apt-get install libboost-all-dev cmake python-dev python-qt4-dev python-qt4-gl python-opengl freeglut3-dev libassimp-dev libeigen3-dev libode-dev doxygen graphviz libqt4-dev python-pygccxmlsudo pip install python-utils

----libccd----
cd ~
mkdir -p github.com/danfis
cd github.com/danfis
git clone https://github.com/danfis/libccd
cd libccd
#ignore their instructions
mkdir build
cd build
cmake ..
make
sudo make install

----octomap----
cd ~
mkdir -p github.com/OctoMap
cd github.com/OctoMap
git clone https://github.com/OctoMap/octomap
cd octomap
mkdir build
cd build
cmake ..
make
sudo make install

----flann----
cd ~
mkdir -p github.com/mariusmuja
cd github.com/mariusmuja
git clone https://github.com/mariusmuja/flann
cd flann
mkdir build
cd build
cmake ..
make
sudo make install

----fcl----
cd ~
mkdir -p github.com/flexible-collision-library
cd github.com/flexible-collision-library
git clone https://github.com/flexible-collision-library/fcl
cd fcl
mkdir build
cd build
cmake ..
make
#if during the previous step you run into an issue where ld complains about not having compiled libccd with -fPIC
#you can delete /usr/local/lib/libccd.a -- and then it'll find the correct libccd.so file instead
sudo make install

----pygccxml----
cd ~
mkdir -p github.com/gccxml
cd github.com/gccxml
git clone https://github.com/gccxml/pygccxml
cd pygccxml
sudo python setup.py install

----pyplusplus----
cd ~
mkdir -p bitbucket.org/ompl
cd bitbucket.org/ompl
hg clone https://bitbucket.org/ompl/pyplusplus
cd pyplusplus
sudo python setup.py install

----ompl----
cd ~
mkdir ompl
cd ompl
git clone https://github.com/ompl/omplapp.git
cd omplapp
git clone https://github.com/ompl/ompl.git
mkdir -p build/Release
cd build/Release
cmake ../..
make update_bindings
make
sudo make install




