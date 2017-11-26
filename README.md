# AIDE
AIDE is short for Artificial Intelligence Development Environment and is a project that facilitates the creation of AI Applications by providing an event based architecture and semantic UI tools. 

This repository is under heavy development. Deployable version is coming soon.
## Installation
Assuming you have ubuntu 16.04, install following:

```bash
sudo apt-get update
sudo apt-get install mongodb docker docker-compose default-jre python2-pip
```
then add yourself to the docker group
```
sudo usermod -aG docker $USER
```
and relog.

then, install [ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu) and setup a [catkin workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

then, clone this project and the message converter into your workspace's `src` folder (assuming you are in your catkin workspace):
```bash
cd src
git clone git@github.com:Lambda-3/aide.git
git clone git@github.com:vschlegel/rospy_message_converter.git
```
then
```bash
cd src/aide/aide_core
sudo pip install -r requirements.txt
```
to install the required python libs.
Now, build your workspace
```bash
#back to your catkin workspace
cd ../../../
catkin_make
```

Obtain [indra](https://github.com/Lambda-3/IndraComposed), [aide-java-packaged](https://github.com/vschlegel/aide-java-packaged), and [aide-frontend](https://github.com/vschlegel/aide-frontend) and follow their setup instructions. These projects don't have to be cloned into your catkin workspace.
For Indra, the `w2v-en-wiki-2014` model is used, change accordingly in `aide/aide_core/src/aide_core/apis/indra.py` if you want to use another one.

`location` and `weather` apis are disabled, if you have keys for those apis (to be added in `credentials.py`), move them from `*.py.bak` to `.py`. same goes for the whatsapp helper script, you will need to obtain the [yowsup](https://github.com/tgalal/yowsup/) project for that (`sudo pip install yowsup2`).

Now, to run AIDE, start the required components:
```bash
roscore
```
In your `aide-frontend` directory:
```bash
node_modules/.bin/grunt serve
```
In your `aide-java-packaged` directory:
```
cd csparql/bin
./cep.sh
```
Then start the ROS nodes:
```bash
rosrun aide_core rest.py
rosrun aide_core extractor_handler.py
rosrun aide_core action_handler.py
rosrun aide_core api_handler.py
rosrun aide_core event_handler.py
```
Start some helper scripts
```bash
rosrun aide_helpers smart_building.py
rosrun aide_helpers input_reader.py
rosrun aide_helpers ac_control.py
```
to have some data to work with.
