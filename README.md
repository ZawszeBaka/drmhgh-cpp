Team1
ws://127.0.0.1:9090

roslaunch lane_detect lane_detect.launch

Install OpenCV to virtualenv # recommended

pkg-config --modversion opencv // check opencv version
dpkg --get-selections | grep opencv // installed packages
dpkg -l | grep opencv // list all the packages
dpkg -L | grep opencv // list all installed packages

# ROS_Package_example
## I. Installation
1. Ubuntu 16.04 or newer
2. One of these following version of [ROS](https://ros.org)

    - [Melodic Morenia](http://wiki.ros.org/melodic)
    - It is recommended to install the full version
      ```
      $ sudo apt-get install ros-<distro>-desktop-full
      ```

3. Create catkin workspace
    ```
    $ mkdir -p ~/catkin_ws/src
    $ cd ~/catkin_ws/
    $ catkin_make
    $ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    $ source ~/.bashrc
    $ #or
    $ source devel/setup.bash
    ```

4. Install rosbridge-suite
    ```
    $ sudo apt-get install ros-<distro>-rosbridge-server
    ```
5. [New version unity](https://drive.google.com/drive/u/0/folders/1ShsdXU_2Dk86wIaTQb9mbJNWgT3kEy0_?hl=en)






=====================================================================
Update Code :https://drive.google.com/drive/folders/1sqVxYwPfHmCDlFcriqGFqOyqtwIWCU62?usp=sharing- Retrieved Team1_image
