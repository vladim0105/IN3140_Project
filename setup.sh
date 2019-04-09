echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
cd src
git clone https://github.uio.no/INF3480/crustcrawler_simulation.git
git clone https://github.uio.no/INF3480/crustcrawler_pen.git
cd ..
catkin_make
source devel/setup.bash
