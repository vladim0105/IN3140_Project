Installation:

    git clone https://github.com/vladim0105/IN3140_Project.git
 Inside the cloned directory:
 

    sudo ./setup.sh
Simulation:

    source devel/setup.bash
    roslaunch crustcrawler_pen_gazebo controller.launch control:=trajectory
    rosrun in3140 path_planner.py (arguments here)
Arguments:

    --image/-img //Relative path to image from where you are
    --lift_height/-lift //The lift height
    --scale/-scl //How much to scale the image, default: 1
    --careful //Create more liftpoints to avoid accidents
    --origin/-o //The origin of where to draw the image, pivot on image is top-left corner.
Important to note that by default the program treats 1px as 1cm, therefore scaling down may be required to keep the image within the workspace of the robot.
