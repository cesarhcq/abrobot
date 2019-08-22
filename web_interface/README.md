# GuntherBOT - Autonomous Mobile Robot with Web Interface (Bootstrap 4 + ROS)

GuntherBOT is an autonomous mobile robot developed by [Acta Visio](http://www.acta-visio.com) for warehouse indoor environments. GuntherBOT is based on Arlo Robot as mobile platform to perform different tasks in an indoor environments. GuntherBOT was developed with Robot Operating System (ROS) - see more in [ROS Wiki](https://www.ros.org/), Arduino Leonardo and Raspberry Pi 3B. 

Bootstrap 4 framework allows you to create modern, great looking websites really quickly and without a big effort. It is currently one of the most popular front-end libraries for a reason. It is also open source [Bootstrap 4 + ROS](https://medium.com/husarion-blog/bootstrap-4-ros-creating-a-web-ui-for-your-robot-9a77a8e373f9) and available under developer friendly MIT license. Building websites which not only look good but also scale well on different devices such as your laptop or a mobile phone is not a big deal with Bootstrap.

In this README We will show how to use that framework together with ROS (Robot Operating System) — the most popular robotic middleware — to create awesome, intuitive web user interfaces for robots. All the work related a interface web was based in this article [Link](https://medium.com/husarion-blog/bootstrap-4-ros-creating-a-web-ui-for-your-robot-9a77a8e373f9)

Version | ROS Distro | Operating System
------------ | ------------- | ------------
1.0 | Kinetic Kame | Ubuntu 16.04 LTS (Xenial)

## Install dependencies and follow the installation instructions.

- [x] ROS Kinetic-devel: [ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu).
- [x] Install MS Visual Studio Code in Ubuntu 16.04/16.10: [VSC](http://tipsonubuntu.com/2017/03/03/install-ms-visual-studio-code-ubuntu-16-0416-10/).
- [x] Bootstrap 4, Font awesome 4, Font Awesome 5 Free & Pro snippets: [Bootstrap 4...](https://marketplace.visualstudio.com/items?itemName=thekalinga.bootstrap4-vscode).
- [x] Live Server: [LS](https://marketplace.visualstudio.com/items?itemName=ritwickdey.LiveServer).
- [x] Prettier Code formatter: [PCF](https://marketplace.visualstudio.com/items?itemName=esbenp.prettier-vscode).
- [x] vscode-icons:[Vsc.Icon](https://marketplace.visualstudio.com/items?itemName=vscode-icons-team.vscode-icons).

## Requirements

### 1. Laptop / PC with installed Visual Studio Code (VSC)

First, in VSC you need to install the following plugins:

- **Bootstrap 4, Font awesome 4, Font Awesome 5 Free & Pro snippets:** this is the framework itself with couple of additional features which contain ready-to-use templates and other stuff useful while developing in bootstrap

- **Live Server:** to see your web UI changes in realtime

- **Prettier Code formatter:** to keep your source code clean

- **vscode-icons:** to make your workspace look good

### 2. Physical robot or gazebo simulation model

On top of standard ROS, you will need to install couple additional packages. To do that, in your robot Linux terminal run the following commands:

```
sudo apt update
```

and

```
sudo apt install python-tornado python-pip ros-kinetic-rosbridge-suite ros-kinetic-web-video-server nginx
```

## Creating a web UI in VSC

### 1. Creating a new project

First create an empty folder (we named it `web_interface`). Then start VSC and open the folder you just created. In VSC create a new file called named `index.html`. Thanks to one of the plugins for VSC which we have installed earlier creating a new Bootstrap 4 project is very straightforward. Just start typing `b4`, then press `ctrl + space` for auto completion menu to appear and choose `b4-$` . It will create a web page template with bootstrap libraries already included.
Here’s how creating a new project looks like:

Creating a new web UI project in VSC [Link-YouTube](https://www.youtube.com/watch?v=2PAQu0AnJ40)

Now right click the `index.html` file and select “Open with Live Server [Alt + L Alt + O]” option. You will see an empty page in your web browser . It will be updated automatically each time you save your project. If you have trouble to use, try to make manual install in Live Server: [LS](https://marketplace.visualstudio.com/items?itemName=ritwickdey.LiveServer).

### 2. Writing the HTML code

- Change the name of your page in the `<title>` tag to “ROS + Bootstrap 4 demo”: `<title>ROS + Bootstrap 4 demo</title>`

- Change the page background color to `bg-light` inside the `<body>` tag: `<body class="bg-light">`

- Add some additional scripts above `</body>` tag:

```
<script src="https://static.robotwebtools.org/roslibjs/current/roslib.min.js"></script>
<script src="https://static.robotwebtools.org/EventEmitter2/current/eventemitter2.min.js"></script>
<script src="https://static.robotwebtools.org/keyboardteleopjs/current/keyboardteleop.min.js"></script>
<script src="//yoannmoinet.github.io/nipplejs/javascripts/nipplejs.js"></script>
<script src="webui.js"></script>
```

First 3 scripts come from Robot Web Tools(http://robotwebtools.org/ ). `roslib.min.js` is basic ROS javascript library, it allows your web apps to communicate with ROS system. `eventemitter2.min`.js is a support library for handling various ROS events. `keyboardteleop.min.js` is a support library for controlling the robot using the keyboard.

`nipplejs.js` is a library for creating a joystick for the web UI that is especially useful for controlling the robot on mobile devices.

And finally `webui.js` will be created in the next section of this article.

- Below `<body class="bg-light">` paste following code:


```
<!-- SPEED -->
<div class="row">
    <div class="col-md-4"></div>
    <div class=" col-md-4">
        <label for="robot-speed">
            <strong>Robot speed</strong>
        </label>
        <input type="range" min="15" max="80" class="custom-range" id="robot-speed">
    </div>
    <div class="col-md-4"></div>
</div>

<!-- VIDEO -->
<div class="row my-4">
    <div class="col d-flex justify-content-center">
        <img src="" class="p-1 bg-dark" alt="" id="video" />
    </div>
</div>

<!-- JOYSTICK -->
<div class="row my-4">
    <div class="col">
        <div class="d-flex justify-content-center" style="width: 210px; height: 210px;">
            <div id="joystick"></div>
        </div>
    </div>
</div>

<!-- INFO -->
<div class="row my-4">
    <div class="col-md-2"></div>
    <div class="col-md-8">
        <div class="alert alert-success">
            <h4 class="alert-heading">ROS + Bootstrap interface demo</h4>
            <ul>
                <li>set speed using a slider</li>
                <li>use joystick or WASD keys on keyboard to move </li>
            </ul>
        </div>
    </div>
    <div class="col-md-2"></div>
</div>
```

If do not work, try this aternative link - [Link to Open](https://gist.github.com/DominikN/52daa750c924b368fdacba621bfb975f#file-bootstrap-ros-demo-2-html).

This is a basic structure of your app. In consists of a placeholder for the video, joystick and slider for selecting keyboard speed. Do not worry that `<img>` tag has an empty src parameter, it will be populated by javascript.

That is all for the html part, remember to save all the changes made in `index.html`.

### 3. Writing the JavaScript code

Create a new file, name it `webui.js` and save in the same folder as `index.html`.

Begin by creating some global variables:

```
var twist;
var cmdVel;
var publishImmidiately = true;
var robot_IP;
var manager;
var teleop;
var ros;
```

Write a function to send ROS messages with velocity for robot:

```
function moveAction(linear, angular) {
    if (linear !== undefined && angular !== undefined) {
        twist.linear.x = linear;
        twist.angular.z = angular;
    } else {
        twist.linear.x = 0;
        twist.angular.z = 0;
    }
    cmdVel.publish(twist);
}
```

Write a function to send ROS messages with velocity for robot:


```
function initVelocityPublisher() {
    // Init message with zero values.
    twist = new ROSLIB.Message({
        linear: {
            x: 0,
            y: 0,
            z: 0
        },
        angular: {
            x: 0,
            y: 0,
            z: 0
        }
    });
    // Init topic object
    cmdVel = new ROSLIB.Topic({
        ros: ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/Twist'
    });
    // Register publisher within ROS system
    cmdVel.advertise();
}
```

Write a function to create keyboard controller object:

```
function initTeleopKeyboard() {
    // Use w, s, a, d keys to drive your robot

    // Check if keyboard controller was aready created
    if (teleop == null) {
        // Initialize the teleop.
        teleop = new KEYBOARDTELEOP.Teleop({
            ros: ros,
            topic: '/cmd_vel'
        });
    }

    // Add event listener for slider moves
    robotSpeedRange = document.getElementById("robot-speed");
    robotSpeedRange.oninput = function () {
        teleop.scale = robotSpeedRange.value / 100
    }
}
```

Write a function to create joystick object:

```
function createJoystick() {
    // Check if joystick was aready created
    if (manager == null) {
        joystickContainer = document.getElementById('joystick');
        // joystck configuration, if you want to adjust joystick, refer to:
        // https://yoannmoinet.github.io/nipplejs/
        var options = {
            zone: joystickContainer,
            position: { left: 50 + '%', top: 105 + 'px' },
            mode: 'static',
            size: 200,
            color: '#0066ff',
            restJoystick: true
        };
        manager = nipplejs.create(options);
        // event listener for joystick move
        manager.on('move', function (evt, nipple) {
            // nipplejs returns direction is screen coordiantes
            // we need to rotate it, that dragging towards screen top will move robot forward
            var direction = nipple.angle.degree - 90;
            if (direction > 180) {
                direction = -(450 - nipple.angle.degree);
            }
            // convert angles to radians and scale linear and angular speed
            // adjust if youwant robot to drvie faster or slower
            var lin = Math.cos(direction / 57.29) * nipple.distance * 0.005;
            var ang = Math.sin(direction / 57.29) * nipple.distance * 0.05;
            // nipplejs is triggering events when joystic moves each pixel
            // we need delay between consecutive messege publications to 
            // prevent system from being flooded by messages
            // events triggered earlier than 50ms after last publication will be dropped 
            if (publishImmidiately) {
                publishImmidiately = false;
                moveAction(lin, ang);
                setTimeout(function () {
                    publishImmidiately = true;
                }, 50);
            }
        });
        // event litener for joystick release, always send stop message
        manager.on('end', function () {
            moveAction(0, 0);
        });
    }
}
```

…and finally main app initialization:

```
window.onload = function () {
    // determine robot address automatically
    // robot_IP = location.hostname;
    // set robot address statically
    robot_IP = "10.5.10.117";

    // // Init handle for rosbridge_websocket
    ros = new ROSLIB.Ros({
        url: "ws://" + robot_IP + ":9090"
    });

    initVelocityPublisher();
    // get handle for video placeholder
    video = document.getElementById('video');
    // Populate video source 
    video.src = "http://" + robot_IP + ":8080/stream?topic=/camera/rgb/image_raw&type=mjpeg&quality=80";
    video.onload = function () {
        // joystick and keyboard controls will be available only when video is correctly loaded
        createJoystick();
        initTeleopKeyboard();
    };
}
```

Choose between static IP address for the robot or dynamically defined IP with `location.hostname` depending on your configuration. If the web server is running on a device different than the robot, IP must be set manually to the robot address. If the app is deployed to a server which is running on your robot, it can be set automatically.

Video address string might need to be adjusted. Part `/camera/rgb/image_raw` is the topic name, where video is published by the robot. Set this topic according to your camera.

You web user interface is now ready.

## Running the demo on your robot

### 1. Flashing default firmware to ROSbot 2.0 real-time board