# The rundown

`npm install roslib` gets the js library for ros websockets

`sudo apt-get install ros-humble-bridge-suite` gets the rospackage for ros websockets

launch webserver for ros data: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`. By default uses port `9090`, but can be changed with `port:=[SOME_PORT]`.

launch webserver for webapp: `python3 -m http.server 8080 --bind 0.0.0.0`. The `--bind 0.0.0.0` flag makes the webserver listen to all network interfaces. Run this command **Within** the folder of `index.html`.

Sequential scripts to run demo node in the html file
```
    <script type="text/javascript" src="node_modules/roslib/build/roslib.min.js"></script>
    <script type="text/javascript" src="ros_demo.js"></script>
```

## Publishing topics from the command line to websockets
```
ros2 topic pub /some_topic msg_type "data : some_msg"
// must be in we YAML format. As in, use data, followed by a colon.
```
## Accessing Webapp from other computers
This project focuses on LAN Webserver hosting, meaning you don't access it via a typical URL. Instead, for whatever computer is running the Webserver (the one that houses the webapp, typically will be Chimera), you must enter the **local IP** of the computer hosting it, followed by the port. Here is an example of how to find the IP addresses of other computers on the network, how to find your own IP, and what to put in the address bar of your browser to get to the webapp:
```
// Finding your local IP (linux):
ip addr show | grep 'inet'

// Finding all IPs on a network (linux):
nmap -sP {your_local_ip_up_to_last"."}

// What to put in the search bar:
{local_ip_of_webserver_pc}:{port}
```