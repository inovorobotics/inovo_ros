# Setting up the network

There are various ways to configure the robot to allow ROS communications to take place.

## Setting up the ROS master

### Using the internal ROS master (Recommended)
The Inovo robot control unit runs a ROS master by default on port 11311. Before running any ROS nodes on an external machine, set the ROS_MASTER_URI environment variable.
```
export ROS_MASTER_URI=http://<hostname-of-inovo-control-unit>:11311
rosrun ...
```

You can also configure this to happen on all new terminals:
```
echo "export ROS_MASTER_URI=http://<hostname-of-inovo-control-unit>:11311" >> ~/.bashrc
source  ~/.bashrc
```

### Using an alternative ROS master
SSH into the Inovo robot control unit and modify the RCU's service file `/etc/systemd/system/rcu.service`

```
...
[Service]
# Important - roslaunch doesn't trap SIGTERM
KillSignal=SIGINT 
User=root

# ADD THIS LINE
Environment="ROS_MASTER_URI=http://<master-ip-address>:<master-port-number>"

Environment="ROS_PYTHON_LOG_CONFIG_FILE=/etc/ros/python_logging.conf"
ExecStart=/bin/bash -c "source /opt/rcu/setup.bash && mon launch --disable-ui --flush-stdout --name=rosmon --disable-diagnostics --log=/dev/null rcu ${LAUNCHFILE}"
...
```

Then restart the RCU service:
```
systemctl restart rcu
```

## Setting up the hostnames
ROS nodes communicate with each other using hostname lookups. By default, nodes will report to the ROS master using the hostname which is provided by using `hostname` command. For the inovo control unit this will be something like `psuXXX` where `XXX` is the serial number. For ROS communications to work, all hosts on the ROS network must be able to resolve each other's IP addresses via their hostname.

It is possible to configure some router's DHCP/DNS servers to resolve hosts via their hostname, but configuration varies between router manufacturers and is outside the scope of this tutorial.

If all else fails then you will need add any ROS hosts hostnames into the /etc/hosts file in the Inovo control unit.

```
# SSH into the RCU
ssh root@psuXXX

# Add the hostname and IP, repeat for all hosts
echo "<host-ip> <hostname>" >> /etc/hosts

# Test this hostname is resolveable
ping <hostname>
```

Repeat this process for all your nodes that should participate in the ROS network.
