# Setting up the Inovo Robot's network for remote ROS communication

In order to connect and control the Inovo robot using ROS, you must first configure the environment on your development machine.

The first step ts to make sure you can connect to the Inovo robot using its hostname.
Note: If your DHCP and DNS server are the same thing, and it is configured in just the right way, this will work automatically.
More often than not, however, you will need to add the robot's hostname to your `/etc/hosts` file like so:

Note: replace `192.168.1.1` with the IP address of your robot and `psu001` it's hostname, both of which can be found on the pendant's home page.
```
192.168.1.1 psu001
```

Your machine should now be able to resolve the robot via this hostname, which you can test by trying to ping it.

Configure the following environment variables.
```
export ROS_MASTER_URI=http://psu001:11311
export ROS_IP=192.168.1.2
```
Replace `192.168.1.2` with the IP address of your local machine. This should be its external IP which the robot will use to connect to your local machine.

You might find it handy to add these lines to your `.bashrc` file, which will set up this configuration for every new bash session.
```
echo "export ROS_MASTER_URI=http://psu001:11311" >> ~/.bashrc
echo "export ROS_IP=192.168.1.2" >> ~/.bashrc
source ~/.bashrc
```

## Testing the Network

Now you can try to communicate with the robot. You can check the connection by running a test example script in this repository.
First make sure you have set up your workspace, instructions for this can be found in the [README](../README.md).
```
cd ~/inovo_ws
source devel/setup.bash
```

Now run the following command to check if the network is OK:
```
rosrun commander_api check_network.py
```

It should print something like this:
```
Connected to the 'move' action server. Network is configured correctly!
```
