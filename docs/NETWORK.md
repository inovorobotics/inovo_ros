# Setting up the Inovo Robot's network for remote ROS communications

In order to connect and control the Inovo robot using ROS, you must first configure the environment on your development machine.

The first step ts to make sure you can connect to the Inovo robot using its hostname.
Note: If your DHCP and DNS server are the same thing, and it is configured in just the right way, this will work automatically. Most of the time however you will need to add the robot's hostname to your `/etc/hosts` file like so:

Note: replace `psuXXX` below with the hostname of your robot, which can be found on the pendant's home page.
```
192.168.2.3 psuXXX
```

Test the configuration is correct by trying to ping robot using it's hostname.
```
ping psuXXX
```

You should see something like this:
```
PING psuXXX (192.168.2.3) 56(84) bytes of data.
64 bytes from psuXXX (192.168.2.3): icmp_seq=1 ttl=64 time=1.27 ms
64 bytes from psuXXX (192.168.2.3): icmp_seq=2 ttl=64 time=1.22 ms
64 bytes from psuXXX (192.168.2.3): icmp_seq=3 ttl=64 time=1.23 ms
```
Press Ctrl-C to cancel the ping command.

Now you should configure some environment variables to communicate with your development machine:
```
export ROS_MASTER_URI=http://psuXXX:11311
export ROS_IP=<dev-machine-ip>
```
Replace `<dev-machine-ip>` with your development machine's external IP address. This IP should be accessible from the Inovo robot to allow connections back to your machine.

You might find it handy to add these lines to your `.bashrc` file.
```
echo "export ROS_MASTER_URI=http://psuXXX:11311" >> ~/.bashrc
echo "export ROS_IP=<dev-machine-ip>" >> ~/.bashrc
source ~/.bashrc
```
