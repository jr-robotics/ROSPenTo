# ROSPenTo
The Robot Operating System (ROS) penetration testing tool (ROSPenTo) can send XML remote procedure calls (XMLRPC) to the ROS-Master and to ROS-Nodes.

## Description
ROSPenTo is a penetration testing tool for the Robot Operating System (ROS). Any ROS-Network can be analyzed.

```
By no means do we want to encourage or promote the unauthorized tampering with
running robotic applications since this can cause damage and serious harm.

Nevertheless, we think it is important to show that those vulnerabilities exist
and to make the ROS community aware how easily an application can be undermined.
```

## Getting Started

### Compiling and running
#### Windows
In Windows, you can compile and run ROSPenTo straightforward using

```
nuget restore
msbuild
cd RosPenToConsole\bin\debug\
RosPenToConsole.exe
```
or open, compile and run it in Visual Studio.

#### Linux
In Linux, make sure you have a current version of Mono installed (apt-get'ing might return an old version).

Follow the installation instructions on the Mono website http://www.mono-project.com/download/stable/#download-lin

Afterwards (in the root folder of ROSPenTo) run

```
nuget restore
msbuild
cd RosPenToConsole/bin/Debug
mono RosPenToConsole.exe
```

#### Docker container
```bash
docker build -t rospento . # build the container
docker run -it rospento
> ROSPenTo #  an alias for "rospento" has also been created
```


In running, there are two options to use the ROSPenTo:
* without parameters
* with parameters

### Without parameters
When the application is started without parameters the command line interface (CLI) is shown.
After a system was analyzed (1.) the following commands can be executed:

(0.) Exit

(1.) Analyse system...

(2.) Print all analyzed systems

(3.) Print information about analyzed system...

(4.) Print nodes of analyzed system...

(5.) Print node types of analyzed system (Python or C++)...

(6.) Print topics of analyzed system...

(7.) Print services of analyzed system...

(8.) Print communications of analyzed system...

(9.) Print communications of topic...

(10.) Print parameters...

(11.) Update publishers list of subscriber (add)...

(12.) Update publishers list of subscriber (set)...

(13.) Update publishers list of subscriber (remove)...

(14.) Isolate Service...

(15.) Unsubscribe node from parameter (only C++)...

(16.) Update subscribed parameter at Node (only C++)...

More than one ROS-Network can be analyzed.

Run the following command to start the CLI:
```
RosPenToConsole.exe
```

### With parameters
When you specify parameters you can run a **publisher update** command
* -t, --target     Required. ROS Master URI of the target system.
* -p, --pentest    Required. ROS Master URI of the penetration testing system.
* --sub            Required. Name of the affected subscriber in the target system.
* --top            Required. Name of the affected topic.
* --pub            Required. Name of the new publisher in the penetration testing system.
* --add            (Default: False) publisherUpdate command adds publisher to existing ones.
* --set            (Default: False) publisherUpdate command sets new publisher.
* --remove         (Default: False) publisherUpdate command removes publisher from existing ones.

You have to specify all the required parameters and exactly one of the following: ```{--add, --set, --remove}```

E.g.: The following command runs the publisher update procedure on the local machine with two running roscores and adds a new publisher to the existing subscriber:
```
RosPenToConsole.exe -t http://127.0.0.1:11311 --sub /subscriberNode -p http://127.0.0.1:11312 --top /topicName --pub /newPublisherNode --add
```

# An example

Run two roscores on your machine
```
roscore&
roscore -p 11312&
```
Run a publisher in the first master

```
rosrun rospy_tutorials talker
```

Run a subscriber in the second master

```
export ROS_MASTER_URI=http://localhost:11312
rosrun rospy_tutorials listener
```

Initially, you will see no output from the listener since it is running "alone" within its ROS network.


Then start ROSPenTo to analyze the network (using mono on Linux or without it).
Follow the instructions on https://www.mono-project.com/download/stable/#download-lin to install mono.
```
(mono )RosPenToConsole.exe
```

Press 1

Enter the host URI `http://localhost:11311`

You will see the nodes running in the first ROS core (mainly rosout)

Press 1 again

Enter the host URI `http://localhost:11312`

You will see a similar output but with different ports.

Let's assume the following output for the first roscore (System 0)

```
System 0: http://127.0.0.1:11311/
Nodes:
        Node 0.1: /rosout (XmlRpcUri: http://127.0.0.1:41865/)
        Node 0.0: /talker_5957_1529503884881 (XmlRpcUri: http://127.0.0.1:46015/)
Topics:
        Topic 0.0: /chatter (Type: std_msgs/String)
        Topic 0.1: /rosout (Type: rosgraph_msgs/Log)
        Topic 0.2: /rosout_agg (Type: rosgraph_msgs/Log)
Services:
        Service 0.3: /rosout/get_loggers
        Service 0.2: /rosout/set_logger_level
        Service 0.1: /talker_5957_1529503884881/get_loggers
        Service 0.0: /talker_5957_1529503884881/set_logger_level
Communications:
        Communication 0.0:
                Publishers:
                        Node 0.0: /talker_5957_1529503884881 (XmlRpcUri: http://127.0.0.1:46015/)
                Topic 0.0: /chatter (Type: std_msgs/String)
                Subscribers:
        Communication 0.1:
                Publishers:
                        Node 0.0: /talker_5957_1529503884881 (XmlRpcUri: http://127.0.0.1:46015/)
                Topic 0.1: /rosout (Type: rosgraph_msgs/Log)
                Subscribers:
                        Node 0.1: /rosout (XmlRpcUri: http://127.0.0.1:41865/)
        Communication 0.2:
                Publishers:
                        Node 0.1: /rosout (XmlRpcUri: http://127.0.0.1:41865/)
                Topic 0.2: /rosout_agg (Type: rosgraph_msgs/Log)
                Subscribers:
```

And this for the second roscore (System 1)

```
System 1: http://127.0.0.1:11312/
Nodes:
        Node 1.0: /listener_6113_1529504103477 (XmlRpcUri: http://127.0.0.1:40499/)
        Node 1.1: /rosout (XmlRpcUri: http://127.0.0.1:37823/)
Topics:
        Topic 1.1: /chatter (Type: std_msgs/String)
        Topic 1.0: /rosout (Type: rosgraph_msgs/Log)
        Topic 1.2: /rosout_agg (Type: rosgraph_msgs/Log)
Services:
        Service 1.1: /listener_6113_1529504103477/get_loggers
        Service 1.0: /listener_6113_1529504103477/set_logger_level
        Service 1.3: /rosout/get_loggers
        Service 1.2: /rosout/set_logger_level
Communications:
        Communication 1.0:
                Publishers:
                        Node 1.0: /listener_6113_1529504103477 (XmlRpcUri: http://127.0.0.1:40499/)
                Topic 1.0: /rosout (Type: rosgraph_msgs/Log)
                Subscribers:
                        Node 1.1: /rosout (XmlRpcUri: http://127.0.0.1:37823/)
        Communication 1.1:
                Publishers:
                Topic 1.1: /chatter (Type: std_msgs/String)
                Subscribers:
                        Node 1.0: /listener_6113_1529504103477 (XmlRpcUri: http://127.0.0.1:40499/)
        Communication 1.2:
                Publishers:
                        Node 1.1: /rosout (XmlRpcUri: http://127.0.0.1:37823/)
                Topic 1.2: /rosout_agg (Type: rosgraph_msgs/Log)
                Subscribers:

```
Now perform the following sequence


`11  <--- press 11 to perform a publisher update`
```
To which subscriber do you want to send the publisherUpdate message?
Please enter number of subscriber (e.g.: 0.0):
```
`1.0 <--- Select the subscriber 0 from System 1`
```
Which topic should be affected?
Please enter number of topic (e.g.: 0.0):
```
`1.1 <--- Select topic "Chatter" from System 1`
```
Which publisher(s) do you want to add?
Please enter number of publisher(s) (e.g.: 0.0,0.1,...):
```
`0.0 <--- Select the first node from Systm 0`
```
sending publisherUpdate to subscriber '/listener_6113_1529504103477 (XmlRpcUri: http://127.0.0.1:40499/)' over topic '/chatter (Type: std_msgs/String)' with publishers '/talker_5957_1529503884881 (XmlRpcUri: http://127.0.0.1:46015/)'
PublisherUpdate completed successfully.
```

Now the chatter publisher from the first ROS network publishes to the subscriber in the second.

You should see the output of the listener now.
