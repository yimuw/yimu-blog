# TCP/IP Net Video Player

A toy Net Video Player implementation in C++.

## 1. Reading

Check this article for a introduction to TCP/IP and the design of the system: https://wang-yimu.com/tcp-ip-net-video-player/

## 2. Build

### 2.1 Dependency

The code only runs on Linux since it depends on Linux TCP system calls.

The only C++ dependency is openCV.

If you can't get openCV, you can comment the net player. There is another example which transfer numbers.

#### On ubuntu18.04 

you can install opencv by,

```
apt-get install libopencv-dev
```

#### On ubuntu16.04 

It is a little bit tricky.

your options are
1. Talk to your devOps to get it
2. Build from source: https://www.learnopencv.com/install-opencv-3-4-4-on-ubuntu-16-04/
3. Use a docker with opencv installed. It is actually super easy. https://www.learnopencv.com/install-opencv-docker-image-ubuntu-macos-windows/
4. Commented the compilation of net player. There is another example of TCP/IP.

### 2.2 make

in the `/yimu-blog/comms/tcp_ip` folder, do

```
mkdir build
cd build
cmake ..
make
```

## 3. How to run

### 3.1 Net video player

I push sample images in `yimu-blog/data/image_seqence_basketball`. Not the best practice but it makes our life easier. The image message size is **hard-coded** to use the sample images.

You can start 2 terminals to run the server and client.

Or you can start the server on one machine and start client on another machine. But please make sure those two machines are in the same LAN(local area network).

### **Start the Server**

First we need to know the ip address of your machine.

Assuming you are in `yimu-blog/comms/tcp_ip/build`,

Running `./play_video/play_video_server` without arguments outputs the ip address of your machine.

```
yimu@yimu-mate: /yimu-blog/comms/tcp_ip/build$ ./play_video/play_video_server 

wlp60s0 IP Address 192.168.1.9
please make sure you connected to the wlp ip
ArgumentParser error: too few required arguments passed to ./play_video/play_video_server
Usage: ./play_video/play_video_server --ip IP --port PORT --image-dir IMAGE-DIR
```


Then start the server with the **ip address of your machine**, a **port number** and the **path to images**.

The port number can be any number as long as the port is not in use. 

Start the server by,
```
yimu@yimu-mate:/yimu-blog/comms/tcp_ip/build$ ./play_video/play_video_server --ip 192.168.1.9 --port 3491 --image-dir ../../../../yimu-blog/data/image_seqence_basketball
```

Now the server started. It is waiting for clients.

### **Start the Client**

Start the client with the ip address and the port number of the **server**.

```
yimu@yimu-mate:/yimu-blog/comms/tcp_ip/build$ ./play_video/play_video_client --ip 192.168.1.9 --port 3491
```

Once the client connect to the server. The server starts to transfer the video to the client. The client plays the video.

Hit `enter` in the client terminal to stop/resume the video. 

When `enter` key is pressed. The client send a control message to the server. The server stop/resume image reading and transition.


### **Stop the client**

Due to a threading issue when I handle the `enter` input, stopping the client is a little bit strange. 

Stop the client by,

1. hit `ctrl+c`
2. hit `enter`

You can stop the server by stop the client. I know, It is strange.

### 3.2 Another example, transfer number

This example has no dependency to opencv.

You can start 2 terminal to run the server and client.

Or you can start the server on one machine and start client on another machine. But please make sure those two machines are in the same LAN(local area network).

### **Start the Server**

First we need to know the ip address of your machine.

Assuming you are in `yimu-blog/comms/tcp_ip/build`,

Running `./send_number/server_number` without arguments outputs the ip address of your machine.

```
yimu@yimu-mate:~/Desktop/yimu-blog/comms/tcp_ip/build$ ./send_number/server_number 
wlp60s0 IP Address 192.168.1.9
please make sure you connected to the wlp ip
ArgumentParser error: too few required arguments passed to ./send_number/server_number
Usage: ./send_number/server_number --ip IP --port PORT
```

Then start the server with the **ip address of your machine**, a **port number**.

The port number can be any number as long as the port is not in use. 

```
yimu@yimu-mate:~/Desktop/yimu-blog/comms/tcp_ip/build$ ./send_number/server_number --ip 192.168.1.9 --port 2333
```

Now the server started. It is waiting for clients.

### **Start the client**

Start the client with the ip address and the port number of the **server**.

```
yimu@yimu-mate:~/Desktop/yimu-blog/comms/tcp_ip/build$ ./send_number/client_number --ip 192.168.1.9 --port 2333
```

Now the client connects the server.

The server send double (start from 0) to the client.

The client send int (start from 1000) to the server.

### **Stop the Client/Server**

`ctrl+c`

## 4. Licence

BSD license

Optional but appreciated: If you benefit from the work, you can ONLY ask author leetcode **easy** questions.



