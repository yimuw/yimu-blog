# Shared Memory Inter-Process Communication

A simple IPC system implemented by Shared Memory.

I implemented a IPC system by TCP. The details are in this article: https://wang-yimu.com/tcp-ip-net-video-player/

Due to the streaming nature of TCP protocol, I had to 

1. Serialize a message into chuck of data.
2. Break the chunk of data into packages.
3. Send the packages by TCP. 
4. Signal the end of the message.
5. Receive all the packages and deserialize.

For large messages, the procedure can be inefficient. 

Alternatively, we can do the IPC using Shared Memory. 

Basically, if I want to send a object to another process, I just need to copy the object into the share memory. The reader process simply needs to read the object. (The assumption is the object is shallow copyable).

However, the share memory is subjected to race condition. So I need to design a way to synchronize the readers and the writers.

Check this article for the design of the system: https://wang-yimu.com/tcp-ip-net-video-player/

## 1. Build

### 1.1 Dependency

The code only runs on Linux since it depends on Linux Shared Memory system calls.

The only C++ dependency is openCV.

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

### 1.2 make

in the `/yimu-blog/comms/shared_mem` folder, do

```
mkdir build
cd build
cmake ..
make
```

## 2. How to run

I pushed sample images in `yimu-blog/data/image_seqence_basketball`. It is not the best practice but it makes our life easier. I highly recommend using the sample images to play with the system since I hard coded some parameters.

You need to open 3 terminals because we are going to run the **writer** process, the **reader1** and the **reader2**.

### Run the writer process

Assuming you are in the build directory.

Run this command in your first terminal. The argument is the path to the image directory.

```
yimu@yimu-mate: ./writer <path to the image dir. the images are in /yimu-blog/data/image_seqence_basketball/>
```

The writer process loads images from the dir, shows the images and writes the images into the shared memory IPC system.

### Run the reader1

Assuming you are in the build directory.

To run the reader1, run

```
./reader1
```

The reader1 process reads the images, converts images to gray and displays them.

### Run the reader2

Assuming you are in the build directory.

To run the reader2, run

```
./reader2
```

The reader2 process reads the images, does the Canny edge detection and displays the edge images.

## 4. Licence

BSD license

Optional but appreciated: If you benefit from the work, you can ONLY ask author leetcode **easy** questions.



