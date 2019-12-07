FROM ubuntu:18.04

RUN apt-get update
RUN apt-get upgrade -y

RUN apt-get install -y\ 
        python3-numpy \ 
        python3-scipy \
        python3-matplotlib

RUN apt-get install -y\ 
        build-essential \
        cmake \ 
        unzip

RUN apt-get install -y libopencv-dev python3-opencv