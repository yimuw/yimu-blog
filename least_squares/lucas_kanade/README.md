# Lucas-Kanade

Simple implementation of Lucas-Kanade in C++.

## 1. Reading

Check this article about Lucas-Kanade tracker basis: <->

Check this article about Lucas-Kanade tracker with covariance: <->

Check this article about Lucas-Kanade tracker on SE2: <->

## 2. How build

### dependency

The only dependency is opencv.

On ubuntu18.04, you can install it by,

```
apt-get install libopencv-dev
```

On ubuntu16.04, it is a little bit tricky.

your options are
1. talk to your devOps to get it
2. build from source: https://www.learnopencv.com/install-opencv-3-4-4-on-ubuntu-16-04/
3. use a docker with opencv installed.

### build

in the "yimu-blog/least_squares/lucas_kanade/" folder, do

```
mkdir build
cd build
cmake ..
make
```

### 3. How to run

I push sample images in "yimu-blog/data/image_seqence_basketball". Not the best practice but it makes our life easier.
#### 3.1 tracking

Assmuing you are in "yimu-blog/least_squares/lucas_kanade/build",
```
./lk_translation2d/lucas-kanada <path-to /yimu-blog/data/image_seqence_basketball>
```

#### 3.2 tracking with covariance

Assmuing you are in "yimu-blog/least_squares/lucas_kanade/build",
```
./lk_translation2d/lucas-kanada <path-to /yimu-blog/data/image_seqence_basketball> show_cov
```

