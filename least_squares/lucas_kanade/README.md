# Lucas-Kanade

Lucas-Kanade implementation in C++.

## 1. Reading

Check this article about Lucas-Kanade tracker basis: https://wang-yimu.com/lucas-kanade-tracker/

Check this article about Lucas-Kanade tracker with covariance: https://wang-yimu.com/optimization-or-probability-the-uncertainty-for-optimization-problems/

Check this article about Lucas-Kanade tracker on SE2: <->

## 2. Build

### 2.1 dependency

The only dependency is opencv.

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

### 2.2 make

in the `yimu-blog/least_squares/lucas_kanade/` folder, do

```
mkdir build
cd build
cmake ..
make
```

## 3. How to run

I push sample images in `yimu-blog/data/image_seqence_basketball`. Not the best practice but it makes our life easier.
#### 3.1 tracking

Assmuing you are in `yimu-blog/least_squares/lucas_kanade/build`,
```
./lk_translation2d/lucas-kanada <path-to /yimu-blog/data/image_seqence_basketball>
```

#### 3.2 tracking with covariance

Assmuing you are in `yimu-blog/least_squares/lucas_kanade/build`,
```
./lk_translation2d/lucas-kanada <path-to /yimu-blog/data/image_seqence_basketball> show_cov
```

## 4. Licence

BSD license

Optional: If you benefit from the work, you can ONLY ask author leetcode easy questions.



