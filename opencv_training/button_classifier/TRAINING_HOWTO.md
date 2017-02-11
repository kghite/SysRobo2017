# Training the Button Panel Classifier (OpenCV 2.4)

## Getting Training Images

### Video

**Positive**: Take about 2 minutes of video of the feature to be detected in different lights, orientations, and angles.  Every frame of the video should contain the feature.

**Negative**: Take about 2 minutes of video in the same environment without the feature present.

### Splitting

Place the videos into the project directory and split them into images with the following two commands:

```
ffmpeg -i positive.mp4 positive_images/positive%d.jpg
ffmpeg -i negative.mp4 negative_images/negative%d.jpg
```

To record the location of each of the files, run the following commands in the project directory.

```
find ./positive_images -iname "*.jpg" > positives.txt
find ./negative_images -iname "*.jpg" > negatives.txt

```

## Creating Positive Samples

### Get Files

OpenCV has a createsamples function that will expand the small positive sample set by changing lighting and orientation to create new images.

Clone the example repository below to copy utility files from.

'git clone https://github.com/mrnugget/opencv-haar-classifier-training'

Move the bin and tools folders along with their contents into the project directory with the image folders and then delete the git repo that was just cloned.

### Create Samples

Run the following command in the project directory after changing the parameters below as necessary:

**-w** and **-h**: Should be a compact ratio of your images from the video. For example, using 1080wx1920h images, the aspect ratio is 9:16, which scales to -w 45 -h 80.  Higher scale numbers mean higher quality but longer training time.

```
perl bin/createsamples.pl positives.txt negatives.txt samples 1500\
  "opencv_createsamples -bgcolor 0 -bgthresh 0 -maxxangle 1.1\
  -maxyangle 1.1 maxzangle 0.5 -maxidev 40 -w 80 -h 40"
```

## Create Sample .vec files

All of the generated sample files need to be combined into one samples.vec file for traincascade.  To do this, run the python script in the tools folder as follows:

```
find ./samples -name '*.vec' > samples.txt
python tools/mergevec.py -v samples/ -o samples.vec
```

## Train the Classifier

Run the following command in the project directory after changing the parameters below as necessary:

**-numStages**: Between 10 and 20 for a run time of a few hours but good recognition quality.

**-numPos**: About 0.8 x number of samples that were generated into the samples folder. (Some samples get discarded, so you need extra.)

**-numNeg**: Number of images in you negative_images folder.

**-w** and **-h**: Same as the parameters for createsamples.pl.

```
opencv_traincascade -data classifier -vec samples.vec -bg negatives.txt\
  -numStages 20 -minHitRate 0.999 -maxFalseAlarmRate 0.5 -numPos 400\
  -numNeg 600 -w 45 -h 80 -mode ALL -precalcValBufSize 1024\
  -precalcIdxBufSize 1024
```

## Using the Result

The trainer should output a .xml file with the classification data. Put this file in the python_scripts directory and update the detector script with the correct file name.