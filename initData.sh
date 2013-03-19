#!/bin/bash

cp ./data/dataToLoadOpenCVFile.yml ${HOME}/.vs_core/plugins/skeletonTrackerVSNI/


cp ./data/skeletonTrackerVSNI.conf ${HOME}/.vs_core/plugins/skeletonTrackerVSNI/

cp ./data/haarcascades/haarcascade_frontalface_alt.xml ${HOME}/.vs_core/plugins/skeletonTrackerVSNI/

cp ./data/lbpcascades/lbpcascade_frontalface.xml ${HOME}/.vs_core/plugins/skeletonTrackerVSNI/

#cp ./data/haarcascades/haarcascade_eye_tree_eyeglasses.xml ${HOME}/.vs_core/plugins/skeletonTrackerVSNI/

#cp ./data/haarcascades/haarcascade_mcs_mouth.xml ${HOME}/.vs_core/plugins/skeletonTrackerVSNI/

cp ./data/UserCalibration.bin ${HOME}/.vs_core/plugins/skeletonTrackerVSNI/

sudo cp ./data/FeatureExtraction.ini /usr/etc/primesense/Features_1_5_2/
