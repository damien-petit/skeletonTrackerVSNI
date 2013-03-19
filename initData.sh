#!/bin/bash

cp ./data/dataToLoadOpenCVFile.yml ${HOME}/.vs_core/plugins/bci-self-interact-vision/


cp ./data/bci-self-interact-vision.conf ${HOME}/.vs_core/plugins/bci-self-interact-vision/

cp ./data/haarcascades/haarcascade_frontalface_alt.xml ${HOME}/.vs_core/plugins/bci-self-interact-vision/

cp ./data/lbpcascades/lbpcascade_frontalface.xml ${HOME}/.vs_core/plugins/bci-self-interact-vision/

#cp ./data/haarcascades/haarcascade_eye_tree_eyeglasses.xml ${HOME}/.vs_core/plugins/bci-self-interact-vision/

#cp ./data/haarcascades/haarcascade_mcs_mouth.xml ${HOME}/.vs_core/plugins/bci-self-interact-vision/

cp ./data/UserCalibration.bin ${HOME}/.vs_core/plugins/bci-self-interact-vision/

sudo cp ./data/FeatureExtraction.ini /usr/etc/primesense/Features_1_5_2/
