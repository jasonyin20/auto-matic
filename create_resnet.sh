#!/usr/bin/env sh

DATA=/home/rm/caffe/examples/mydatabase
IMGDIGNAME=number_train
IMGLIST=number_train/train.txt
LMDBNAME=number_train_lmdb
echo "Create train lmdb..."
rm -rf $DATA/number_train/number_train_lmdb
/home/rm/caffe/build/tools/convert_imageset --shuffle=true \--resize_height=28 \--resize_width=28 \
$DATA/$IMGDIGNAME/ $DATA/$IMGLIST $DATA/$LMDNAME

echo "DONE"
 
