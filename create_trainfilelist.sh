# /usr/bin/env sh
DATA=/home/rm/caffe/examples/mydatabase/number_train #DATA为训练集的路径
echo "Create train.txt..."
rm -rf $DATA/train.txt
find $DATA -name Nin*.jpg | cut -d '/' -f8 | sed "s/$/ 1/">>$DATA/train.txt
find $DATA -name Si*.jpg  | cut -d '/' -f8 | sed "s/$/ 2/">>$DATA/train.txt

echo "Done"
 
