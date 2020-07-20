#/!bin/bash
Date=$(expr $( date '+%Y_%m_%d_' ) )

Activate=Sigmoid
Mid=15
Result="Result/$Date"$Activate"_"$Mid
mkdir $Result
mkdir $Result"/log/"
mkdir $Result"/model/"

cp CNNmodel/$Activate"_mid"$Mid.py CNNmodel/DCNNAE.py
cp Saito_Train_DCNN.sh $Result
cp CNNmodel/DCNNAE.py $Result

python Train_DCNN.py \
--traindata "/home/assimilation/wang_workspace/1223train.npz" \
--testdata "/home/assimilation/wang_workspace/1223test.npz" \
--snap $Result"/model/" \
--log  $Result"/log/" \
--batchsize_train 53 \
--batchsize_test 53 \
--maxiter 50000 \
--testiter 1 \
--printiter 1 \
--inchannel 3 \
--imsize_x 128 \
--imsize_y 96 \
--decay 0.00001 \
--alpha 0.12 \
--beta1 0.999 \
--seed 1 \
--gpuid 0 \
--minval 0 \
--maxval 255 \
