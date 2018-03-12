#!/bin/sh

cd guesswhat
export PYTHONPATH=$(pwd)/src:$PYTHONPATH

python src/guesswhat/eval/interactive_dialogue.py \
    -data_dir data \
    -img_dir data/img/ft_vgg_img \
    -crop_dir data/img/ft_vgg_crop \
    -exp_dir exp \
    -config config/eval/config.json \
    -networks_dir out \
    -qgen_identifier 867d59b933a89f4525b189da9d67f17b \
    -guesser_identifier e2c11b1757337d7969dc223c334756a9 \
    -dict_file dict.json # in data folder
