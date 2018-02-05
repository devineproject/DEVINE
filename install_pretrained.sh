#!/bin/bash

cd guesswhat

mkdir data
mkdir data/img
mkdir data/img/raw
mkdir data/img/ft_vgg_img
mkdir data/img/ft_vgg_crop
mkdir out
mkdir out/oracle
mkdir out/guesser
mkdir out/qgen
mkdir out/looper

echo "######Fetching data######"

wget https://s3-us-west-2.amazonaws.com/guess-what/guesswhat.train.jsonl.gz -P data/
wget https://s3-us-west-2.amazonaws.com/guess-what/guesswhat.valid.jsonl.gz -P data/
wget https://s3-us-west-2.amazonaws.com/guess-what/guesswhat.test.jsonl.gz -P data/

wget www.florian-strub.com/github/ft_vgg_img.zip -P data/img
unzip data/img/ft_vgg_img.zip -d data/img

wget http://florian-strub.com/github/pretrained_models.tf1-3.zip
unzip pretrained_models.tf1-3.zip

cp out/dict.json data/dict.json

echo "######Installing python dependencies######"

pip install \
    nltk \
    tensorflow-gpu \
    tqdm \
    image
