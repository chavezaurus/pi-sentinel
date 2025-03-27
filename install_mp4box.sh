#!/bin/bash

git clone https://github.com/gpac/gpac.git

cd gpac
./configure --use-zlib=no --static-bin
make -j4
sudo make install

cd ..
rm -rf gpac
