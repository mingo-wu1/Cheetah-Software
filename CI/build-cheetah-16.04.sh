#!/bin/bash
# Maintainer: wuchunming02@countrygarden.com.cn
# Function: build cheetah


mkdir -p deploy/deb
mkdir -p deploy/cheetah/opt/bdr/cheetah
cp -rf CI/DEBIAN deploy/cheetah/

###################
# install cheetah #
###################
# set the arg: architecture
machine=$(uname -m)
if [ ${machine} == "x86_64" ]; then
    architecture="amd64"
elif [ ${machine} == "aarch64" ]; then
    architecture="arm64"
fi

osVersion=$(cat /etc/issue | tr -cd '[0-9][.]')
osVersion=${osVersion:0:5}

sed -i '/Architecture: /d' `grep "Package: " -rl --include="control" deploy/`
sed -i "3a Architecture: ${architecture}" `grep "Package: " -rl --include="control" deploy/`

##########################
# build and pack cheetah #
##########################
ln -s python /usr/bin/python3
mkdir lcm/build
pushd lcm/build
cmake ..
make package
dpkg -i ./packages/lcm_1.4.0-1_amd64.deb
popd

export PATH=/opt/qt514/bin:$PATH
export LD_LIBRARY_PATH=/opt/qt514/lib:$LD_LIBRARY_PATH

mkdir build
pushd build
cmake -DCMAKE_INSTALL_PREFIX=/opt/bdr/cheetah ..
../scripts/make_types.sh
#cmake -DCMAKE_INSTALL_PREFIX=/opt/bdr/cheetah -DCMAKE_BUILD_TYPE=Release ..
make -j
make package
popd

#cp -r install/* lib/* include/* deploy/cheetah/opt/bdr/cheetah

#pushd deploy
#umask 0022
#sudo chmod -R 755 .
#sudo dpkg -b cheetah deb
