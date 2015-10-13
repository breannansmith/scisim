#!/bin/bash

read -p "Warning, this will remove include/eigen. Do you wish to remove include/eigen (y/n)? " yn
case $yn in
    [Nn]* ) echo "Eigen was not removed." && exit;;
    [Yy]* ) echo "Removing Eigen.";;
    * ) echo "Invalid input, Eigen was not removed." && exit;;
esac

rm -fr include/eigen
echo "Eigen removed."
