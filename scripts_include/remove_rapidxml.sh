#!/bin/bash

read -p "Warning, this will remove include/rapidxml. Do you wish to remove include/rapidxml (y/n)? " yn
case $yn in
    [Nn]* ) echo "RapidXml was not removed." && exit;;
    [Yy]* ) echo "Removing RapidXml.";;
    * ) echo "Invalid input, RapidXml was not removed." && exit;;
esac

rm -fr include/rapidxml
echo "RapidXml removed."
