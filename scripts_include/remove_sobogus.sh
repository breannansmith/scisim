#!/bin/bash

read -p "Warning, this will remove include/sobogus, scisim/ConstrainedMaps/bogus/FrictionProblem.hpp, scisim/ConstrainedMaps/bogus/FrictionProblem.cpp, and scisim/ConstrainedMaps/bogus/FrictionProblem.impl.hpp. Do you wish to remove So-bogus (y/n)? " yn
case $yn in
    [Nn]* ) echo "So-bogus was not removed." && exit;;
    [Yy]* ) echo "Removing So-bogus.";;
    * ) echo "Invalid input, So-bogus was not removed." && exit;;
esac

rm -fr include/sobogus scisim/ConstrainedMaps/bogus/FrictionProblem.hpp scisim/ConstrainedMaps/bogus/FrictionProblem.cpp scisim/ConstrainedMaps/bogus/FrictionProblem.impl.hpp
echo "So-bogus removed."
