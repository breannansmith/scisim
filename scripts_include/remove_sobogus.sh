#!/bin/bash

die()
{
  echo >&2 "$@"
  echo "Usage: $0 [-f]"
  exit 1
}

[ $# -le 1 ] || die "Invalid number of arguments provided."

if [ $# -eq 1 ] && [ "$1" != "-f" ]; then
  die "Invalid argument provided."
fi

prompt_user=true
if [ "$1" = "-f" ]; then
  prompt_user=false
fi

if [ "$prompt_user" = true ]; then
  read -p "Warning, this will remove include/sobogus, scisim/ConstrainedMaps/bogus/FrictionProblem.hpp, scisim/ConstrainedMaps/bogus/FrictionProblem.cpp, and scisim/ConstrainedMaps/bogus/FrictionProblem.impl.hpp. Do you wish to remove So-bogus (y/n)? " yn
  case $yn in
    [Nn]* ) echo "So-bogus was not removed." && exit;;
    [Yy]* ) echo "Removing So-bogus.";;
    * ) echo "Invalid input, So-bogus was not removed." && exit;;
  esac
fi

rm -fr include/sobogus scisim/ConstrainedMaps/bogus/FrictionProblem.hpp scisim/ConstrainedMaps/bogus/FrictionProblem.cpp scisim/ConstrainedMaps/bogus/FrictionProblem.impl.hpp
echo "So-bogus removed."
