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
    read -p "Warning, this will remove include/eigen. Do you wish to remove include/eigen (y/n)? " yn
    case $yn in
        [Nn]* ) echo "Eigen was not removed." && exit;;
        [Yy]* ) echo "Removing Eigen.";;
        * ) echo "Invalid input, Eigen was not removed." && exit;;
    esac
fi

rm -fr include/eigen
echo "Eigen removed."
