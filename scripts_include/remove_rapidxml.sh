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
    read -p "Warning, this will remove include/rapidxml. Do you wish to remove include/rapidxml (y/n)? " yn
    case $yn in
        [Nn]* ) echo "RapidXml was not removed." && exit;;
        [Yy]* ) echo "Removing RapidXml.";;
        * ) echo "Invalid input, RapidXml was not removed." && exit;;
    esac
fi

rm -fr include/rapidxml
echo "RapidXml removed."
