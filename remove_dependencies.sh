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

./scripts_include/remove_eigen.sh $1
echo ""

./scripts_include/remove_rapidxml.sh $1
echo ""

./scripts_include/remove_sobogus.sh $1
echo ""
