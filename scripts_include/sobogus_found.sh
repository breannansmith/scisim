#!/bin/bash

installed_sobogus_dir_md5="d0541b424d14aa99b69d710c87e6309d"
installed_hpp_md5="1b87f8666802292e68766095262b941d"
installed_cpp_md5="4ab02502b7bc3432e05a257ad252fa60"
installed_impl_md5="15a6292d5dcb9ac377dbbce539341d58"

# Check the installed headers
computed_installed_sobogus_dir_md5=`find include/sobogus -type f -name '*.[hc]pp' -exec md5sum {} + | awk '{print $2$1}' | sort -fd | md5sum | cut -c -32`
if [ "$computed_installed_sobogus_dir_md5" == "$installed_sobogus_dir_md5" ]
then
  exit 0
else
  exit 1
fi

# Check the local hpp file
computed_installed_hpp_md5=`md5sum scisim/ConstrainedMaps/bogus/FrictionProblem.hpp | cut -c -32`
if [ "$computed_installed_hpp_md5" == "$installed_hpp_md5" ]
then
  exit 0
else
  exit 1
fi

# Check the local cpp file
computed_installed_hpp_md5=`md5sum scisim/ConstrainedMaps/bogus/FrictionProblem.cpp | cut -c -32`
if [ "$computed_installed_cpp_md5" == "$installed_cpp_md5" ]
then
  exit 0
else
  exit 1
fi

# Check the local impl file
computed_installed_impl_md5=`md5sum scisim/ConstrainedMaps/bogus/FrictionProblem.impl.hpp | cut -c -32`
if [ "$computed_installed_impl_md5" == "$installed_impl_md5" ]
then
  exit 0
else
  exit 1
fi
