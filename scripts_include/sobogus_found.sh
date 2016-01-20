#!/bin/bash

installed_sobogus_dir_md5="1a7ef59339952ab4f23a229abf80c2e6"
installed_hpp_md5="5f6590e7e6ddb278f6e7740c3e097976"
installed_cpp_md5="14c5475bbf334d29c56450cfc5363fd7"
installed_impl_md5="b68880eda45bafba666d7f977c4efc7d"

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
