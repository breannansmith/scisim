#!/bin/bash

original_sobogus_md5="fd502f6a24be313ea38410093e8df36d"
patched_sobogus_md5="a824f74df64ed5592acfd322a9b53497"
sobogus_url="https://bitbucket.org/gdaviet/so-bogus.git"
desired_sobogus_revision="fd97f98c41c331e28870775ecf2802dfea57c3a7"
# md5 on installed So-bogus source files
actual_installed_sobogus_dir_md5="6e13d2b085f6893e75151d2a611ce8fd"
actual_installed_hpp_md5="1bcfc6b838e0b84652c66e02550df5d3"
actual_installed_cpp_md5="ce26cfc11cb73e8c156f61cca487c11b"
actual_installed_impl_md5="b94ef20cd897ba9c4954f8f77c0fcbd5"

# Verify that git is installed
command -v git >/dev/null 2>&1 || { echo >&2 "Error, please install git and rerun get_sobogus.sh."; exit 1; }

# Verify that md5sum is installed
command -v md5sum >/dev/null 2>&1 || { echo >&2 "Error, please install md5sum and rerun get_sobogus.sh."; exit 1; }

# TODO: Run a checksum on the existing source files to see if they are ok

# If the output directory or interface scripts exist
if [ -d "include/sobogus" ] || [ -e "scisim/ConstrainedMaps/bogus/FrictionProblem.hpp" ] || [ -e "scisim/ConstrainedMaps/bogus/FrictionProblem.cpp" ] || [ -e "scisim/ConstrainedMaps/bogus/FrictionProblem.impl.hpp" ]; then
  # If the So-bogus install is up to date
  computed_installed_sobogus_dir_md5=`find include/sobogus -type f -name '*.[hc]pp' -exec md5sum {} + | awk '{print $2$1}' | sort -fd | md5sum | cut -c -32`
  computed_installed_hpp_md5=`md5sum scisim/ConstrainedMaps/bogus/FrictionProblem.hpp | cut -c -32`
  computed_installed_cpp_md5=`md5sum scisim/ConstrainedMaps/bogus/FrictionProblem.cpp | cut -c -32`
  computed_installed_impl_md5=`md5sum scisim/ConstrainedMaps/bogus/FrictionProblem.impl.hpp | cut -c -32`
  if [ "$computed_installed_sobogus_dir_md5" == "$actual_installed_sobogus_dir_md5" ] && [ "$computed_installed_hpp_md5" == "$actual_installed_hpp_md5" ] && [ "$computed_installed_cpp_md5" == "$actual_installed_cpp_md5" ] && [ "$computed_installed_impl_md5" == "$actual_installed_impl_md5" ]
  then
    echo "So-bogus library is already up to date, no further action is needed."
    exit 0
  fi
  # Warn the user and exit
  echo "Error, So-bogus source exists, please manually delete include/sobogus, scisim/ConstrainedMaps/bogus/FrictionProblem.hpp, scisim/ConstrainedMaps/bogus/FrictionProblem.cpp, and scisim/ConstrainedMaps/bogus/FrictionProblem.impl.hpp and rerun get_sobogus.sh."
  exit 1
fi

echo "Installing So-bogus"

# Create a temporary working directory
temp_dir_name=`uuidgen`
if [ -d "$temp_dir_name" ]; then
  echo "Error, temporary working directory $temp_dir_name exists, this is a bug. Please contact the maintainer."
  exit 1
fi
echo "--->  Creating temporary directory $temp_dir_name"
mkdir $temp_dir_name

function cleanup {
  echo "--->  Removing temporary directory $temp_dir_name"
  rm -fr "$temp_dir_name"
}
trap cleanup EXIT

# Download So-bogus
echo "--->  Downloading So-bogus source"
git clone $sobogus_url $temp_dir_name/so_bogus --quiet
cd $temp_dir_name/so_bogus
# Simpler, but requires git 1.8 (servers on 1.7)
#git reset --hard a0710cf -C $temp_dir_name/so_bogus
git reset --hard $desired_sobogus_revision --quiet
cd - > /dev/null

# Run a checksum on the download
echo "--->  Verifying So-bogus checksum"
computed_original_sobogus_md5=`find $temp_dir_name/so_bogus/src -type f -name '*.[hc]pp' -exec md5sum {} + | awk '{print $2$1}' | while read inputline; do echo "${inputline#*/}"; done | sort -fd | md5sum | cut -c -32`
if [ "$original_sobogus_md5" != "$computed_original_sobogus_md5" ]
then
  echo "Error, md5 checksum for So-bogus of $computed_original_sobogus_md5 is not $original_sobogus_md5."
fi

# Patch the So-bogus source
patch -d $temp_dir_name/so_bogus/src -p1 --quiet < scripts_include/sobogus_patch.patch

# Run a checksum on the patched download
echo "--->  Verifying patched So-bogus checksum"
computed_patched_sobogus_md5=`find $temp_dir_name/so_bogus/src -type f -name '*.[hc]pp' -exec md5sum {} + | awk '{print $2$1}' | while read inputline; do echo "${inputline#*/}"; done | sort -fd | md5sum | cut -c -32`
if [ "$patched_sobogus_md5" != "$computed_patched_sobogus_md5" ]
then
  echo "Error, patched md5 checksum for So-bogus of $computed_patched_sobogus_md5 is not $patched_sobogus_md5."
fi

# Move the source files to their final destinations
echo "--->  Moving So-bogus to destinations"
mkdir -p include/sobogus
mkdir -p scisim/ConstrainedMaps/bogus
cp -r $temp_dir_name/so_bogus/src/Core include/sobogus
cp -r $temp_dir_name/so_bogus/src/Extra include/sobogus
cp $temp_dir_name/so_bogus/src/Interfaces/FrictionProblem.hpp scisim/ConstrainedMaps/bogus
cp $temp_dir_name/so_bogus/src/Interfaces/FrictionProblem.cpp scisim/ConstrainedMaps/bogus
cp $temp_dir_name/so_bogus/src/Interfaces/FrictionProblem.impl.hpp scisim/ConstrainedMaps/bogus

trap - EXIT
cleanup
echo "Successfully installed So-bogus"
