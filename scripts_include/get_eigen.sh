#!/bin/bash

actual_eigen_tar_md5="21a928f6e0f1c7f24b6f63ff823593f5"
eigen_url="http://bitbucket.org/eigen/eigen/get/3.2.5.tar.bz2"
eigen_file_name="3.2.5.tar.bz2"
extracted_eigen_name="eigen-eigen-bdd17ee3b1b3"
# md5 on installed Eigen source files
actual_installed_eigen_md5="2539903ddef842de07c5a6047922ee8b"

# Verify that wget is installed
command -v wget >/dev/null 2>&1 || { echo >&2 "Error, please install wget and rerun get_eigen.sh."; exit 1; }

# Verify that md5sum is installed
command -v md5sum >/dev/null 2>&1 || { echo >&2 "Error, please install md5sum and rerun get_eigen.sh."; exit 1; }

# If the output directory exists
if [ -d "include/eigen" ]; then
  # If the Eigen install is up to date
  computed_installed_eigen_md5=`find include/eigen -type f -name '*.h' -exec md5sum {} + | awk '{print $2$1}' | sort -fd | md5sum | cut -c -32`
  if [ "$computed_installed_eigen_md5" == "$actual_installed_eigen_md5" ]
  then
    echo "Eigen matrix library is already up to date, no further action is needed."
    exit 0
  fi
  # Otherwise, warn the user and exit
  echo "Error, directory include/eigen exists, please manually delete include/eigen manually and rerun get_eigen.sh."
  exit 1
fi

# Create a temporary working directory
temp_dir_name=`uuidgen`
if [ -d "$temp_dir_name" ]; then
  echo "Error, temporary working directory $temp_dir_name exists, this is a bug. Please contact the maintainer."
  exit 1
fi
echo "Creating temporary directory $temp_dir_name"
mkdir $temp_dir_name

function cleanup {
  echo "Removing temporary directory $temp_dir_name"
  rm -fr "$temp_dir_name"
}
trap cleanup EXIT

# Download Eigen
echo "Downloading Eigen source"
wget -q "$eigen_url" -P "$temp_dir_name"

# Run a checksum on the download
echo "Verifying Eigen checksum"
computed_eigen_tar_md5=`md5sum $temp_dir_name/$eigen_file_name | cut -c -32`
if [ "$actual_eigen_tar_md5" != "$computed_eigen_tar_md5" ]
then
  echo "Error, md5 checksum for $eigen_file_name does not match $actual_eigen_tar_md5."
fi

# Extract the tar archive
echo "Extracting Eigen"
tar -xf "$temp_dir_name"/"$eigen_file_name" -C "$temp_dir_name"
# Move the source to its final location
echo "Moving Eigen to destination"
mkdir -p include/eigen
mv $temp_dir_name/$extracted_eigen_name/Eigen include/eigen/
mv $temp_dir_name/$extracted_eigen_name/signature_of_eigen3_matrix_library include/eigen/

echo "Successfully installed Eigen"
