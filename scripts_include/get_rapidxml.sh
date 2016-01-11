#!/bin/bash

actual_rapidxml_zip_md5="7b4b42c9331c90aded23bb55dc725d6a"
rapidxml_url="http://downloads.sourceforge.net/project/rapidxml/rapidxml/rapidxml%201.13/rapidxml-1.13.zip"
rapidxml_file_name="rapidxml-1.13.zip"
extracted_rapidxml_name="rapidxml-1.13"
# md5 on installed RapidXml source files
actual_installed_rapidxml_md5="d698fa3e3547874766de5ba8cb2177c9"

# Verify that wget is installed
command -v wget >/dev/null 2>&1 || { echo >&2 "Error, please install wget and rerun get_rapidxml.sh."; exit 1; }

# Verify that md5sum is installed
command -v md5sum >/dev/null 2>&1 || { echo >&2 "Error, please install md5sum and rerun get_rapidxml.sh."; exit 1; }

# If the output directory exists
if [ -d "include/rapidxml" ]; then
  # If the RapidXml install is up to date
  computed_installed_rapidxml_md5=`find include/rapidxml -type f -name '*.hpp' -exec md5sum {} + | awk '{print $2$1}' | sort -fd | md5sum | cut -c -32`
  if [ "$computed_installed_rapidxml_md5" == "$actual_installed_rapidxml_md5" ]
  then
    echo "RapidXml library is already up to date, no further action is needed."
    exit 0
  fi
  # Warn the user and exit
  #
  echo "Error, directory include/rapidxml exists, please manually delete include/rapidxml and rerun get_rapidxml.sh."
  exit 1
fi

echo "Installing RapidXml"

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

# Download RapidXml
echo "--->  Downloading RapidXml source"
wget -q "$rapidxml_url" -P "$temp_dir_name"

# Run a checksum on the download
echo "--->  Verifying RapidXml checksum"
computed_rapidxml_zip_md5=`md5sum $temp_dir_name/$rapidxml_file_name | cut -c -32`
if [ "$actual_rapidxml_zip_md5" != "$computed_rapidxml_zip_md5" ]
then
  echo "Error, md5 checksum for $rapidxml_file_name does not match $actual_rapidxml_zip_md5."
  exit 1
fi

# Extract the zip archive
echo "--->  Extracting RapidXml"
unzip -q "$temp_dir_name"/"$rapidxml_file_name" -d "$temp_dir_name"
# Move the source to its final location
echo "--->  Moving RapidXml to destination"
mkdir -p include/rapidxml
mv $temp_dir_name/$extracted_rapidxml_name/*.hpp include/rapidxml/

trap - EXIT
cleanup
echo "Successfully installed RapidXml"
