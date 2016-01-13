#!/bin/bash

die()
{
  echo >&2 "$@"
  exit 1
}

isnumber() { test "$1" && printf '%f' "$1" >/dev/null; }

integerregexp='^[0-9]+$'

# Ensure that the user provided the correct number of arguments
if [ "$#" -ne 3 ]; then
  echo "Invalid number of arguments."
  echo "Usage:" $0 "xml_file_name end_time output_frequency"
  exit 1
fi

# Ensure that the user provided valid arguments
[ -e "$1" ] || die "Error, first argument must be a file name. Exiting."
isnumber "$2"
[ $? -eq 0 ] || die "Error, second argument must be a scalar. Exiting." # TODO: Check if this argument is positive
isnumber "$3"
[ $? -eq 0 ] || die "Error, third argument must be a scalar. Exiting." # TODO: Check if this argument is positive

xml_file_name=$1
end_time=$2
output_frequency=$3

output_directory=$(uuidgen)

# Ensure that the data storage directory does not exist
if [ -d $output_directory ]; then
  echo "Failed to execute test. Temporary output directory" $output_directory "already exists."
  exit 1
fi

# Check whether the proper Python modules are installed
python -c "import h5py" 2> /dev/null
if [ $? -ne 0 ] ; then
  echo "Error, static particle test requires the 'h5py' Python module."
  exit 1
fi

# Create a directory to store test output
mkdir $output_directory
if [ $? -ne 0 ] ; then
  echo "Failed to create directory" $output_directory ". Exiting."
  exit 1
fi

# Run an initial, full simulation
./rigidbody2d_cli $xml_file_name -e $end_time -o $output_directory -f $output_frequency > /dev/null
if [ $? -ne 0 ] ; then
  echo "Failed to execute initial simulation run. Exiting."
  rm -rf $output_directory
  exit 1
fi

# For each output file
for output_state_file in $output_directory/*.h5
do
  python assets/shell_scripts/static_particle_test.py -i $output_state_file
  if [ $? -ne 0 ] ; then
    echo "State of file $output_state_file appears incorrect"
    rm -rf $output_directory
    exit 1
  fi
done

# Clean up
rm -rf $output_directory
if [ $? -ne 0 ] ; then
  echo "Failed to clean up after test. Exiting."
  exit 1
fi

exit 0
