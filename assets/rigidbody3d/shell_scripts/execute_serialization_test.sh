#!/bin/bash

trap ctrl_c INT

die()
{
  echo >&2 "$@"
  exit 1
}

isnumber() { test "$1" && printf '%f' "$1" >/dev/null; }
isnonnegative() { [ "1" = `echo "$1"'>='0 | bc -l` ]; }
integerregexp='^[0-9]+$'

# Ensure that the user provided the correct number of arguments
if [ "$#" -ne 4 ] && [ "$#" -ne 5 ]; then
  echo "Invalid number of arguments."
  echo "Usage:" $0 "xml_file_name end_time compare_frame_number resume_frame_number [output_frame_rate]"
  exit 1
fi

xml_file_name=$1
end_time=$2
compare_frame=$3
resume_frame=$4
output_frame_rate="0"
if [ "$#" -eq 5 ]; then
  output_frame_rate=$5
fi

# Ensure that the user provided valid arguments
[ -e "$xml_file_name" ] || die "Error, first argument must be a file name. Exiting."
isnumber "$end_time"
[ $? -eq 0 ] || die "Error, second argument must be a non-negative scalar. Exiting."
isnonnegative "$end_time"
[ $? -eq 0 ] || die "Error, second argument must be a non-negative scalar. Exiting."
if ! [[ "$compare_frame" =~ $integerregexp ]] ; then
  echo "Error, third argument must be a non-negative integer. Exiting." >&2; exit 1
fi
if ! [[ "$resume_frame" =~ $integerregexp ]] ; then
  echo "Error, fourth argument must be a non-negative integer. Exiting." >&2; exit 1
fi
isnumber "$output_frame_rate"
[ $? -eq 0 ] || die "Error, output_frame_rate argument must be a non-negative scalar. Exiting."
isnonnegative "$output_frame_rate"
[ $? -eq 0 ] || die "Error, output_frame_rate argument must be a non-negative scalar. Exiting."
[ "$compare_frame" -gt "$resume_frame" ] || die "Error, compare_frame_number must be greater than resume_frame_number"

output_directory=$(uuidgen)

# Clean up the temporary storage directory if the user forces an exit
function ctrl_c()
{
  echo "Cleaning up: rm -rf $output_directory"
  rm -rf $output_directory
  exit 1
}

# Ensure that the data storage directory does not exist
if [ -d $output_directory ]; then
  echo "Failed to execute test. Temporary output directory" $output_directory "already exists."
  exit 1
fi

# Create a directory to store test output
echo "Creating temporary output directory: mkdir $output_directory"
mkdir $output_directory
if [ $? -ne 0 ] ; then
  echo "Failed to create directory" $output_directory ". Exiting."
  exit 1
fi

# Run an initial, full simulation
echo "Executing full simulation run: ./rigidbody3d_cli $xml_file_name -s 1 -e $end_time -o $output_directory -f $output_frame_rate -i > /dev/null"
./rigidbody3d_cli $xml_file_name -s 1 -e $end_time -o $output_directory -f $output_frame_rate -i > /dev/null
if [ $? -ne 0 ] ; then
  echo "Failed to execute initial simulation run. Exiting."
  rm -rf $output_directory
  exit 1
fi

# Create a copy of the serial data for the given frame
echo "Copying serial data: cp $output_directory/serial_$compare_frame.bin $output_directory/serial_final_output.bin"
cp $output_directory/serial_$compare_frame.bin $output_directory/serial_final_output.bin
if [ $? -ne 0 ] ; then
  echo "Failed to backup serial output. Exiting."
  rm -rf $output_directory
  exit 1
fi
# Create a copy of the config data for the given frame
echo "Copying config data: cp $output_directory/config_$compare_frame.h5 $output_directory/config_final_output.h5"
cp $output_directory/config_$compare_frame.h5 $output_directory/config_final_output.h5
if [ $? -ne 0 ] ; then
  echo "Failed to backup config output. Exiting."
  rm -rf $output_directory
  exit 1
fi
# Create a copy of the force data for the given frame
echo "Copying force data: cp $output_directory/force_$compare_frame.h5 $output_directory/force_final_output.h5"
cp $output_directory/forces_$compare_frame.h5 $output_directory/forces_final_output.h5
if [ $? -ne 0 ] ; then
  echo "Failed to backup force output. Exiting."
  rm -rf $output_directory
  exit 1
fi

# Resume from an intermediate frame
echo "Resuming from intermediate state: ./rigidbody3d_cli -r $output_directory/serial_$resume_frame.bin > /dev/null"
./rigidbody3d_cli -r $output_directory/serial_$resume_frame.bin > /dev/null
if [ $? -ne 0 ] ; then
  echo "Failed to execute the resumed simulation. Exiting."
  rm -rf $output_directory
  exit 1
fi

# Check if the final serial files are identical
echo "Comparing final serial data: diff $output_directory/serial_final_output.bin $output_directory/serial_$compare_frame.bin"
diff $output_directory/serial_final_output.bin $output_directory/serial_$compare_frame.bin
diff_return_value_serial=$?
# Check if the final config files are identical
echo "Comparing final config data: h5diff $output_directory/config_final_output.h5 $output_directory/config_$compare_frame.h5"
h5diff $output_directory/config_final_output.h5 $output_directory/config_$compare_frame.h5
diff_return_value_config=$?
# Check if the final force files are identical
echo "Comparing final force data: h5diff $output_directory/forces_final_output.h5 $output_directory/forces_$compare_frame.h5"
#h5diff --exclude-path "/run_stats" $output_directory/forces_final_output.h5 $output_directory/forces_$compare_frame.h5
h5diff $output_directory/forces_final_output.h5 $output_directory/forces_$compare_frame.h5
diff_return_value_force=$?

# Clean up
echo "Cleaning up: rm -rf $output_directory"
rm -rf $output_directory
if [ $? -ne 0 ] ; then
  echo "Failed to clean up after test. Exiting."
  exit 1
fi

# Check the serial file
if [ $diff_return_value_serial -ne 0 ] ; then
  echo "Error, final serial files do not agree."
  exit $diff_return_value_serial
fi
# Check the config file
if [ $diff_return_value_config -ne 0 ] ; then
  echo "Error, final config files do not agree."
  exit $diff_return_value_config
fi
# Check the force file
if [ $diff_return_value_force -ne 0 ] ; then
  echo "Error, final force files do not agree."
  exit $diff_return_value_force
fi

# Exit with success
echo "Serialization test succeeded."
exit 0
