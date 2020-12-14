#!/usr/bin/env bash

echo "Getting build directory ..."
build_dir=$(realpath `dirname "$BASH_SOURCE"`/build)
echo $build_dir

if [[ $GAZEBO_PLUGIN_PATH == *"$build_dir"* ]]; then
  echo "Already contains: $build_dir"
else 
  echo "Append: $build_dir"
  export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$(realpath `dirname "$BASH_SOURCE"`/build)
fi

echo "Current GAZEBO_PLUGIN_PATH: $GAZEBO_PLUGIN_PATH"