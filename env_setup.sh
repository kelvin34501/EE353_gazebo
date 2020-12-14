#!/usr/bin/env bash

echo "Getting build directory ..."
build_dir=$(realpath `dirname "$BASH_SOURCE"`/build)
echo $build_dir

if [[ $GAZEBO_PLUGIN_PATH == *"$build_dir"* ]]; then
  echo "Already contains: $build_dir"
else 
  echo "Append: $build_dir"
  export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$build_dir
fi

echo "Current GAZEBO_PLUGIN_PATH: $GAZEBO_PLUGIN_PATH"

echo "Getting model directory ..."
model_dir=$(realpath `dirname "$BASH_SOURCE"`/model)
echo $model_dir

if [[ $GAZEBO_MODEL_PATH == *"$model_dir"* ]]; then
  echo "Already contains: $model_dir"
else 
  echo "Append: $model_dir"
  export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$model_dir
fi

echo "Current GAZEBO_MODEL_PATH: $GAZEBO_MODEL_PATH"
