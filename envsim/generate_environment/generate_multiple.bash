#!/bin/bash

# Provide number of environments to be generated

if [ -z $1 ]
then
  N=10
else
  N=$1
fi

if [ -z $2 ]
then
  SINGLE_SPHERE_PARAM=1
else
  SINGLE_SPHERE_PARAM=$2
fi
#OLD EASY are 80
#GOOD PARAMS ARE: 140 sphere = 0.3
if [ -z $3 ]
then
  DENSITY_MULTIPLIER=1
else
  DENSITY_MULTIPLIER=$3
fi
rm -rf environment*

for i in $(eval echo {0..$N})
do
  dirname="environment_""$i"
  mkdir -p $dirname
  python3 obstacle_generator.py "$RANDOM" $SINGLE_SPHERE_PARAM $DENSITY_MULTIPLIER
  mv csvtrajs $dirname
  mv dynamic_obstacles.yaml $dirname
  mv static_obstacles.csv $dirname
done
