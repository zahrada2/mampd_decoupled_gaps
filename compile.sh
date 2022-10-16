#!/bin/bash
# an easy compile script for the mampd solver

#colors
RED='\033[0;31m'
NC='\033[0m' # no color
GREEN='\033[0;32m'

current_dir="$(dirname "$(readlink -e "${BASH_SOURCE[0]:-$0}")")"
build_folder=$current_dir/build
source_folder=$current_dir/src

mkdir -p $build_folder;
cd $build_folder;
cmake $source_folder;
make;
make_ret=$?

# echo ${FILE##*/}
# basename "$FILE"

executable_file="$(basename $(find $build_folder -maxdepth 1 -type f -executable -print))"
compiled_file=$build_folder/$executable_file
symlink_file=$current_dir/$executable_file

if  [ $make_ret -eq 0 ]; then
  printf "${GREEN}compilation ended, linking executable...${NC}\n"
  if [ -f "$compiled_file" ]; then
    if [ -f "$symlink_file" ]; then
        printf "${GREEN}symlink $symlink_file exists${NC}\n"
    else 
        ln -s $compiled_file $symlink_file
        printf "${GREEN}created a symlink at $symlink_file${NC}\n"
    fi
  else 
    printf "${RED}compiled file $compiled_file not found! No symlink created, but compilation may have succeeded.${NC}\n"
  fi
fi
