#!/bin/sh
func(){
  for file in `ls $1`
  do
    if [ -d $1"/"$file ]
    then
      if [ "$file" != "other" ]
      then
        func $1"/"$file
    fi
      #func $1"/"$file
    else
      #echo "begin: $1"
      tmp=${1:2}
      str=${tmp#*/}
      str="./out/"$str
      echo "after: $str"
      mkdir -p $str
      break;
    fi
  done
}
INIT_PATH="./ini"
func $INIT_PATH