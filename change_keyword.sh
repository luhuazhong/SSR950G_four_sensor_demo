#!/bin/sh

#change xxxOutPutPath的路径。
func(){
  for file in `ls $1`
  do
    if [ -d $1"/"$file ] && [ "$file" != "other" ]
    then
      func $1"/"$file
    else
      if [ $file != "dynamic_change.ini" ] && [ "${file##*.}" = "ini" ]
      then
        path=${1:2}
        #echo "-1: $path"
        path=${path#*/}
        #echo "0:$path"
        path="./out/"$path
        #echo "2: $path"
        inipath=$1"/"$file
        echo "outputpath = $path , inipath = $inipath"

        #1. 修改outputpath
        #sed -i "s|[oO]ut[pP]ut[pP]ath.*|OutPutPath=\"${path}\"|g" $inipath

        timeoutexit=$(awk -F "=" '/ExitTimeOutS/' $inipath)
        echo -e "timeoutexit= $timeoutexit\n"

        #2. 向base.ini文件添加ExitTimeOuts参数。
        if [ "${#timeoutexit}" -eq 0 ]
        then
          sed -i "1iExitTimeOutS=2" $inipath
        fi

        #3. 删除ExitTimeOutS这一行
        #sed -i "/[eE]xit[tT]ime[oO]ut[sS]/d" $inipath
      fi

    fi
  done
}
INIT_PATH="./ini"
func $INIT_PATH