#!/bin/sh

vpeFilePath="./prog_vpe"

ergodicFunc()
{
    inipath=""
    for file in `ls $1`  #./ini
    do
      if [ "$file" = "vifut" ] || [ "$file" = "hdr_switch_to_linear" ] || [ "$file" = "HDR_mode" ] || [ "$file" = "other" ]
      then
        continue
      fi

      if [ -d $1"/"$file ] && [ $file != "stress" ]
      then
        ergodicFunc $1"/"$file   #./ini/a_ut
      else
      #echo "${file##*.}"
        if [ "${file##*.}" = "ini" ]  #只运行ini文件
        then
          path=$1"/"$file  #得到文件的完整的目录
          inipath=$inipath","$path
        fi
      fi
    done

    #echo "inipath = $inipath"

    if [[ "$inipath" != "" ]] #文件不为空，并且要求是ini文件
    then
      if [[ ${inipath:0:1} = "," ]]
      then
        #echo ",is capture"
        inipath=${inipath#*,}            #删除逗号
        #echo "after delete : $inipath"

        inipath=${inipath//,/ }         #逗号替换空格
        #echo "replace space : $inipath"
        echo "exec $vpeFilePath $inipath"

        $vpeFilePath $inipath
      fi
    fi
inipath=""
}

INIT_PATH="./ini"
ergodicFunc $INIT_PATH



