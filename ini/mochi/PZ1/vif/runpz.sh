#!/bin/sh

vpeFilePath="./prog_vpe"

ergodicFunc()
{
    inipath=""

	  if [ ! -n "$1" ]
	  then
		file="4chn_normal"
	  elif [ $1 = 0 ]
	  then
		file="4chn_normal"
	  elif [ $1 = 1 ]
	  then
	    file="8chn_normal"
	  elif [ $1 = 2 ]
	  then
	    file="16ch_normal"
	  elif [ $1 = 3 ]
	  then
	    file="4chn_DI"
	  elif [ $1 = 4 ]
	  then
	    file="8chn_DI"
	  elif [ $1 = 5 ]
	  then
	    file="16chn_DI"
	  else
		file="4chn_normal"
	  fi
	  
	  #echo $1 $file
	  #echo `ls $file`

	for ini in `ls $file`  #./ini
	do
		
		#echo "${ini##*.}"
        if [ "${ini##*.}" = "ini" ]  #只运行ini文件
        then
          path=$file"/"$ini  #得到文件的完整的目录
          inipath=$inipath","$path
        fi

	done
    echo "inipath = $inipath"

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

echo vif_pad_mode 2 > /dev/vif
ergodicFunc $1



