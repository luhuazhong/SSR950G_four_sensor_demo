#给stress_common.sh的功能测试集合
current_path=$(dirname $(readlink -f $0))


#指定是否输出到log文件
# export log_file="log.txt"				

# echo "" > $log_file

#source cfg file
export cfg_path=./ini/mochi/mochi_performance_config.sh

source $cfg_path

# mercury6 case
# cpu loading
# $1:cnt $2: run_cmd
standard_test()
{
# cpu loading
#$current_path/stress_common.sh -c -n $1 -f "$2"
# bandwidth
#$current_path/stress_common.sh -b -n $1 -f "$2"
# serial log
#$current_path/stress_common.sh -l 60 -n $1 -f "$2"
# kill -9
#$current_path/stress_common.sh -k -n $1 -f "$2"
# cpu + bandwidth + serial log + kill -9
#$current_path/stress_common.sh -c -b -k -l 60 -n $1 -f "$2"
#isr ctrl
$current_path/stress_common.sh -l 3600 -n $1 -f "$2"
}

if [ "$chip" == "mercury6p" ];then
standard_test 1 "./runall_stress.sh"
fi
