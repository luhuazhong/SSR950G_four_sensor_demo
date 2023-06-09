#$language = "python"
#$interface = "1.0"


import os
import random
import time
comlist =[]
#SCRIPT_TAB = crt.GetScriptTab()

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def main():
    fname=time.strftime("%Y%m%d_%H%M%S")
    sfname='send'+fname #接收log名称
    sfile=open(sfname,'w')

    if not os.path.exists('case.txt'):
        crt.Dialog.MessageBox("not exist")

    else:
        f = open("case.txt", "r")
        f2 = f.readlines()
        for lines in f2:
            comlist.append(lines.strip('\n'))


        result3 = crt.Dialog.Prompt("请选择顺序(0)执行和随机(1)执行：","xuanze","",False)
        result2 = crt.Dialog.Prompt("set sleeptime：","sleeptime","",False)
        result2=int(result2)
        if result3 == "0":
            result1 = crt.Dialog.Prompt("请设置循环次数：","looptimes","",False)
            result1=int(result1)

            for j in range(0,result1):
                for i in range(0,len(comlist)):
                    #input_s = (comlist[i]+'\r\n').encode('utf-8')
                    crt.Screen.Send(str(comlist[i]))
                    crt.Screen.Send("\r\n")
                    if (str(comlist[i]))=="q":
                        time.sleep(20)
                    else:
                        time.sleep(result2)
                    # outPut = crt.Screen.ReadString(["error","core dump","CMDQ","segmentation fault","panic","leak"],result2)

                    # #crt.Sleep(5000)
                    # index = crt.Screen.MatchIndex

                    # if index > 0:
                        # crt.Sleep(2000)
                        # crt.Dialog.MessageBox("system problem")
                        # i+=1
                j+=1

        elif result3 == "1":
            while(1):
                i = random.randint(0,len(comlist)-1)
                crt.Screen.Send(str(comlist[i]))
                sfile.write(time.strftime("%Y-%m-%d %H:%M:%S:")+str(comlist[i])+"\n")
                crt.Screen.Send("\r\n")
                time.sleep(result2)

                # outPut = crt.Screen.ReadString(["error","core dump","CMDQ","segmentation fault","panic","leak"],result2)

                # #crt.Sleep(5000)
                # index = crt.Screen.MatchIndex
                # if index > 0:
                    # crt.Sleep(2000)
                # crt.Dialog.MessageBox("system problem")

        else :
            crt.Dialog.MessageBox("输入错误或退出测试")

   # crt.Dialog.MessageBox("测试结束！")
        sfile.close()

main()