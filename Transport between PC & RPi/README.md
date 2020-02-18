# Transport-betweent-PC-RP
In RP.py, this script sends several integers, which start from 0 then increase by 1 each second.
In PC.py, it recieves these integers and these integers are divided by 2. Then the outcomes are sent to RP.py.
In Motor_UDP_Controll.py, user can send current signal to 4 motors in GUI. The infomation of motors, including Rotation Angle, Rpm and Torque is shown in windows.
In RPi_UDP_Receiver.py, it receives commands from PC, then sends the commands to Motors. The info of motors' encoders is sent back to PC.
