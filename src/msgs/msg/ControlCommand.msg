std_msgs/Header header

## 控制命令的编号 防止接收到错误命令， 编号应该逐次递加
# uint32 command_id

## 消息来源
string source
## 控制命令的模式 
uint8 mode
# enum Mode 控制模式枚举
uint8 idle=0
uint8 takeoff=1
uint8 hold=2
uint8 land=3
uint8 move=4
uint8 disarm=5
uint8 user_Mode1=6
uint8 user_Mode2=7

## 控制参考量 
## 位置参考量：位置、速度、加速度、加加速度、加加加速度
## 角度参考量：偏航角、偏航角速度、偏航角加速度
PositionReference reference_state