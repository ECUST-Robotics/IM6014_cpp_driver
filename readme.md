## 编译方式
```bash
cd rc/IM6014资料包/c++源码
g++ -std=c++17 -Wall -Wextra main.cpp im6014_motor.cpp -o im6014_demo
```
## 运行
```bash
./im6014_demo
```
## 偏移量设置
由于 宇树im6014电机 采用的是绝对编码器，无法根据实际装配设置绝对零点
因此采用程序当中添加 偏移量 的方法，可在`im6014_motor.cpp`脚本当中做修改
比如你装配后发现 /dev/ttyUSB3 的 id=1 需要把当前绝对编码器位置 2.35 rad 当成新零点，就改成：
```py
case 1: return 2.35;
```