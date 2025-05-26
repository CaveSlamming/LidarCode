- [cn](#操作指南)
- [en](#Instructions)
# 操作指南

>此SDK仅适用于youyeetoo销售的激光雷达产品，产品型号为:
> - youyeetoo LiDAR FHL-LD06
> - youyeetoo LiDAR FHL-LD19
## 0. 获取雷达的Linux SDK
```bash
cd ~

mkdir  ldlidar_ws

cd ldlidar_ws

//将ldlidar_stl_sdk.zip复制到本目录

unzip ldlidar_stl_sdk.zip

```

## 1. 系统设置
- 第一步，通过板载串口或者USB转串口模块(例如,cp2102模块)的方式使雷达连接到你的系统主板.
- 第二步，设置雷达在系统中挂载的串口设备-x权限(以/dev/ttyUSB0为例)
	- 实际使用时，根据雷达在你的系统中的实际挂载情况来设置，可以使用`ls -l /dev`命令查看.

``` bash
cd ~/ldlidar_ws/ldlidar_stl_sdk

sudo chmod 777 /dev/ttyUSB0
```

## 2. 编译

```bash
cd ~/ldlidar_ws/ldlidar_stl_sdk

./auto_build.sh
```

## 3. 运行
  ``` bash
  ./start_node.sh
  ```

# Instructions
> This SDK is only applicable to the LiDAR products sold by youyeetoo. The product models are :
> - youyeetoo LiDAR FHL-LD06
> - youyeetoo LiDAR FHL-LD19
## 0. get LiDAR Linux SDK
```bash
cd ~

mkdir  ldlidar_ws

cd ldlidar_ws

//Copy ldlidar_stl_sdk.zip to this directory

unzip ldlidar_stl_sdk.zip

```
## step 1: system setup
- Connect the LiDAR to your system motherboard via an onboard serial port or usB-to-serial module (for example, CP2102 module).

- Set the -x permission for the serial port device mounted by the radar in the system (for example, /dev/ttyUSB0)

  - In actual use, the LiDAR can be set according to the actual mounted status of your system, you can use 'ls -l /dev' command to view.

``` bash
cd ~/ldlidar_ws/ldlidar_stl_sdk

sudo chmod 777 /dev/ttyUSB0
```

## step 2: build

``` bash
cd ~/ldlidar_ws/ldlidar_stl_sdk

./auto_build.sh
```

## step 3: run

  ``` bash
  ./start_node.sh
  ```
