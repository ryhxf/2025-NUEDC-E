all_duo文件夹是舵机驱动库

detect文件夹是视觉+控制核心逻辑

SeekFree_MSPM0G3507_Opensource_Library.zip是MSPM0G3507单片机代码



gpio_control.service是RDK-X5板子的自启动文件，需嵌入至"/etc/systemd/system/gpio_control.service"路径

查看实时查看日志（自启动后，可新开终端查看）
```
sudo journalctl -u spiderpi_1.service -f
```

禁用开机自启动
```
sudo systemctl disable gpio_control.service
```

设置开机自启动
```
sudo systemctl enable gpio_control.service
```

临时启动
```
sudo systemctl start gpio_control.service
```

临时停止
```
sudo systemctl stop gpio_control.service
```




# B站视频讲解：
