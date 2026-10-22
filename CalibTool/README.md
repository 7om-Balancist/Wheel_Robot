# CalibTool — 灰度标定工程

独立固件。主工程 `car7.15` 从 2026-09-17 起**不再现场采集**灰度，改成上电从外部
SPI flash 读回；**采集这一步搬到了本工程**。

## 怎么用

1. 在 CubeIDE 里构建本工程 → `Debug/CalibTool.bin`
2. 通过 Luby 的 U 盘模式烧进控制器（按住 BACK 开机 → 再按 BACK → 拷到 U 盘根目录 →
   按 OK 退出 → 选到 CalibTool 执行）
3. 上电后按屏幕提示做三步：
   - `GRAY CALIB 1/3` 车头压在擂台前边缘
   - `GRAY CALIB 2/3` 车尾压在擂台后边缘
   - `GRAY CALIB 3/3` 整车放到台下地面
4. 屏幕显示四个数后：**KEY1 存盘 / KEY2 重测**
5. 存盘后会做**回读验证**，显示 `VERIFY OK!` 才说明真的落盘了
6. 断电，换回主工程的 bin

> ⚠ **把屏幕上那四个数抄下来填进主工程的 `g_gray_fallback`**（`car7.15/Core/Src/calibrate.c`）。
> 主工程在 flash 读不到标定值时会回退到它，必须填实测值。

## 屏幕怎么看

| 显示 | 含义 |
|---|---|
| `SPI FLASH CHECK / ID BF2541 OK` | flash 认到了（`BF 25 41` 是 SST25VF016B） |
| `FAM:SST SR:xx` | 器件族别与状态寄存器原值 |
| `CRC32 SELF OK` | CRC 算法自检通过（不自检的 CRC 等于没有 CRC） |
| `FLASH NOT FOUND!` | 读不到 ID → 查 HOLD#(pin7) / 焊接 / 供电，不做无意义的 18 秒采集 |
| `SAVE RESULT / OK` | 写入成功 |
| `READBACK TEST / VERIFY OK!` | 回读比对一致 → 可以断电了 |

## 存储位置

写在外部 SPI flash 的**最后两个 4KB 扇区**，A/B 双槽交替：

```
0x1FF000  槽 A
0x1FE000  槽 B
```

保存时先擦**另一个**槽，所以任意时刻断电都还有一份完整的。地址定义在
`Core/Inc/cal_store.h` 的三个宏里，将来确认了安全扇区要迁移时只改那三行。

## ⚠ 用之前先跑 FlashTest

Luby 用户文档说控制器插 USB 会显示一个 **1M 大小的 U 盘**，而外挂 flash 正好 2MiB ——
**那个 U 盘很可能就建在同一颗芯片上**。在 `FlashTest` 的只读普查给出结果之前，
**不要用本工程往 flash 里写任何东西**。

（代码里已经有三层防线：扇区内容分类守卫 → 擦写窗口 → 逐字节前置断言。
 但"末尾 8KB 空闲"目前仍然只是有依据的猜测，不是结论。）

 **验证后结论** flash中bootloader与用户保存的bin文件都是从低地址开始写的，也就是说最后两个扇区是安全的（除非用户拷贝了过多bin文件进flash）

## 关于看门狗

本工程**不启动**自己的 IWDG。因为三步采集阻塞约 18.4 秒，而 IWDG 即使配到极限
（÷256 + RLR=4095）在最坏 LSI 60kHz 下也只有 17.48 秒 —— 物理上盖不住，是硬约束。
但**如果 bootloader 自己留了个狗**，`Wdg_ConfigMaxNoStart()` 会把它拉到最大窗口
（~17.5 秒）并重载一次；采样循环里也通过 `SST25_Port_WdgFeed()` 持续喂。
