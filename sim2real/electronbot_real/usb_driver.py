#!/usr/bin/env python3
"""
ElectronBot USB CDC 直连驱动 (libusb)

替代闭源 USBInterface.lib，实现 Linux 下的 USB Bulk 通信。

协议 (从固件分析):
  VID: 0x1001, PID: 0x8023
  Endpoint OUT: 0x01 (发送 ExtraData 32字节)
  Endpoint IN:  0x81 (接收 JointAngles 24字节 + LCD 图像 172800字节)

ExtraData 格式 (32字节):
  [0]:     enable (uint8_t, 0=disable / 1=enable)
  [1-24]:  6 个 float (little-endian): j1..j6
  [25-31]: 保留

用法:
  driver = USBDriver()
  driver.open()
  driver.send_joint_angles([0, 10, 30, 5, 30, 5])
  angles = driver.receive_joint_angles()
  driver.close()
"""

import struct
import time
import numpy as np
from typing import Tuple, Optional


# USB 标识
USB_VID = 0x1001
USB_PID = 0x8023

# 端点
EP_OUT = 0x01
EP_IN = 0x81

# 数据尺寸
EXTRADATA_SIZE = 32
JOINT_ANGLES_PACKET = 28  # 实测
LCD_IMAGE_SIZE = 172800  # 240*240*3


class USBDriver:
    """ElectronBot USB CDC 驱动"""

    def __init__(self, vid: int = USB_VID, pid: int = USB_PID):
        self.vid = vid
        self.pid = pid
        self._handle = None

    def open(self) -> bool:
        """打开 USB 设备"""
        try:
            import usb.core
            import usb.util

            self._device = usb.core.find(idVendor=self.vid, idProduct=self.pid)

            if self._device is None:
                print(f"[ERROR] USB 设备未找到 (VID={self.vid:04x}, PID={self.pid:04x})")
                return False

            # 分离内核驱动
            if self._device.is_kernel_driver_active(0):
                self._device.detach_kernel_driver(0)

            # 设置配置
            self._device.set_configuration()

            # 获取端点
            cfg = self._device.get_active_configuration()
            intf = cfg[(0, 0)]

            self._ep_out = usb.util.find_descriptor(
                intf, custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT
            )
            self._ep_in = usb.util.find_descriptor(
                intf, custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN
            )

            print(f"[INFO] USB 已连接: {self._device.manufacturer} {self._device.product}")
            self._connected = True
            return True

        except ImportError:
            print("[WARN] pyusb 未安装: pip install pyusb")
            return False
        except Exception as e:
            print(f"[ERROR] USB 连接失败: {e}")
            return False

    def close(self):
        """关闭设备"""
        if self._device:
            import usb.util
            usb.util.dispose_resources(self._device)
        self._connected = False

    def is_connected(self) -> bool:
        return getattr(self, '_connected', False)

    def send_extra_data(self, enable: bool, joint_angles: np.ndarray) -> bool:
        """
        发送 ExtraData (32字节)

        参数:
          enable: 使能控制
          joint_angles: 6 个目标角度 (度, 机械角度)
        """
        buf = bytearray(EXTRADATA_SIZE)
        buf[0] = 1 if enable else 0

        # 6 个 float (little-endian)
        for i in range(6):
            val = float(joint_angles[i])
            struct.pack_into('<f', buf, 1 + i * 4, val)

        try:
            self._device.write(EP_OUT, bytes(buf), timeout=1000)
            return True
        except Exception as e:
            print(f"[ERROR] USB 发送失败: {e}")
            return False

    def receive_joint_angles(self) -> Optional[np.ndarray]:
        """
        接收当前关节角度 (实测 28 字节包)

        返回:
          6 维机械角度 (度)
        """
        try:
            data = self._device.read(EP_IN, JOINT_ANGLES_PACKET, timeout=1000)
            angles = struct.unpack('<6f', data[4:28])
            return np.array(angles, dtype=np.float64)
        except Exception as e:
            print(f"[ERROR] USB 接收失败: {e}")
            return None
