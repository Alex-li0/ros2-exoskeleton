#!/bin/bash
# 将 Linux 内核串口延迟从 ~16ms 压低至 ~1ms
# 用法: sudo ./set_low_latency.sh [设备路径]
# 建议加入 udev 规则或 systemd service 自动执行

DEV="${1:-/dev/ttyUSB0}"

if [ ! -e "$DEV" ]; then
  echo "错误: 设备 $DEV 不存在"
  exit 1
fi

setserial "$DEV" low_latency
echo "已设置 $DEV 为 low_latency 模式"
setserial -g "$DEV"
