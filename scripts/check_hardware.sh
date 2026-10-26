#!/usr/bin/env bash
# Nero 部署硬件体检: 相机 USB 速率/省电、CAN 状态、电池余量提示
# Usage: bash scripts/check_hardware.sh
# 判定: Orbbec 相机必须 speed=5000+ 且 control=on; can0 必须 UP

echo "===== 1. USB 相机 (Orbbec VID=2bc5) ====="
# usbcore.autosuspend=-1 (modprobe.d 全局关闭) 时, control=auto 也不会真挂起
global_as=$(cat /sys/module/usbcore/parameters/autosuspend 2>/dev/null)
[ "$global_as" = "-1" ] && echo "  (全局 usbcore autosuspend=-1: USB 自动挂起已内核级关闭)"
for d in /sys/bus/usb/devices/*/idVendor; do
    v=$(cat "$d" 2>/dev/null)
    if [ "$v" = "2bc5" ]; then
        dev=$(dirname "$d")
        speed=$(cat "$dev/speed" 2>/dev/null)
        control=$(cat "$dev/power/control" 2>/dev/null)
        verdict="OK"
        [ "${speed:-0}" -lt 5000 ] && verdict="!! 速度不足 (应插USB3口/换USB3线/避开USB2 Hub)"
        if [ "$control" != "on" ] && [ "$global_as" != "-1" ]; then
            verdict="$verdict !! 省电未关 (重插/跑手动置on命令/或配 modprobe.d 全局关闭)"
        fi
        echo "  $(cat "$dev/product" 2>/dev/null): ${speed}M control=${control} -> ${verdict}"
    fi
done

echo "===== 2. CAN 总线 ====="
if ip link show can0 &>/dev/null; then
    state=$(ip -br link show can0 | awk '{print $2}')
    echo "  can0: ${state}"
    [ "$state" = "UP" ] || echo "  -> !! DOWN: sudo ip link set can0 up type can bitrate 1000000"
    echo "  CAN 适配器: $(lsusb | grep -c 1d50:606f) 个 (期望 1)"
else
    echo "  !! can0 不存在 (适配器未插好或驱动未加载: sudo modprobe gs_usb)"
fi

echo "===== 3. XBox 手柄 ====="
lsusb | grep -i "045e:0b12" >/dev/null && echo "  已连接" || echo "  !! 未发现"

echo "===== 4. GPU ====="
nvidia-smi --query-gpu=name,memory.total,memory.used --format=csv,noheader 2>/dev/null || echo "  !! nvidia-smi 不可用"

echo "===== 5. USB 树 (人工核对相机行末尾速度) ====="
lsusb -t
