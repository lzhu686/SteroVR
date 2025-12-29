#!/bin/bash
# USB双目立体相机完整系统启动器
# 同时启动WebSocket服务器和HTTPS Web服务器

echo "🎥 ========================================================"
echo "   USB双目立体相机 WebSocket + HTTPS 系统"
echo "   ========================================================"
echo ""

# 检查必要文件
missing_files=()
if [ ! -f "usb_stereo_websocket_server.py" ]; then
    missing_files+=("usb_stereo_websocket_server.py")
fi
if [ ! -f "dual_infrared_viewer.html" ]; then
    missing_files+=("dual_infrared_viewer.html")
fi
if [ ! -f "dual_infrared_vr_viewer.html" ]; then
    missing_files+=("dual_infrared_vr_viewer.html")
fi
if [ ! -f "https_server.py" ]; then
    missing_files+=("https_server.py")
fi

if [ ${#missing_files[@]} -ne 0 ]; then
    echo "❌ 缺少必要文件:"
    for file in "${missing_files[@]}"; do
        echo "   - $file"
    done
    exit 1
fi

# 检查Python依赖
echo "🔍 检查依赖..."
python3 -c "import cv2, websockets, numpy" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "❌ 缺少Python依赖，正在安装..."
    pip3 install opencv-python websockets numpy
    if [ $? -ne 0 ]; then
        echo "❌ 依赖安装失败"
        exit 1
    fi
fi

echo "📷 相机检测将在服务器启动时自动进行..."

# 获取Python配置信息
echo "📋 读取相机配置..."
if [ -f "usb_stereo_websocket_server.py" ]; then
    config_info=$(python3 usb_stereo_websocket_server.py --get-config 2>/dev/null)
    if [ $? -eq 0 ]; then
        stereo_resolution=$(echo "$config_info" | head -1)
        camera_resolution=$(echo "$config_info" | tail -1)
    else
        # 默认值
        stereo_resolution="2560x720@15fps"
        camera_resolution="1280x720"
    fi
else
    stereo_resolution="2560x720@15fps"
    camera_resolution="1280x720"
fi

echo ""
echo "🚀 准备启动完整系统..."
echo "   WebSocket服务器: wss://localhost:8765 (SSL)"
echo "   HTTPS Web服务器: https://localhost:8443"
echo "   双目立体页面: https://localhost:8443/dual_infrared_viewer.html"
echo "   VR立体视觉: https://localhost:8443/dual_infrared_vr_viewer.html"
echo ""
echo "📋 相机配置:"
echo "   • USB双目相机: ${stereo_resolution} (自动分割为${camera_resolution}左右图像)"
echo "   • 图像格式: 原始RGB (无立体校正)"
echo "   • 传输格式: JPEG Base64编码"
echo ""
echo "📋 SSL证书说明:"
echo "   • HTTPS服务器启动时会自动生成SSL证书"
echo "   • WebSocket服务器会使用相同的证书文件"
echo "   • 首次访问时浏览器会提示证书不安全，请选择继续"
echo "   • Quest 3等VR设备需要HTTPS + WSS才能正常使用WebXR"
echo ""

# 创建启动函数
start_websocket() {
    echo "🔌 启动USB双目相机WebSocket服务器 (with SSL support)..."
    python3 usb_stereo_websocket_server.py --ssl
}

start_https() {
    echo "🌐 启动HTTPS服务器..."
    sleep 2  # 等待WebSocket服务器先启动
    python3 https_server.py
}

start_test_mode() {
    echo "🎯 启动测试模式WebSocket服务器..."
    python3 usb_stereo_websocket_server.py --ssl  # 现在默认使用设备0，不存在会自动启用测试模式
}

start_full_system() {
    echo "� 启动完整系统..."
    # 后台启动WebSocket服务器
    start_websocket &
    websocket_pid=$!
    
    # 等待一下再启动HTTPS服务器
    sleep 3
    
    # 前台启动HTTPS服务器
    start_https
}

# 清理函数
cleanup() {
    echo ""
    echo "🛑 正在停止服务器..."
    pkill -f usb_stereo_websocket_server.py
    pkill -f https_server.py
    echo "✅ 所有服务器已停止"
    exit 0
}

# 设置信号处理
trap cleanup SIGINT SIGTERM

# 询问启动方式
echo "选择启动方式:"
echo "1) 完整系统 (USB双目相机WebSocket + HTTPS服务器)"
echo "2) 仅USB双目相机WebSocket服务器"
echo "3) 仅HTTPS服务器"
echo "4) 测试模式 (无真实相机，生成虚拟图像)"
echo ""
read -p "请选择 [1/2/3/4]: " choice

case $choice in
    1)
        start_full_system
        ;;
    2)
        echo "🎯 启动USB双目相机WebSocket服务器..."
        start_websocket
        ;;
    3)
        echo "🎯 启动HTTPS服务器..."
        start_https
        ;;
    4)
        start_test_mode
        ;;
    *)
        echo "❌ 无效选择，默认启动完整系统..."
        start_full_system
        ;;
esac

# 清理后台进程
cleanup
