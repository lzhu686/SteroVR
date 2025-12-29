#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简单的HTTPS服务器用于提供HTML页面
自动生成SSL证书并启动HTTPS服务
"""

import http.server
import ssl
import socket
import subprocess
import os
import threading
import time
import webbrowser

class SimpleHTTPSServer:
    def __init__(self, port=8443, directory="."):
        self.port = port
        self.directory = os.path.abspath(directory)
        self.cert_file = "server.crt"
        self.key_file = "server.key"
        
    def generate_ssl_cert(self):
        """生成自签名SSL证书"""
        if os.path.exists(self.cert_file) and os.path.exists(self.key_file):
            print(f"✅ SSL证书已存在")
            return True
            
        print("🔐 生成SSL证书...")
        try:
            # 生成私钥和证书
            cmd = [
                'openssl', 'req', '-x509', '-newkey', 'rsa:2048',
                '-keyout', self.key_file, '-out', self.cert_file,
                '-days', '365', '-nodes',
                '-subj', '/C=CN/ST=Guangdong/L=Guangzhou/O=StereoVision/CN=localhost'
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode == 0:
                print("✅ SSL证书生成成功")
                return True
            else:
                print(f"❌ SSL证书生成失败: {result.stderr}")
                return False
        except FileNotFoundError:
            print("❌ 未找到openssl命令，请安装openssl")
            print("   Ubuntu/Debian: sudo apt install openssl")
            print("   使用HTTP服务器代替...")
            return False
        except Exception as e:
            print(f"❌ 生成SSL证书失败: {e}")
            return False
    
    def get_local_ip(self):
        """获取本地IP地址"""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
            s.close()
            return local_ip
        except:
            return "localhost"
    
    def start_server(self):
        """启动HTTPS/HTTP服务器"""
        # 切换到目标目录
        original_dir = os.getcwd()
        os.chdir(self.directory)
        
        try:
            # 创建HTTP服务器
            handler = http.server.SimpleHTTPRequestHandler
            httpd = http.server.HTTPServer(('0.0.0.0', self.port), handler)
            
            # 尝试添加SSL支持
            use_ssl = self.generate_ssl_cert()
            if use_ssl:
                context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
                context.load_cert_chain(self.cert_file, self.key_file)
                httpd.socket = context.wrap_socket(httpd.socket, server_side=True)
                protocol = "https"
            else:
                protocol = "http"
            
            local_ip = self.get_local_ip()
            
            print("🌐 HTTPS服务器启动成功!")
            print(f"📁 服务目录: {self.directory}")
            print(f"🔗 本地访问: {protocol}://localhost:{self.port}")
            print(f"🔗 网络访问: {protocol}://{local_ip}:{self.port}")
            print(f"🎥 双目相机页面: {protocol}://localhost:{self.port}/dual_infrared_viewer.html")
            print(f"🥽 VR立体视觉: {protocol}://localhost:{self.port}/dual_infrared_vr_viewer.html")
            print("🔥 按 Ctrl+C 停止服务器")
            print("=" * 60)
            
            # 自动打开浏览器
            def open_browser():
                time.sleep(2)  # 等待服务器启动
                url = f"{protocol}://localhost:{self.port}/dual_infrared_viewer.html"
                print(f"🚀 自动打开浏览器: {url}")
                try:
                    webbrowser.open(url)
                except:
                    print("⚠️  无法自动打开浏览器，请手动访问上述链接")
            
            # 在后台线程中打开浏览器
            browser_thread = threading.Thread(target=open_browser)
            browser_thread.daemon = True
            browser_thread.start()
            
            # 启动服务器
            httpd.serve_forever()
            
        except KeyboardInterrupt:
            print("\n🛑 服务器已停止")
        except Exception as e:
            print(f"❌ 服务器启动失败: {e}")
        finally:
            os.chdir(original_dir)

def main():
    """主函数"""
    print("🌐 启动双目RGB相机 HTTPS服务器")
    print("=" * 50)
    
    # 检查HTML文件是否存在
    html_files = ["dual_infrared_viewer.html", "dual_infrared_vr_viewer.html"]
    missing_files = [f for f in html_files if not os.path.exists(f)]
    
    if missing_files:
        print(f"❌ 未找到以下HTML文件: {', '.join(missing_files)}")
        print("   请在包含HTML文件的目录中运行此脚本")
        return
    
    # 启动服务器
    server = SimpleHTTPSServer(port=8443)
    server.start_server()

if __name__ == "__main__":
    main()
