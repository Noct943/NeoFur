# -----------------------------------------------------------------------------
#      Final Single-Stage Dockerfile for RK-Series ROS2 Jazzy & MPP Dev
#      (Corrected based on user's version and official MPP documentation)
# -----------------------------------------------------------------------------

# 使用您指定的代理镜像，解决网络问题
FROM proxy.vvvv.ee/ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# 1. 安装运行 FishROS 的基础依赖
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    sudo wget curl software-properties-common python3-yaml python3-distro && \
    rm -rf /var/lib/apt/lists/*

# 2. 安装 ROS2 Foxy 并配置国内源
COPY fish_install.yaml .
RUN wget http://fishros.com/install -O fishros && /bin/bash fishros && \
    rm -f fishros fish_install.yaml && \
    rm -rf /var/lib/apt/lists/*  /tmp/* /var/tmp/* \
    && apt-get clean && apt autoclean

# 3. 安装额外的开发工具和 MPP 运行时依赖
# [优化] 明确安装 cmake 和 build-essential，使 Dockerfile 更健壮，不依赖 fishros 的默认行为
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    git python3-pip cmake build-essential libdrm-dev clangd && \
    rm -rf /var/lib/apt/lists/*

# 4. 安装 Python 包
RUN pip3 install --no-cache-dir -i https://pypi.tuna.tsinghua.edu.cn/simple \
    colcon-common-extensions

# 5. 编译安装 MPP 和 GSL，并清理源码
RUN \
    # --- Build and Install MPP ---
    git clone --depth 1 https://gitee.com/mirrors_rockchip-linux/mpp.git /tmp/mpp && \
    cd /tmp/mpp && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    \
    # --- Install GSL (header only) ---
    git clone --depth 1 https://gitee.com/mirrors/GSL.git /tmp/GSL && \
    cd /tmp/GSL && \
    cp -r ./include/gsl /usr/local/include/ && \
    \
    # --- Cleanup ---
    rm -rf /tmp/mpp /tmp/GSL

# --------------------------------------------------------------------------
#     集成官方预编译的 librga 库 (for RK3588 / aarch64)
# --------------------------------------------------------------------------
# 假设 airockchip/librga 仓库被克隆到了 Docker 构建上下文的 librga/ 目录中
# 先创建 librga 子目录
RUN mkdir -p /usr/local/include/librga
# 将头文件复制到该子目录中
COPY librga/include /usr/local/include/librga

# 将 .so 动态库文件复制到标准的 lib 路径
COPY librga/libs/Linux/gcc-aarch64/librga.so /usr/local/lib/librga.so

# 刷新动态链接器缓存，让系统能够找到刚刚添加的 librga.so
RUN ldconfig

# 6. 配置环境并设置工作目录
COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

WORKDIR /workspace

# 默认启动一个 bash shell
CMD ["bash"]
