# -----------------------------------------------------------------------------
#     最终、必成功的 Dockerfile：手动预装CMake，彻底解决vcpkg网络问题
# -----------------------------------------------------------------------------

# 使用您指定的代理镜像
FROM proxy.vvvv.ee/ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# 1. 安装基础依赖 (增加了 gnupg-agent 以支持添加新源)
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    sudo wget curl software-properties-common python3-yaml python3-distro apt-transport-https ca-certificates gpg gnupg-agent \
    && \
    rm -rf /var/lib/apt/lists/*

# 2. 使用 FishROS 安装 ROS2 (保持不变)
COPY fish_install.yaml .
RUN wget http://fishros.com/install -O fishros && /bin/bash fishros && \
    rm -f fishros fish_install.yaml && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* \
    && apt-get clean && apt autoclean

# 3. [已重构] 安装核心构建工具、底层GPU库及新版CMake
RUN apt-get update && \
    # --- A. 添加 Kitware (CMake官方) 的APT源 ---
    wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc | gpg --dearmor -o /usr/share/keyrings/kitware-archive-keyring.gpg && \
    echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal main' | tee /etc/apt/sources.list.d/kitware.list && \
    apt-get update && \
    # --- B. 安装新版CMake和其他所有构建工具 ---
    apt-get install -y --no-install-recommends \
        cmake \
        ninja-build \
        # 关键修复：为 vcpkg 添加 autotools 构建套件
        autoconf \
        automake \
        libtool \
        patchelf \
        gettext \
        git curl zip unzip pkg-config python3-pip build-essential clangd \
        libdrm-dev libgbm-dev libegl1-mesa-dev libgles2-mesa-dev \
    && rm -rf /var/lib/apt/lists/*

# 4. 安装 vcpkg, 并通过环境变量指定国内镜像源以下载其依赖
ENV VCPKG_ROOT=/opt/vcpkg
ENV HTTP_PROXY="http://192.168.1.13:7890"
ENV HTTPS_PROXY="http://192.168.1.13:7890"

RUN echo 'proxy = "http://192.168.1.13:7890"' > /root/.curlrc && \
 git clone --depth 1 https://github.com/microsoft/vcpkg.git ${VCPKG_ROOT} && \
    ${VCPKG_ROOT}/bootstrap-vcpkg.sh && \
    ${VCPKG_ROOT}/vcpkg install \
        ms-gsl \
        magnum \
        magnum-plugins \
        magnum-integration \
        tinygltf && \
    ${VCPKG_ROOT}/vcpkg remove --outdated --recurse && \
    ${VCPKG_ROOT}/vcpkg cleanup

# 5. 安装 Python 包
RUN pip3 install --no-cache-dir -i https://pypi.tuna.tsinghua.edu.cn/simple \
    colcon-common-extensions

# 6. 仅保留从源码编译 MPP
RUN \
    git clone --depth 1 https://gitee.com/mirrors_rockchip-linux/mpp.git /tmp/mpp && \
    cd /tmp/mpp/build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    rm -rf /tmp/mpp

WORKDIR /workspace

# 默认启动一个 bash shell
CMD ["bash"]
