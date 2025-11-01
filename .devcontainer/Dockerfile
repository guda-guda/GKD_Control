FROM ubuntu:22.04

# 临时关闭 SSL 验证，解决代理握手错误
RUN echo 'Acquire::https::Verify-Peer "false";' > /etc/apt/apt.conf.d/99insecure && \
    echo 'Acquire::https::Verify-Host "false";' >> /etc/apt/apt.conf.d/99insecure

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai
SHELL ["/bin/bash", "-c"]

# 强制使用清华源，绕过代理
RUN sed -i 's@http://archive.ubuntu.com/ubuntu@https://mirrors.tuna.tsinghua.edu.cn/ubuntu@g' /etc/apt/sources.list && \
    sed -i 's@http://security.ubuntu.com/ubuntu@https://mirrors.tuna.tsinghua.edu.cn/ubuntu@g' /etc/apt/sources.list && \
    apt-get clean && apt-get update --fix-missing && \
    apt-get install -y --no-install-recommends \
        git curl wget make cmake sudo build-essential \
        ca-certificates lsb-release software-properties-common gnupg zsh && \
    rm -rf /var/lib/apt/lists/*

# 安装 LLVM 21（使用清华源）
RUN curl -fsSL https://mirrors.tuna.tsinghua.edu.cn/llvm-apt/llvm.sh | \
    bash -s -- 21 -m https://mirrors.tuna.tsinghua.edu.cn/llvm-apt && \
    apt-get install -y clang-21 clang-tools-21 lld-21 lldb-21 libc++-21-dev libc++abi-21-dev && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

RUN update-alternatives --install /usr/bin/clang clang /usr/bin/clang-21 100 && \
    update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-21 100 && \
    update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-21 100

# 创建用户
ARG USERNAME=gkd
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN adduser --disabled-password --gecos "" --uid $USER_UID $USERNAME && \
    usermod -aG sudo $USERNAME

USER $USERNAME
WORKDIR /home/$USERNAME
ENV PATH="/home/${USERNAME}/.local/bin:${PATH}"

# 安装 xmake
RUN bash <(curl -fsSL https://xmake.io/shget.text)

# ===== 设置系统语言环境为 UTF-8 =====
USER root
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US.UTF-8 && \
    update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8
USER $USERNAME
# =======================================

# 安装 oh-my-zsh
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended && \
    sed -i 's/ZSH_THEME="robbyrussell"/ZSH_THEME="agnoster"/' /home/${USERNAME}/.zshrc && \
    sed -i 's/plugins=(git)/plugins=(git sudo docker)/' /home/${USERNAME}/.zshrc

ENV SHELL=/usr/bin/zsh
CMD ["zsh"]
