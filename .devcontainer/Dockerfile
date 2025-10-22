FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai
SHELL ["/bin/bash", "-c"]

RUN apt-get update && \
    apt-get install -y git curl wget make cmake sudo build-essential \
                       ca-certificates lsb-release software-properties-common gnupg zsh && \
    rm -rf /var/lib/apt/lists/*

# 安装 LLVM 21
RUN curl -fsSL  -fsSL https://mirrors.tuna.tsinghua.edu.cn/llvm-apt/llvm.sh | bash -s -- 21 -m https://mirrors.tuna.tsinghua.edu.cn/llvm-apt && \
    apt-get install -y clang-21 clang-tools-21 lld-21 lldb-21 libc++-21-dev libc++abi-21-dev && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

RUN update-alternatives --install /usr/bin/clang clang /usr/bin/clang-21 100
RUN update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-21 100
RUN update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-21 100
RUN update-alternatives --config clang
RUN update-alternatives --config clang++
RUN update-alternatives --config clangd

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

# 安装 oh-my-zsh
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended && \
    sed -i 's/ZSH_THEME="robbyrussell"/ZSH_THEME="agnoster"/' /home/${USERNAME}/.zshrc && \
    sed -i 's/plugins=(git)/plugins=(git sudo docker)/' /home/${USERNAME}/.zshrc

ENV SHELL=/usr/bin/zsh
CMD ["zsh"]

# 代理设置
ENV HTTP_PROXY=http://127.0.0.1:7890
ENV HTTPS_PROXY=http://127.0.0.1:7890

