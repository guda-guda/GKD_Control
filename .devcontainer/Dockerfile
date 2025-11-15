FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai
SHELL ["/bin/bash", "-c"]

# 1. 基础工具
RUN apt-get update && \
    apt-get install -y \
        build-essential \
        git curl wget \
        cmake \
        ca-certificates \
        zsh sudo \
        python3 python3-pip \
        locales \
        software-properties-common gnupg \
    && rm -rf /var/lib/apt/lists/*

# 2. UTF-8 本地化
RUN locale-gen en_US.UTF-8 zh_CN.UTF-8 || true && \
    update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# 3. 安装 Clang/LLVM 21作为备用工具链，不强制为默认,使用国内源
RUN curl -fsSL https://mirrors.tuna.tsinghua.edu.cn/llvm-apt/llvm.sh -o /tmp/llvm.sh && \
    bash /tmp/llvm.sh 21 -m https://mirrors.tuna.tsinghua.edu.cn/llvm-apt && \
    apt-get update && \
    apt-get install -y clang-21 clang-tools-21 lld-21 lldb-21 libc++-21-dev libc++abi-21-dev && \
    ln -s /usr/bin/clangd-21 /usr/bin/clangd && \
    rm -rf /var/lib/apt/lists/* /tmp/llvm.sh

# 不在这里改默认编译器，需要时在 xmake 里手动切 toolchain 即可

# 4. 创建非 root 用户
ARG USERNAME=gkd
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
RUN adduser --disabled-password --gecos "" --uid ${USER_UID} ${USERNAME} && \
    usermod -aG sudo ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
WORKDIR /home/${USERNAME}

ENV SHELL=/usr/bin/zsh
ENV PATH="/home/${USERNAME}/.local/bin:${PATH}"

# 5. 安装 xmake
RUN bash <(curl -fsSL https://xmake.io/shget.text)

# 6. 安装 oh-my-zsh，并写入干净的 agnoster 配置
RUN RUNZSH=no CHSH=no KEEP_ZSHRC=yes \
    sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" && \
    mkdir -p ~/.oh-my-zsh && \
    printf '%s\n' 'export ZSH="$HOME/.oh-my-zsh"' '' 'ZSH_THEME="agnoster"' 'plugins=(git)' '' 'source $ZSH/oh-my-zsh.sh' '' 'export LANG=en_US.UTF-8' 'export LC_ALL=en_US.UTF-8' > ~/.zshrc

CMD ["zsh"]

