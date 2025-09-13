# Isaac Sim 5.0.0 + ROS 2 Humble (robust build)
FROM nvcr.io/nvidia/isaac-sim:5.0.0

# ---------- Base env ----------
ENV ACCEPT_EULA=Y \
    PRIVACY_CONSENT=Y \
    QT_X11_NO_MITSHM=1 \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all \
    DEBIAN_FRONTEND=noninteractive

# Workdir for your convenience
WORKDIR /root

# ---------- Caches & dirs ----------
RUN mkdir -p \
    /root/.cache/ov \
    /root/.cache/pip \
    /root/.cache/nvidia/GLCache \
    /root/.nv/ComputeCache \
    /root/.nvidia-omniverse/logs \
    /root/.local/share/ov/data \
    /root/.local/share/ov/pkg \
    /root/Documents \
    /isaac-sim/exts/isaacsim.asset.browser/cache

# ---------- Python pip (best-effort) ----------
RUN python3 -m pip install --upgrade pip || true

# ---------- Locale & timezone (avoid warnings) ----------
RUN set -eux; \
  echo 'Etc/UTC' > /etc/timezone; \
  ln -sf /usr/share/zoneinfo/Etc/UTC /etc/localtime; \
  rm -rf /var/lib/apt/lists/* /var/lib/apt/lists/partial /var/cache/apt/archives/partial; \
  echo 'Acquire::Retries "3";' > /etc/apt/apt.conf.d/99retries; \
  echo 'Acquire::http::Pipeline-Depth "0";' > /etc/apt/apt.conf.d/99pipeline; \
  apt-get update -o Acquire::CompressionTypes::Order::=gz; \
  apt-get install -y --no-install-recommends locales tzdata ca-certificates curl gnupg lsb-release; \
  sed -i 's/^# *\(en_US.UTF-8 UTF-8\)/\1/' /etc/locale.gen; \
  locale-gen; \
  update-locale LANG=en_US.UTF-8; \
  rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# (Optional) pin to a local mirror or snapshot for ultimate stability:
# RUN sed -i 's|http://archive.ubuntu.com/ubuntu|http://kr.archive.ubuntu.com/ubuntu|g' /etc/apt/sources.list
# RUN sed -i 's|http://archive.ubuntu.com/ubuntu|http://snapshot.ubuntu.com/ubuntu/20250911|g' /etc/apt/sources.list

# ---------- ROS 2 Humble repo (keyring, no apt-key/keyserver) ----------
RUN set -eux; \
  install -m 0755 -d /etc/apt/keyrings; \
  curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg; \
  chmod 0644 /etc/apt/keyrings/ros-archive-keyring.gpg; \
  sh -c 'echo "deb [signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list'

# ---------- Install ROS 2 Humble (ros-base) ----------
# rviz/OGRE/VtK 등 GUI 의존성으로 깨지는 이슈 회피를 위해 ros-base 사용
RUN set -eux; \
  rm -rf /var/lib/apt/lists/* /var/lib/apt/lists/partial /var/cache/apt/archives/partial; \
  apt-get update -o Acquire::CompressionTypes::Order::=gz; \
  apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    git; \
  rm -rf /var/lib/apt/lists/*

# ---------- rosdep init/update (best-effort; won’t fail build if network flakes) ----------
RUN set -eux; \
  rosdep init || true; \
  rosdep update --rosdistro humble || true

# ---------- Helpful defaults ----------
ENV ROS_DISTRO=humble \
    ROS_DOMAIN_ID=34 \
    ROS_LOCALHOST_ONLY=1 \
    XDG_RUNTIME_DIR=/tmp/runtime-root
RUN mkdir -p /tmp/runtime-root && chmod 0700 /tmp/runtime-root

# Auto-source in interactive shells
RUN printf '\n# --- ROS 2 Humble ---\n[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash\n' >> /etc/bash.bashrc && \
    printf '\n# --- User ROS2 workspace ---\n[ -f ~/ros2_ws/install/local_setup.bash ] && source ~/ros2_ws/install/local_setup.bash\n' >> /etc/bash.bashrc && \
    printf '\n# --- Convenience aliases ---\n' >> /etc/bash.bashrc && \
    printf "alias cw='cd ~/ros2_ws'\n" >> /etc/bash.bashrc && \
    printf "alias cs='cd ~/ros2_ws/src'\n" >> /etc/bash.bashrc && \
    printf "alias cb='cd ~/ros2_ws && colcon build --symlink-install'\n" >> /etc/bash.bashrc && \
    printf "alias cbp='cd ~/ros2_ws && colcon build --symlink-install --packages-select'\n" >> /etc/bash.bashrc && \
    printf "alias sb='source ~/.bashrc'\n" >> /etc/bash.bashrc && \
    printf "alias rt='ros2 topic list'\n" >> /etc/bash.bashrc && \
    printf "alias re='ros2 topic echo'\n" >> /etc/bash.bashrc && \
    printf "alias rn='ros2 node list'\n" >> /etc/bash.bashrc && \
    printf "alias testpub=\"ros2 run demo_nodes_cpp talker\"\n" >> /etc/bash.bashrc && \
    printf "alias testsub=\"ros2 run demo_nodes_cpp listener\"\n" >> /etc/bash.bashrc

# ---------- Optional one-shot setup script (kept, but not auto-run) ----------
RUN printf '#!/usr/bin/env bash\nset -eo pipefail\nif [ -f /opt/ros/humble/setup.bash ]; then\n  source /opt/ros/humble/setup.bash\n  if [ ! -d "$HOME/ros2_ws" ]; then\n    echo "[setup] create ros2_ws"; mkdir -p "$HOME/ros2_ws/src"\n    cd "$HOME/ros2_ws/src"\n    # Example: clone your repo (adjust as you like)\n    git clone https://github.com/sangbeom0321/isaac_sim_orchard.git\n    cd "$HOME/ros2_ws" && colcon build --symlink-install\n  fi\nelse\n  echo "[setup] /opt/ros/humble not found."\nfi\n' > /root/setup_ros2.sh && chmod +x /root/setup_ros2.sh && /bin/bash -lc '/root/setup_ros2.sh'

# ---------- Do nothing by default (attach later) ----------
ENTRYPOINT []
CMD ["bash", "-lc", "sleep infinity"]
