# ROS 2 Humble desktop for the buggy, in Docker on a Mac (or any PC).
#
# - Full XFCE desktop served over RDP: open it with Microsoft "Windows App"
#   at localhost:3390 (user ros / password ros). RViz starts on login.
# - Talks to the Pi through zenoh-bridge-ros2dds over one TCP connection
#   (Pi port 7447), so no DDS/UDP ports have to cross Docker's NAT.
#   The bridge version MUST match the one on the Pi (1.10.2).
#
# Works natively on Apple Silicon (arm64) and Intel (amd64).
#
# Base: the official multi-arch ros:humble-ros-base image (Ubuntu 22.04).
# Not osrf/ros:humble-desktop: on Apple Silicon it ran as emulated x86_64,
# which broke the bridge and would make RViz slow.

FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Fail early (and say why) if this image is not native for the machine.
RUN echo "Image CPU: $(uname -m) / $(dpkg --print-architecture)"

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-humble-rviz2 \
        ros-humble-rviz-default-plugins \
        ros-humble-nav2-rviz-plugins \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-tf2-tools \
        xfce4 xfce4-terminal dbus-x11 \
        xrdp xorgxrdp \
        curl ca-certificates unzip \
    && (apt-get purge -y xfce4-screensaver light-locker xscreensaver || true) \
    && rm -rf /var/lib/apt/lists/*

# Zenoh bridge for the CPU this image really runs on (`uname -m`, not
# TARGETARCH, which can differ from the base image's architecture).
# Checksums from the GitHub release. The final `--version` makes the build
# fail here if the binary cannot run, instead of at container start.
ARG ZENOH_VERSION=1.10.2
RUN arch="$(uname -m)" \
    && echo "Installing zenoh bridge for $arch" \
    && case "$arch" in \
        x86_64)        z=x86_64;  sum=d6c3ff1c6886b9a284dd2b192a0acbdd5240aea266ad07e8febfcc0b6bce06bc ;; \
        aarch64|arm64) z=aarch64; sum=af4ee9ee4b442a88dcff9051c56ff853025efbaf6ffb8405f7bc24a761b7b9da ;; \
        *) echo "unsupported architecture: $arch" >&2; exit 1 ;; \
       esac \
    && curl -fsSL -o /tmp/zenoh.zip \
        "https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/${ZENOH_VERSION}/zenoh-plugin-ros2dds-${ZENOH_VERSION}-${z}-unknown-linux-gnu-standalone.zip" \
    && echo "${sum}  /tmp/zenoh.zip" | sha256sum -c - \
    && unzip -q /tmp/zenoh.zip -d /tmp/zenoh \
    && install -m 755 /tmp/zenoh/zenoh-bridge-ros2dds /usr/local/bin/ \
    && rm -rf /tmp/zenoh /tmp/zenoh.zip \
    && zenoh-bridge-ros2dds --version

# Desktop user (only reachable on the Mac's localhost, see docker-compose.yml)
RUN useradd -m -s /bin/bash ros \
    && echo "ros:ros" | chpasswd \
    && adduser xrdp ssl-cert

# DDS inside the container stays on loopback; only the bridge talks to the Pi.
COPY cyclonedds.xml /etc/cyclonedds.xml
RUN printf '%s\n' \
        '[ -n "$BASH_VERSION" ] || return 0' \
        'source /opt/ros/humble/setup.bash' \
        'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' \
        'export CYCLONEDDS_URI=file:///etc/cyclonedds.xml' \
        'export ROS_DOMAIN_ID=0' \
        'export LIBGL_ALWAYS_SOFTWARE=1' \
        '# `rviz2` with no arguments opens the buggy Nav2 view' \
        'rviz2() { if [ $# -eq 0 ]; then command rviz2 -d "$HOME/buggy.rviz"; else command rviz2 "$@"; fi; }' \
        'alias rviz=rviz2' \
        > /etc/profile.d/ros.sh \
    && echo 'source /etc/profile.d/ros.sh' >> /home/ros/.bashrc

# RDP session -> XFCE (unset vars that otherwise give a black screen)
RUN sed -i '2i unset DBUS_SESSION_BUS_ADDRESS\nunset XDG_RUNTIME_DIR\n. /etc/profile' \
        /etc/xrdp/startwm.sh \
    && echo 'startxfce4' > /home/ros/.xsession

# "Buggy RViz" in the menu, and started automatically at login
RUN printf '%s\n' \
        '[Desktop Entry]' \
        'Type=Application' \
        'Name=Buggy RViz' \
        'Comment=RViz with the buggy Nav2 view' \
        'Exec=bash -lc "rviz2 -d $HOME/buggy.rviz"' \
        'Icon=applications-science' \
        'Terminal=false' \
        > /usr/share/applications/buggy-rviz.desktop \
    && mkdir -p /home/ros/.config/autostart \
    && cp /usr/share/applications/buggy-rviz.desktop /home/ros/.config/autostart/ \
    && chown -R ros:ros /home/ros

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 3389
ENTRYPOINT ["/entrypoint.sh"]
