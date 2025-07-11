ARG base_image=ros:jazzy
FROM ${base_image} AS base
COPY docker/overrides/. /

FROM base AS base-installed-deps
COPY . /ws/src/usv-autonomy
RUN --mount=type=cache,dst=/var/lib/apt/lists,sharing=locked \
    --mount=type=cache,dst=/var/cache/apt,sharing=locked \
    . /opt/ros/${ROS_DISTRO}/setup.sh \
    && rm -f "/etc/apt/apt.conf.d/docker-clean" \
    && apt update \
    && rosdep install \
      --from-paths "/ws/src" \
      --ignore-packages-from-source \
      --rosdistro "${ROS_DISTRO}" \
      --default-yes

FROM base-installed-deps AS built-workspace
# Build the project
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cd /ws \
    && colcon build --symlink-install

# Add workspace setup.bash to sourcing chain in entrypoint script
# RUN echo 'source "/ws/install/setup.bash"' >> ~/.bashrc
RUN sed -i '/^###AFTER_ROS_DISTRO/i source "/ws/install/setup.bash" --' \
    "/ros_entrypoint.sh" \
    && \
    sed -i '$ d' ~/.bashrc \
    && \
    echo 'source <(grep "^source" /ros_entrypoint.sh)' >> ~/.bashrc
# TODO: this will yield a double-entry in both files for build-devel; refactor

FROM built-workspace AS devel-workspace

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cd /ws \
    && colcon build --symlink-install --cmake-args "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"

RUN --mount=type=cache,dst=/var/lib/apt/lists,sharing=locked \
    --mount=type=cache,dst=/var/cache/apt,sharing=locked \
       rm -f "/etc/apt/apt.conf.d/docker-clean" \
    && apt update \
    && apt install -y \
       python3-pip \
       pipx \
       \
       clangd \
       \
       python3-pylsp \
       python3-pylsp-mypy

# TODO(pszenher): how to we ask pip to install `python-lsp-ruff`
#     without trying (and failing) to upgrade other package versions
#     (as some are installed via `apt`)
RUN pip install --break-system-packages python-lsp-ruff "typing_extensions==4.10"

WORKDIR "/ws"


FROM built-workspace AS test-workspace
# Run colcon-recognized project tests
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cd /ws \
    && colcon test

FROM built-workspace AS final
