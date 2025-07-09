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
RUN echo 'source "/ws/install/setup.bash"' >> ~/.bashrc

FROM built-workspace AS test-workspace
# Run colcon-recognized project tests
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cd /ws \
    && colcon test

FROM built-workspace AS final
WORKDIR "/ws"
