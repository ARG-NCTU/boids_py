FROM jetseaaiusvfleet/moos-ros2-base:release

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /workspace

RUN apt-get update && \
    apt-get install -y python3 python3-pygame tmux && \
    rm -rf /var/lib/apt/lists/*

COPY . ./

ENTRYPOINT ["/workspace/entrypoint.sh"]
