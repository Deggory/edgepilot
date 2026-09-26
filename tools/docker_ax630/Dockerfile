# AX630C(MaixCAM2) 런타임 빌드 환경. 보드 이미지와 같은 Ubuntu 22.04 arm64라
# 여기서 만든 바이너리가 보드의 glibc/libstdc++에서 그대로 돈다. Apple Silicon의
# colima(aarch64)에서는 에뮬레이션 없이 네이티브로 빌드된다.
FROM --platform=linux/arm64 ubuntu:22.04
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      build-essential cmake pkg-config libusb-1.0-0-dev ca-certificates git \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /src
