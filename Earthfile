VERSION 0.7
FROM ubuntu:jammy
WORKDIR /workdir

build:
    BUILD --platform=linux/arm64 +build-rpi-arm64

release:
    BUILD +release-rpi-arm64

build-rpi-arm64:
    FROM DOCKERFILE  .

release-rpi-arm64:
    FROM --platform=linux/arm64 +build-rpi-arm64
    ARG REGISTRY=ghcr.io
    ARG IMAGE_NAME=opensprinkler-firmware
    ARG --required TAG
    SAVE IMAGE --push $REGISTRY/$IMAGE_NAME:$TAG
