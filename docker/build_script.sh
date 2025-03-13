#!/bin/bash

# Set image name
IMAGE_NAME="ghcr.io/gmu-rtx-ground/gmu-rtx"

# Build Architectures
ARCHS=(amd64 arm64)

# Build images
for ARCH in "${ARCHS[@]}"; do
    echo "==> Building and pushing ${ARCH} image..."
    docker buildx build \
        --platform linux/${ARCH} \
        -f docker/Dockerfile.${ARCH} \
        -t ${IMAGE_NAME}:${ARCH} \
        --ssh default \
        --push \
        .
done

DIGEST_AMD64=$(docker inspect --format='{{index .RepoDigests 0}}' "${DOCKER_IMAGE}:amd64")
echo "Digest for linux/amd64: ${DIGEST_AMD64}"
DIGEST_ARM64=$(docker inspect --format='{{index .RepoDigests 0}}' "${DOCKER_IMAGE}:arm64")
echo "Digest for linux/arm64: ${DIGEST_ARM64}"

# Create and publish new multi-platform image
echo "==> pushing ${IMAGE_NAME}:latest image..."
docker buildx imagetools create -t "${IMAGE_NAME}:latest" \
    ${DIGEST_AMD64} \
    ${DIGEST_ARM64}

docker push "${IMAGE_NAME}:latest"

echo "Complete!"
