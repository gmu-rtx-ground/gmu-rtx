#!/bin/bash

log() {
    local level="$1"
    shift
    local timestamp
    timestamp="$(date '+%Y-%m-%dT%H:%M:%S')"
    printf "[%s] [%s] %s\n" "$timestamp" "$level" "$*"
}

# Set image name
DOCKER_IMAGE="${DOCKER_IMAGE:-ghcr.io/gmu-rtx-ground/gmu-rtx}"

# Build Architectures
ARCHS=(amd64 arm64)

# Build images
for ARCH in "${ARCHS[@]}"; do
    log INFO "Building and pushing ${ARCH} image..."
    docker buildx build \
        --platform linux/${ARCH} \
        -f docker/Dockerfile.${ARCH} \
        -t ${DOCKER_IMAGE}:${ARCH} \
        --ssh default \
        --push \
        .
done

DIGEST_AMD64=$(docker inspect --format='{{index .RepoDigests 0}}' "${DOCKER_IMAGE}:amd64")
log INFO "Digest for linux/amd64: ${DIGEST_AMD64}"
DIGEST_ARM64=$(docker inspect --format='{{index .RepoDigests 0}}' "${DOCKER_IMAGE}:arm64")
log INFO "Digest for linux/arm64: ${DIGEST_ARM64}"
DIGEST_LATEST=$(docker inspect --format='{{index .RepoDigests 0}}' "${DOCKER_IMAGE}:latest")
log INFO "Digest for multi-platform image: ${DIGEST_LATEST}"

docker image rm "${DIGEST_LATEST}"

# Create and publish new multi-platform image
log INFO "Building ${DOCKER_IMAGE}:latest image..."
docker buildx imagetools create -t "${DOCKER_IMAGE}:latest" \
        "${DIGEST_AMD64}" \
        "${DIGEST_ARM64}"

echo "[Info] : Pushing ${DOCKER_IMAGE}:latest"
docker push "${DOCKER_IMAGE}:latest"

echo "Complete!"
