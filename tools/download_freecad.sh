#!/bin/bash

set -e

FREECAD_VERSION="1.1.1"
FREECAD_FILE="FreeCAD_${FREECAD_VERSION}-Linux-x86_64-py311.AppImage"
FREECAD_URL="https://github.com/FreeCAD/FreeCAD/releases/download/${FREECAD_VERSION}/${FREECAD_FILE}"

DOWNLOAD_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
TARGET_PATH="${DOWNLOAD_DIR}/${FREECAD_FILE}"

if [ -f "${TARGET_PATH}" ]; then
    echo "FreeCAD ${FREECAD_VERSION} already exists at ${TARGET_PATH}"
    exit 0
fi

echo "Downloading FreeCAD ${FREECAD_VERSION} from ${FREECAD_URL}"

if command -v curl &>/dev/null; then
    curl -L -o "${TARGET_PATH}" "${FREECAD_URL}"
elif command -v wget &>/dev/null; then
    wget -O "${TARGET_PATH}" "${FREECAD_URL}"
else
    echo "Error: Neither curl nor wget is available. Please install one of them."
    exit 1
fi

chmod +x "${TARGET_PATH}"

echo "Successfully downloaded FreeCAD ${FREECAD_VERSION} to ${TARGET_PATH}"
echo "You can run it with: ${TARGET_PATH}"