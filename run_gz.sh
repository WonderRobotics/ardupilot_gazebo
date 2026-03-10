#!/bin/bash

# 1. Optimize CPU threading
export OPENBLAS_NUM_THREADS=1

# 2. Force NVIDIA for rendering
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json

# 3. Suppress Wayland issues (use X11 bridge)
export QT_QPA_PLATFORM=xcb

# -s: silent (no GUI)
gz sim -s -r --headless-rendering $1.sdf
