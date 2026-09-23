#!/usr/bin/env bash
# Apply cameraif's MicroPython patches to a MicroPython tree.
#
#   ./apply_patches.sh --status [MP_DIR]
#   ./apply_patches.sh --apply  [MP_DIR]
#   ./apply_patches.sh --revert [MP_DIR]
#
# MP_DIR defaults to a `micropython` checkout beside this repository.
#
# Two small patches to the esp32 port, P4 only: the MIPI-CSI sensor-driver
# component (esp_cam_sensor) added to the port's idf_component.yml, and the
# sdkconfig it needs. ESP-IDF ships the CSI controller but no sensor drivers,
# so without these the OV5647 on the P4 panel has nothing to drive it. Each
# patch carries its purpose in its own header. They live here, with the module
# that needs them, rather than as undocumented edits in a pinned upstream tree.
set -euo pipefail
MODE="${1:---status}"
MP_DIR="${2:-$(cd "$(dirname "$0")/.." && pwd)/micropython}"
PATCH_DIR="$(cd "$(dirname "$0")" && pwd)/patches"
if [ ! -d "$MP_DIR" ]; then
    echo "MicroPython tree not found: $MP_DIR" >&2
    exit 1
fi
patches=$(ls "$PATCH_DIR"/*.patch | sort)

case "$MODE" in
    --status)
        for p in $patches; do
            if git -C "$MP_DIR" apply --reverse --check "$p" 2>/dev/null; then
                printf 'applied     %s\n' "$(basename "$p")"
            elif git -C "$MP_DIR" apply --check "$p" 2>/dev/null; then
                printf 'not applied %s\n' "$(basename "$p")"
            else
                printf 'CONFLICT    %s  -- inspect before building\n' "$(basename "$p")"
            fi
        done
        ;;
    --apply)
        for p in $patches; do
            if git -C "$MP_DIR" apply --reverse --check "$p" 2>/dev/null; then
                printf 'already applied %s\n' "$(basename "$p")"
            else
                git -C "$MP_DIR" apply "$p"
                printf 'applied         %s\n' "$(basename "$p")"
            fi
        done
        ;;
    --revert)
        for p in $(echo "$patches" | tac); do
            if git -C "$MP_DIR" apply --reverse --check "$p" 2>/dev/null; then
                git -C "$MP_DIR" apply --reverse "$p"
                printf 'reverted        %s\n' "$(basename "$p")"
            else
                printf 'not applied     %s\n' "$(basename "$p")"
            fi
        done
        ;;
    *)
        echo "usage: $0 --status|--apply|--revert [MP_DIR]" >&2
        exit 2
        ;;
esac
