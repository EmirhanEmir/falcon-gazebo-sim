#!/usr/bin/env bash
#
# FALCON V2 - beş Gazebo system plugin'inin tamamını derler.
#
# Her plugin bağımsız bir CMake projesidir ve kendi plugins/<ad>/build/
# dizinine derlenir; launch script'leri ortaya çıkan paylaşımlı kütüphaneleri
# orada arar.
#
# Kullanım:
#   ./scripts/build_plugins.sh [cmake-build-type]     # varsayılan: Release
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_TYPE="${1:-Release}"

for plugin in aerodynamics propulsion actuators sensors wind; do
  echo "=== Building $plugin ($BUILD_TYPE) ==="
  cmake -S "$REPO_ROOT/plugins/$plugin" -B "$REPO_ROOT/plugins/$plugin/build" \
        -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
  cmake --build "$REPO_ROOT/plugins/$plugin/build" -j "$(nproc)"
done

echo
echo "Done. Shared libraries:"
find "$REPO_ROOT/plugins" -name 'libFalconV2*.so' -printf '  %p\n' | sort
