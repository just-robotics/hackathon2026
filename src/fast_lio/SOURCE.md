# FAST-LIO2 dependency

Vendored from https://github.com/hku-mars/FAST_LIO, ROS2 branch,
commit a4743b095409588842a5b30ddfa27e29d2f99164.
ikd-Tree: e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4.
Upstream licenses are retained. No nested Git repositories or submodules.

Only source/build/runtime files are included; upstream datasets and media are omitted.
COLCON_IGNORE excludes this hardware dependency from the simulation build.
Dockerfile.real removes the marker inside its solution workspace.
