#!/bin/sh

set -eu

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 {dm200|dm250|dm250us}" >&2
    exit 2
fi

case "$1" in
    dm200)
        defconfig=dm200_defconfig
        dtb_name=rk3128-pomera-dm200.dtb
        ;;
    dm250)
        defconfig=dm250_defconfig
        dtb_name=rk3128-pomera-dm250.dtb
        ;;
    dm250us)
        defconfig=dm250us_defconfig
        dtb_name=rk3128-pomera-dm250us.dtb
        ;;
    *)
        echo "Usage: $0 {dm200|dm250|dm250us}" >&2
        exit 2
        ;;
esac

target=$1

source_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
build_dir=$source_dir/build
cross_compile=${CROSS_COMPILE:-arm-linux-gnueabihf-}
jobs=${JOBS:-$(nproc)}
mkkrnlimg=$source_dir/mkkrnlimg.py
mkresource=$source_dir/mkresource.py
zimage=$build_dir/arch/arm/boot/zImage
dtb=$build_dir/arch/arm/boot/dts/rockchip/$dtb_name
output=$source_dir/kernel_${target}.img
resource=$source_dir/resource_${target}.img
modules_archive=$source_dir/modules_${target}.tar.gz

# depmod is commonly installed outside a normal user's PATH.
PATH=$PATH:/usr/sbin:/sbin
export PATH
depmod_cmd=$(command -v "${DEPMOD:-depmod}") || {
    echo "ERROR: depmod is required (install the kmod package)." >&2
    exit 1
}

[ -f "$mkkrnlimg" ] || {
    echo "ERROR: mkkrnlimg.py not found: $mkkrnlimg" >&2
    exit 1
}
[ -f "$mkresource" ] || {
    echo "ERROR: dm200-mkresource.py not found: $mkresource" >&2
    exit 1
}

make -C "$source_dir" O="$build_dir" \
    ARCH=arm CROSS_COMPILE="$cross_compile" \
    "$defconfig"

# Build kernel, DTB and modules together so their release and symbol versions match.
make -C "$source_dir" O="$build_dir" \
    ARCH=arm CROSS_COMPILE="$cross_compile" \
    -j"$jobs" zImage "rockchip/$dtb_name" modules

kernel_release=$(cat "$build_dir/include/config/kernel.release")
# A fresh stage avoids packaging stale modules from another model or release.
modules_stage=$(mktemp -d "$build_dir/modules-${target}.XXXXXX")
trap 'rm -rf -- "$modules_stage"' EXIT
trap 'exit 1' HUP INT TERM
make -C "$source_dir" O="$build_dir" \
    ARCH=arm CROSS_COMPILE="$cross_compile" \
    DEPMOD="$depmod_cmd" INSTALL_MOD_PATH="$modules_stage" \
    INSTALL_MOD_STRIP=1 modules_install

[ -s "$modules_stage/lib/modules/$kernel_release/modules.dep" ] && \
[ -s "$modules_stage/lib/modules/$kernel_release/modules.alias" ] || {
    echo "ERROR: module dependency/alias indexes were not generated." >&2
    exit 1
}

python3 "$mkkrnlimg" "$zimage" "$output"
python3 "$mkkrnlimg" --verify "$output" "$zimage"
python3 "$mkresource" "$dtb" "$resource"

image_size=$(stat -c '%s' "$output")
if [ "$image_size" -gt $((12 * 1024 * 1024)) ]; then
    echo "ERROR: $(basename "$output") exceeds the 12 MiB kernel slot" >&2
    exit 1
fi

# Do not ship the host build-directory link. Keep /lib/modules layout for
# extraction onto a target rootfs and for the rootfs builder's --kernel-modules.
tar -C "$modules_stage" --exclude="./lib/modules/$kernel_release/build" \
    -czf "$modules_stage/modules.tar.gz" ./lib/modules
mv -f -- "$modules_stage/modules.tar.gz" "$modules_archive"

echo "Created: $output"
echo "Created: $resource"
echo "Created: $modules_archive"
echo "Kernel release: $kernel_release"
echo "zImage:  $zimage"
echo "DTB:     $dtb"
