#!/bin/sh

set -eu

usage()
{
    echo "Usage: $0 {dm200|dm250|dm250us} <whole-SD-device|existing-SD-image>" >&2
    exit 2
}

fail()
{
    echo "ERROR: $*" >&2
    exit 1
}

[ "$#" -eq 2 ] || usage
case "$1" in
    dm200|dm250|dm250us) target=$1 ;;
    *) usage ;;
esac

source_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
kernel=$source_dir/kernel_${target}.img
resource=$source_dir/resource_${target}.img
destination=$(readlink -f -- "$2") || fail "Cannot resolve destination: $2"

# These absolute offsets are for the Pomera SD boot layout, not raw eMMC.
kernel_offset=$((12 * 1024 * 1024))
resource_offset=$((24 * 1024 * 1024))
resource_limit=$((6 * 1024 * 1024))

for image in "$kernel" "$resource"; do
    [ -f "$image" ] && [ -r "$image" ] && [ -s "$image" ] ||
        fail "Missing, unreadable or empty image: $image"
    [ ! "$destination" -ef "$image" ] || fail "Destination is an input image: $image"
done
kernel_size=$(stat -c '%s' -- "$kernel")
resource_size=$(stat -c '%s' -- "$resource")
[ "$kernel_size" -le "$((resource_offset - kernel_offset))" ] ||
    fail "Kernel image exceeds the 12 MiB slot"
[ "$resource_size" -le "$resource_limit" ] ||
    fail "Resource image exceeds the 6 MiB slot"
python3 "$source_dir/mkkrnlimg.py" --verify "$kernel"

[ -w "$destination" ] || fail "Destination is not writable: $destination"
PATH=$PATH:/usr/sbin:/sbin
export PATH

check_device()
{
    device_type=$(lsblk -dn -o TYPE -- "$destination") ||
        fail "Cannot inspect destination"
    case "$device_type" in
        disk|loop) ;;
        *) fail "Specify the whole SD device, not a partition or mapped device" ;;
    esac
    read_only=$(blockdev --getro "$destination")
    [ "$read_only" -eq 0 ] || fail "Destination is read-only"
    mountpoints=$(lsblk -nr -o MOUNTPOINTS -- "$destination") ||
        fail "Cannot inspect mounts"
    case "$mountpoints" in
        *[![:space:]]*) fail "Unmount all destination filesystems and disable its swap first" ;;
    esac
    types=$(lsblk -nr -o TYPE -- "$destination") ||
        fail "Cannot inspect dependent devices"
    for type in $types; do
        case "$type" in
            disk|loop|part) ;;
            *) fail "Destination has an active mapped device: $type" ;;
        esac
    done
    starts=$(lsblk -nr -o START -- "$destination") ||
        fail "Cannot inspect partition boundaries"
    for start in $starts; do
        [ "$start" -ge 61440 ] ||
            fail "A partition overlaps the reserved first 30 MiB"
    done
}

check_image()
{
    # An image used by a loop device may contain a live mounted filesystem.
    loops=$(losetup -j "$destination") || fail "Cannot inspect image loop devices"
    [ -z "$loops" ] || fail "Detach loop devices using the destination image first"
}

if [ -b "$destination" ]; then
    check_device
    destination_size=$(blockdev --getsize64 "$destination")
elif [ -f "$destination" ]; then
    check_image
    destination_size=$(stat -c '%s' -- "$destination")
else
    fail "Destination must be a block device or an existing regular SD image"
fi
[ "$destination_size" -ge "$((resource_offset + resource_limit))" ] ||
    fail "Destination is smaller than the reserved 30 MiB boot area"

printf 'Model: %s\nKernel: %s (%s bytes) at 12 MiB\nResource: %s (%s bytes) at 24 MiB\n' \
    "$target" "$kernel" "$kernel_size" "$resource" "$resource_size"
printf 'WARNING: overwriting the SD boot area of %s (%s bytes).\n' \
    "$destination" "$destination_size"
echo "This does not install modules or update U-Boot. Do not use raw eMMC."
printf 'Type the full destination path to continue: '
IFS= read -r confirmation || fail "Confirmation required"
[ "$confirmation" = "$destination" ] || fail "Cancelled; nothing written"

# Recheck mounts after the confirmation prompt; do not mount during the update.
if [ -b "$destination" ]; then
    check_device
fi
if [ -f "$destination" ]; then
    check_image
fi
dd if="$kernel" of="$destination" bs=1M seek=12 conv=notrunc,fsync status=progress
dd if="$resource" of="$destination" bs=1M seek=24 conv=notrunc,fsync status=progress
echo "Written: $destination"
