#!/bin/sh

# resize MMC partition if not already done
resize2fs /dev/mmcblk0

# mask itself
systemctl mask initscript