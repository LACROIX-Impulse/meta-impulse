FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:"

COMPATIBLE_MACHINE_ecube = "ecube"

SRC_URI_append_ecube = " \
    file://0001-arm64-dts-renesas-add-eCube-board.patch \
    file://0002-arm64-dts-renesas-eView-coexecution.patch \
    file://0003-do_not_stop_the_cortex_r7.patch \
    file://0004-mfis-driver-dtree.patch \
    file://0005-mfis-driver-kernel.patch \
    file://0006-mfis-driver-update.patch \
    file://0007-mfis-add-poll.patch \
    file://0008-mfis-add-virtual_cameras.patch \
"

SRC_URI_append_ecube = " file://ecube.cfg"

KERNEL_DEVICETREE_append_ecube= " \
    renesas/r8a77980-ecube.dtb \
"
