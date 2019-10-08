FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:"

COMPATIBLE_MACHINE_ecube = "ecube"

SRC_URI_append_ecube = " \
    file://0001-arm64-dts-renesas-add-eCube-board.patch \
"

SRC_URI_append_ecube = " file://ecube.cfg"

KERNEL_DEVICETREE_append_ecube= " \
    renesas/r8a77980-ecube.dtb \
"
