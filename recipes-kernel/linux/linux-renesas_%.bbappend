FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:"

COMPATIBLE_MACHINE_ecube = "ecube"

SRC_URI_append_ecube = " \
    file://0001-arm64-dts-renesas-add-eCube-board.patch \
    file://0002-arm64-dts-renesas-activate-pcie-on-eCube-board.patch \
    file://0003-arm64-dts-renesas-eView-coexecution-patch.patch \
    file://0004-arm64-dts-renesas-eViewItf-mfis-driver.patch \
    file://0005-Add-SPI-MFIS-device-nodes-and-compatibility-with-spi.patch \
    file://0006-r8a77980-eCube-dts-decrease-MMC-max-frequency.patch \
    file://0007-Do-not-stop-cortex-R7.patch \
"

KERNEL_DEVICETREE_append_ecube= " \
    renesas/r8a77980-ecube.dtb \
"

