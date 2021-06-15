SUMMARY = "SPI to MFIS userspace interface driver"
LICENSE = "GPLv2 & MIT"
LIC_FILES_CHKSUM = " \
    file://GPL-COPYING;md5=b234ee4d69f5fce4486a80fdaf4a4263 \
    file://MIT-COPYING;md5=f46bf18edaaf44b932fce87774c96e1d \
    "

inherit module

PN = "kernel-module-spi-mfis"
PV = "0.1"

DEPENDS = "kernel-module-eviewitf-mfis"
KERNEL_MODULE_PROBECONF += "spi-mfis"
module_conf_spi-mfis = "options spidev bufsiz=32768"


SRC_URI = " \
    file://Makefile \
    file://spi-mfis.c \
    file://GPL-COPYING \
    file://MIT-COPYING \
    "

S = "${WORKDIR}"

COMPATIBLE_MACHINE = "(ecube)"
