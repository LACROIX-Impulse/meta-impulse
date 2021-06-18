SUMMARY = "eView userspace interface MFIS driver module"
LICENSE = "GPLv2 & MIT"
LIC_FILES_CHKSUM = " \
    file://GPL-COPYING;md5=b234ee4d69f5fce4486a80fdaf4a4263 \
    file://MIT-COPYING;md5=f46bf18edaaf44b932fce87774c96e1d \
    "

inherit module

PN = "kernel-module-eviewitf-mfis"
PV = "0.1"

KERNEL_MODULE_PROBECONF += "eviewitf-mfis"
module_conf_eviewitf-mfis = "options eviewitf-mfis boot_mode=0"

SRC_URI = " \
    file://Makefile \
    file://eviewitf-mfis.c \
    file://include/linux/eviewitf-mfis.h \
    file://include/linux/eviewitf-mfis-driver.h \
    file://mfis-shared.h \
    file://GPL-COPYING \
    file://MIT-COPYING \
    "

S = "${WORKDIR}"

# Copy the way Modules.symvers is managed in module.bbclass.
# DEPENDS = kernel-module-eviewitf-mfis ensures these files are available for other modules.
do_install_append() {
    install -Dm0644 ${S}/include/linux/eviewitf-mfis.h ${D}/${includedir}/linux/eviewitf-mfis.h
    install -Dm0644 ${S}/include/linux/eviewitf-mfis-driver.h ${D}/${includedir}/linux/eviewitf-mfis-driver.h
    rm -rf ${D}/${includedir}/kernel-module-eviewitf-mfis
}

FILES_${PN}-${KERNEL_VERSION} += "${includedir}/linux/eviewitf-mfis.h"
FILES_${PN}-dev = "${includedir}/linux/eviewitf-mfis-driver.h"

COMPATIBLE_MACHINE = "(ecube)"
