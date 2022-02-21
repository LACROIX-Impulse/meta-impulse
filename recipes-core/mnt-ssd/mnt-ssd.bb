SUMMARY = "SSD Mount unit configuration"
DESCRIPTION = "Automatically mount SSD partition only if device is detected"
LICENSE = "CLOSED"
PR = "r3"

SRC_URI =  " \
    file://mnt-ssd.mount \
"

inherit allarch systemd

NATIVE_SYSTEMD_SUPPORT = "1"
SYSTEMD_PACKAGES = "${PN}"
SYSTEMD_SERVICE_${PN} = "mnt-ssd.mount"

do_compile () {
}

do_install () {
    install -d ${D}${systemd_unitdir}/system/
    install -m 0644 ${WORKDIR}/mnt-ssd.mount ${D}${systemd_unitdir}/system
}