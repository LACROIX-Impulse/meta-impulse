FILESEXTRAPATHS_append := "${THISDIR}/${PN}:"

SRC_URI += "file://ip-change \
           "

do_install_append () {
    install -d ${D}/mnt/ssd
    install -d ${D}${sbindir}
    install -m 0755 ${WORKDIR}/ip-change ${D}${sbindir}/ip-change
}
