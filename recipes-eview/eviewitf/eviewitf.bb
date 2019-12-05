SUMMARY = "eviewitf"
SECTION = "libs"
LICENSE = "eSoftThings"
LIC_FILES_CHKSUM = "file://${WORKDIR}/git/LICENSE;md5=c6afb05c83ee3f78ca6877faf927aede"

PACKAGE_ARCH = "all"

SRCTAG = "0.2"
#SRCREV = "fcb5440590733584e6439f8e7b6657af80f20f48"
#SRCBRANCH = "release/0.2"

SRC_URI = "git://git@10.224.240.124:7999/em/eview_itf.git;protocol=ssh;tag=${SRCTAG}"
#SRC_URI = "git://git@10.224.240.124:7999/em/eview_itf.git;protocol=ssh;branch=${SRCBRANCH}"

S = "${WORKDIR}/git/"

# we need to pass the Cxx parameter extra to the make call
EXTRA_OEMAKE = "'CC=${CC}' 'AR=${AR}' 'CFLAGS=${CFLAGS}'"

do_compile() {
    oe_runmake all
}

do_install() {
    install -d ${D}${bindir}
    install ${S}build/eviewitf ${D}${bindir}
    install -d ${D}${libdir}
    install ${S}build/libeviewitf.a ${D}${libdir}
    install -d ${D}${includedir}
    install ${S}include/eviewitf.h ${D}${includedir}
}

RDEPENDS_${PN} = ""
RDEPENDS_${PN}-dev = ""
RDEPENDS_${PN}-staticdev = "${PN}-dev"

FILES_${PN} += "${bindir}"
FILES_${PN}-dev += "${includedir}"
FILES_${PN}-staticdev += "${libdir}"

