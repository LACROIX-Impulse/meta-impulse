SUMMARY = "libeviewitf"
SECTION = "libs"
LICENSE = "eSoftThings"
LIC_FILES_CHKSUM = "file://${WORKDIR}/git/LICENSE;md5=c6afb05c83ee3f78ca6877faf927aede"

PACKAGE_ARCH = "all"

SRCTAG = "0.1"
#SRCREV = "1e8f1f468186b043a7ba0d1f1debdc382ebc0238"
#SRCBRANCH = "master"

SRC_URI = "git://git@10.224.240.124:7999/em/eview_itf.git;protocol=ssh;tag=${SRCTAG}"
#SRC_URI = "git://git@10.224.240.124:7999/em/eview_itf.git;protocol=ssh;branch=${SRCBRANCH}"

S = "${WORKDIR}/git/"

# we need to pass the Cxx parameter extra to the make call
EXTRA_OEMAKE = "'CC=${CC}' 'AR=${AR}' 'CFLAGS=${CFLAGS}'"

do_compile() {
    oe_runmake all
}

do_install() {
    install -d ${D}${libdir}
    install ${S}build/libmfis.a ${D}${libdir}
    install -d ${D}${includedir}
    install ${S}include/libmfis.h ${D}${includedir}
}

RDEPENDS_${PN}-dev = ""
RDEPENDS_${PN}-staticdev = "${PN}-dev"

FILES_${PN}-dev += "${includedir}"
FILES_${PN}-staticdev += "${libdir}"

