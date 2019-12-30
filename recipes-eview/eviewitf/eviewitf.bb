SUMMARY = "eviewitf"
SECTION = "libs"
LICENSE = "eSoftThings"
LIC_FILES_CHKSUM = "file://${WORKDIR}/git/LICENSE;md5=c6afb05c83ee3f78ca6877faf927aede"

PACKAGE_ARCH = "all"

SRCTAG = "0.3"
#SRCREV = "fcb5440590733584e6439f8e7b6657af80f20f48"
#SRCBRANCH = "release/0.3"

SRC_URI = "git://git@10.224.240.124:7999/em/eview_itf.git;protocol=ssh;tag=${SRCTAG}"
#SRC_URI = "git://git@10.224.240.124:7999/em/eview_itf.git;protocol=ssh;branch=${SRCBRANCH}"

S = "${WORKDIR}/git/"

# we need to pass the Cxx parameter extra to the make call
EXTRA_OEMAKE = "'CC=${CC}' 'AR=${AR}' 'CFLAGS=${CFLAGS}'"

do_compile() {
    oe_runmake all
}

do_install() {
    oe_runmake install DESTDIR=${D}
}

# Bypass error No GNU_HASH in the elf binary
INSANE_SKIP_${PN} = "ldflags"

RDEPENDS_${PN} = ""
RDEPENDS_${PN}-dev = ""
RDEPENDS_${PN}-staticdev = "${PN}-dev"

FILES_${PN} += "${bindir}"
FILES_${PN}-dev += "${includedir}"
FILES_${PN}-staticdev += "${libdir}"

