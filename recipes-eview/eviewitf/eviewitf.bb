SUMMARY = "eviewitf"
SECTION = "libs"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${WORKDIR}/git/LICENSE;md5=fd1b89b0d4476c14237aaf827567f133"

PACKAGE_ARCH = "${MACHINE_ARCH}"

DEPENDS = "kernel-module-eviewitf-mfis"

SRCTAG = "1.6.0"
#SRCREV = "e07e0407292bea52d37b03294d8e4bc905fd57c8"
#SRCBRANCH = "develop"

SRC_URI = "git://github.com/LACROIX-Impulse/eview_itf.git;protocol=https;tag=${SRCTAG}"
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

