SUMMARY = "eviewitf"
SECTION = "libs"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${WORKDIR}/git/LICENSE;md5=a63d66b53961a05540c4e46bd15d33da"

PACKAGE_ARCH = "${MACHINE_ARCH}"

DEPENDS = "kernel-module-eviewitf-mfis"

SRCTAG = "1.0.0"
#SRCREV = "efb9e33b1d8fa990df7778c5aae5027cb97a4ed2"
#SRCBRANCH = "feature/EMIRROR-836-eviewitf-update-license-file"

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

