SUMMARY = "libeviewitf"
SECTION = "libs"
LICENSE = "eSoftThings"
LIC_FILES_CHKSUM = "file://${WORKDIR}/git/LICENSE;md5=c6afb05c83ee3f78ca6877faf927aede"

SRCTAG = "0.1"
#SRCREV = "1e8f1f468186b043a7ba0d1f1debdc382ebc0238"
#SRCBRANCH = "master"

SRC_URI = "git://git@10.224.240.124:7999/em/eview_itf.git;protocol=ssh;tag=${SRCTAG}"
#SRC_URI = "git://git@10.224.240.124:7999/em/eview_itf.git;protocol=ssh;branch=${SRCBRANCH}"

S = "${WORKDIR}/git/"

# we need to pass the Cxx parameter extra to the make call
#EXTRA_OEMAKE = "'CC=${CC}' 'RANLIB=${RANLIB}' 'AR=${AR}' 'CFLAGS=${CFLAGS} 
#-I${S}/include' 'BUILDDIR=${S}' 'DESTDIR=${D}'"

#inherit autotools-brokensep

do_compile() {
    oe_runmake all 'CC=${CC}'
}

do_install() {
    install -d ${D}/usr/lib
    cp build/libmfis.a ${D}/usr/lib
    install -d ${D}/usr/include
    cp include/libmfis.h ${D}/usr/include
}