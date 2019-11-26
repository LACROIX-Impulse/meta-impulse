SUMMARY = "libeviewitf"
SECTION = "libs"
LICENSE = "eSoftThings"
LIC_FILES_CHKSUM = "file://${WORKDIR}/git/LICENSE;md5=c6afb05c83ee3f78ca6877faf927aede"

# 0.2
SRCREV = "abe5dd6be5e94237e04750035c6305426a92636d"
BRANCH = "feature/EMIRROR-139-clone-eviewitf-adapt-eview-itf-to-integrate-mfis-api-in-meta-esoftthings"

#SRC_URI = "ssh://git@10.224.240.124:7999/em/eview_itf.git"
SRC_URI = "git://git@10.224.240.124:7999/em/eview_itf.git;protocol=ssh;branch=${BRANCH}"

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
    cp libmfis.a ${D}/usr/lib
}