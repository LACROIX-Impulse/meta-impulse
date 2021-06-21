FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:"

SRC_URI_append = " \
    file://0001-config-r8a77980-disable-MMC_HS200_SUPPORT.patch \
"

