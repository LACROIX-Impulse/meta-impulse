FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:"
COMPATIBLE_MACHINE_ecube = "ecube"

SRC_URI_append_ecube = " \
    file://0001-cert_header_sa3-update-for-eView.patch \
"
