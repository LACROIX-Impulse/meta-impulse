FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:"

SRC_URI_append = " \
    file://0001-board-eSoftThings-Add-eCube-board.patch \
"
