
require recipes-eview/images/core-image-esoftthings-base.inc

IMAGE_INSTALL_append = " \
    kernel-module-eviewitf-mfis \
    initscript \
    mnt-ssd \
"

