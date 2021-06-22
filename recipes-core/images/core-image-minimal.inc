COMPATIBLE_MACHINE_ecube = "ecube"

require recipes-eview/images/core-image-esoftthings-base.inc

IMAGE_INSTALL_append = " \
    kernel-module-spi-mfis \
    kernel-module-eviewitf-mfis \
    initscript \
"

