require recipes-eview/images/core-image-esoftthings-base.inc

IMAGE_INSTALL_append = "initscript"
IMAGE_INSTALL_remove = "packagegroup-opencv-sdk packagegroup-surroundview-drm packagegroup-bsp-utest"
