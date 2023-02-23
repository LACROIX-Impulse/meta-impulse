/**
 * @defgroup    app Application layer
 * @defgroup    app_mfis MFIS communication layer
 * @ingroup     app
 * @copyright   (c) 2019-2020 eSoftThings
 * @brief       MFIS (Multifunctional Interface) Shared interface between R7 and A53
 *
 * This file describe all enum and structures shared for communication beetwenn A53 and R7
 *
 * @{
 */
#ifndef _MFIS_SHARED_H
#define _MFIS_SHARED_H

/**
 * @brief Interface used for mfis request. Used to determine
 * the origin of the request.
 */
#define MFIS_SPACE_DRIVER   (0x01) /* Request from eviewitf-mfis */
#define MFIS_SPACE_USER     (0x02) /* Request from user space */
#define MFIS_SPACE_EXTERNAL (0x04) /* Request from driver-space (excluding eviewitf-mfis) */

/* Function is passed on 32b - 000000FF */
#define MFIS_REQUEST_GET_FCT_ID(fn) ((fn << 0) & 0xFF)
#define MFIS_REQUEST_SET_FCT_ID(fn) ((fn & 0xFF) << 0)

/* Permission is passed on 32b - 00FF0000 */
#define MFIS_REQUEST_GET_SPACE(perm) ((perm >> 16) & 0xFF)
#define MFIS_REQUEST_SET_SPACE(perm) ((perm & 0xFF) << 16)

#define MFIS_REQUEST_SET_CMD(fn, perm) (uint32_t)(MFIS_REQUEST_SET_FCT_ID(fn) | MFIS_REQUEST_SET_SPACE(perm))

/**
 * @brief MFIS function types
 */
typedef enum mfis_function_id
{
    FCT_INIT = 0,
    FCT_DEINIT = 1,
    FCT_INIT_END = 2,
    FCT_INIT_CAM = 3,
    FCT_INIT_BLENDING = 4,
    FCT_IOCTL = 5,
    FCT_CAM_GET_REGISTER = 10,
    FCT_CAM_SET_REGISTER = 11,
    FCT_CAM_RESET = 13,
    FCT_SET_HEARTBEAT = 20,
    FCT_SET_BOOT_MODE = 21,
    FCT_SET_DISPLAY = 22,
    FCT_SET_BLENDING = 23,
    FCT_SET_CROPPING = 24,
    FCT_GET_EVIEW_VERSION = 30,
    FCT_GET_MONITORING_INFO = 31,
    FCT_GET_BOOT_MODE = 32,
    FCT_UPDATE_STREAMER = 40,
    FCT_UPDATE_BLENDING = 41,
    FCT_MFIS_SPI = 50,
} mfis_function_id_t;

/**
 * @brief MFIS return codes
 */
typedef enum mfis_return_code
{
    FCT_RETURN_OK = 1,
    FCT_BLOCKED,   /* Operation not allowed */
    FCT_INV_PARAM, /* Invalid parameter */
    FCT_ERROR,     /* Generic failure */
} mfis_return_code_t;

#endif /* _MFIS_SHARED_H */
/* @} */
