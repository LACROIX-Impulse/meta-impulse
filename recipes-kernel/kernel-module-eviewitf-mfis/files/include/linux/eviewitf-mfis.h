/*
 * eviewitf-mfis.h
 *
 * Copyright (C) 2020 eSoftThings
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef EVIEWITF_MFIS_H
#define EVIEWITF_MFIS_H

/* MFIS number of message register */
#define EVIEWITF_MFIS_MSG_SIZE	8
#define EVIEWITF_MFIS_MAX_CAMERA 16
#define EVIEWITF_MFIS_MAX_REAL_CAMERA 8
#define EVIEWITF_MFIS_MAX_VIRTUAL_CAMERA 8
#define EVIEWITF_MFIS_MAX_BLENDING 2

/* Must be kept in sync with other eView components. */
typedef enum eviewitf_mfis_fct_id {
	EVIEWITF_MFIS_FCT_INIT					= 0,
	EVIEWITF_MFIS_FCT_DEINIT				= 1,
	EVIEWITF_MFIS_FCT_IOCTL					= 5,
	EVIEWITF_MFIS_FCT_CAM_GET_REGISTER		= 10,
	EVIEWITF_MFIS_FCT_CAM_SET_REGISTER		= 11,
	EVIEWITF_MFIS_FCT_CAM_SET_FPS			= 12,
	EVIEWITF_MFIS_FCT_CAM_RESET				= 13,
	EVIEWITF_MFIS_FCT_SET_HEARTBEAT			= 20,
	EVIEWITF_MFIS_FCT_SET_BOOT_MODE			= 21,
	EVIEWITF_MFIS_FCT_SET_DISPLAY			= 22,
	EVIEWITF_MFIS_FCT_SET_BLENDING 			= 23,
	EVIEWITF_MFIS_FCT_SET_CROPPING 			= 24,
	EVIEWITF_MFIS_FCT_GET_EVIEW_VERSION		= 30,
	EVIEWITF_MFIS_FCT_GET_MONITORING_INFO	= 31,
	EVIEWITF_MFIS_FCT_GET_BOOT_MODE			= 32,
	EVIEWITF_MFIS_FCT_MFIS_SPI				= 50,
}eviewitf_mfis_fct_id_t;

/* Function return codes */
struct enum eviewitf_mfis_return_code {
    EVIEWITF_MFIS_FCT_RETURN_OK = 1,
    EVIEWITF_MFIS_FCT_RETURN_BLOCKED,
    EVIEWITF_MFIS_FCT_INV_PARAM,
    EVIEWITF_MFIS_FCT_RETURN_ERROR,
}eviewitf_mfis_return_code_t;

/* Enumerate the different types of cameras possible */
typedef enum eviewitf_mfis_camera_type {
    EVIEWITF_MFIS_CAM_TYPE_NONE,
    EVIEWITF_MFIS_CAM_TYPE_GENERIC,
    EVIEWITF_MFIS_CAM_TYPE_VIRTUAL,
    EVIEWITF_MFIS_CAM_TYPE_SEEK,
}eviewitf_mfis_camera_type_t;

/* Structure to get the cameras attributes through an ioctl call */
typedef struct eviewitf_mfis_camera_attributes {
    uint8_t cam_type;
    uint32_t buffer_size;
    uint32_t width;
    uint32_t height;
    uint16_t dt;
}eviewitf_mfis_camera_attributes_t;

/* Structure to get the blending attributes through an ioctl call */
typdef struct eviewitf_mfis_blending_attributes {
    uint32_t buffer_size;
    uint32_t width;
    uint32_t height;
    uint16_t dt;
}eviewitf_mfis_blending_attributes_t;

/* Magic number for IOCTL */
#define EVIEWITF_MFIS_IOC_MAGIC	'a'

/* IOCTL definitions */
/* Start deprecated */
#define EVIEWITF_MFIS_WR_VALUE	_IOW(EVIEWITF_MFIS_IOC_MAGIC, 1, int32_t*)
#define EVIEWITF_MFIS_RD_VALUE	_IOR(EVIEWITF_MFIS_IOC_MAGIC, 2, int32_t*)
/* End deprecated */
#define EVIEWITF_MFIS_CAMERA_ATTRIBUTES		_IOR(EVIEWITF_MFIS_IOC_MAGIC, 3, eviewitf_mfis_camera_attributes_t*)
#define EVIEWITF_MFIS_BLENDING_ATTRIBUTES	_IOR(EVIEWITF_MFIS_IOC_MAGIC, 4, eviewitf_mfis_blending_attributes_t*)
#define EVIEWITF_MFIS_FCT	_IOWR(EVIEWITF_MFIS_IOC_MAGIC, 5, uint32_t*)

#endif /* EVIEWITF_MFIS_H */
