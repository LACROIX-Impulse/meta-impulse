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

#ifndef EVIEWITF_MFIS_DRIVER_H
#define EVIEWITF_MFIS_DRIVER_H

#include <linux/notifier.h>

int eviewitf_mfis_notifier_register(struct notifier_block *nb);
int eviewitf_mfis_notifier_unregister(struct notifier_block *nb);
int eviewitf_mfis_send_msg_driver(int32_t *mfis_buf);

#endif /* EVIEWITF_MFIS_DRIVER_H */
