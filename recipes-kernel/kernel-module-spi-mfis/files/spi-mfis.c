/*
 * eSoftThings SPI/MFIS controller driver
 *
 * Copyright (C) 2020, eSoftThings - All Rights Reserved
 */

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/spi/spi.h>
#include <linux/completion.h>
#include <linux/io.h>

#include <linux/eviewitf-mfis.h>
#include <linux/eviewitf-mfis-driver.h>

#define SPIMFIS_DRV_NAME		"spi-mfis"

#define SPIMFIS_NUM_CS			4
#define SPIMFIS_RX_TIMEOUT_MS		100

typedef enum spimfis_msg_type
{
	SPIMFIS_MSG_TYPE_ECHO = 0x10,
	SPIMFIS_MSG_TYPE_INIT = 0x11,
	SPIMFIS_MSG_TYPE_MAPPING = 0x12,
	SPIMFIS_MSG_TYPE_CONFIG = 0x13,
	SPIMFIS_MSG_TYPE_READ = 0x14,
	SPIMFIS_MSG_TYPE_WRITE = 0x15,
	SPIMFIS_MSG_TYPE_WRITE_READ = 0x16,
	SPIMFIS_MSG_TYPE_READ_COMPLETE = 0x17,
	SPIMFIS_MSG_TYPE_ACK = 0x18,
	SPIMFIS_MSG_TYPE_RESETN = 0x19,
	SPIMFIS_MSG_TYPE_SYNC = 0x1A,
	SPIMFIS_MSG_TYPE_NACK = 0x1B,
} spimfis_msg_type_t;

/**
 * struct spimfis_master - private data of the SPI controller
 * @dev: driver model representation of the controller
 * @master: controller master interface
 * @mfis_rx_mutex: prevent concurrent access to MFIS reception buffer by
 *                 preventing concurrent SPI/MFIS transactions.
 *                 To allow concurrency on different SPI busses, rework the DT
 *                 to have spidev0.[01] and spidev1.[01] with each its own
 *                 spimfis_master. Only CS should be reworked in the driver.
 * @rx_complete: signal reception of SPI/MFIS message
 * @mfis_rx_buf: MFIS reception buffer
 * @mfis_rx_nb: MFIS message reception notifier block
 */
struct spimfis_master {
	struct device *dev;
	struct spi_master *master;

	struct mutex mfis_rx_mutex;
	uint32_t mfis_rx_buf[EVIEWITF_MFIS_MSG_SIZE];
	struct completion rx_complete;

	struct notifier_block mfis_rx_nb;
};

/**
 * struct spimfis_device - device-specific data
 * @tx_buf: SPI buffer to be written (shared memory)
 * @tx_buf_phys: physical address of SPI transmission buffer
 * @tx_buf_len: SPI transmission buffer size in bytes
 * @rx_buf: SPI buffer to be read (shared memory)
 * @rx_buf_phys: physical address of SPI reception buffer
 * @rx_buf_len: SPI reception buffer size in bytes
 */
struct spimfis_device {
	void *tx_buf;
	resource_size_t tx_buf_phys;
	uint16_t tx_buf_len;
	void *rx_buf;
	resource_size_t rx_buf_phys;
	uint16_t rx_buf_len;
};

/* A bit of wasted space if first index != 0 */
const char *spimfis_msg_type_str[] = {
	[SPIMFIS_MSG_TYPE_ECHO]			= "ECHO",
	[SPIMFIS_MSG_TYPE_INIT]			= "INIT",
	[SPIMFIS_MSG_TYPE_MAPPING]		= "MAPPING",
	[SPIMFIS_MSG_TYPE_CONFIG]		= "CONFIG",
	[SPIMFIS_MSG_TYPE_READ]			= "READ",
	[SPIMFIS_MSG_TYPE_WRITE]		= "WRITE",
	[SPIMFIS_MSG_TYPE_WRITE_READ]		= "WRITE_READ",
	[SPIMFIS_MSG_TYPE_READ_COMPLETE]	= "READ_COMPLETE",
	[SPIMFIS_MSG_TYPE_ACK]			= "ACK",
	[SPIMFIS_MSG_TYPE_RESETN]		= "RESETN",
	[SPIMFIS_MSG_TYPE_SYNC]			= "SYNC",
	[SPIMFIS_MSG_TYPE_NACK]			= "NACK",
};

static void _spimfis_format_header(uint32_t *mfis_buf,
				   spimfis_msg_type_t msg_type,
				   uint8_t chip_select);

static int _spimfis_send_msg(struct spi_device *spidev, uint32_t *mfis_buf);
static int _spimfis_expect_msg(struct spi_device *spidev,
			       spimfis_msg_type_t msg_type);
static int _spimfis_send_receive_msg(struct spi_device *spidev,
				     spimfis_msg_type_t msg_type,
				     spimfis_msg_type_t expected_msg_type,
				     uint32_t *mfis_buf,
				     struct spi_transfer *transfer);

static int _spimfis_send_init_msg(struct spi_device *spidev);
static int _spimfis_send_config_msg(struct spi_device *spidev,
				    uint16_t spi_mode,
				    uint32_t spi_max_speed_hz,
				    uint8_t spi_bits_per_word);
static int _spimfis_send_write_msg(struct spi_device *spidev,
				   struct spi_transfer *transfer);
static int _spimfis_send_write_read_msg(struct spi_device *spidev,
					struct spi_transfer *transfer);
static int _spimfis_send_read_msg(struct spi_device *spidev,
				  struct spi_transfer *transfer);

#ifdef ENABLE_ECHO
static int _spimfis_send_echo_msg(struct spi_device *spidev,
				  struct spi_transfer *transfer);
#endif

static int _spimfis_receive_msg_atomic(struct notifier_block *nb,
				       unsigned long action, void *data);
static int _spimfis_process_read_complete_msg(struct spi_device *spidev,
					      void *rx_buf,
					      size_t rx_len);
static int _spimfis_process_mapping_msg(struct spi_device *spidev);

/* Runs in interrupt context */
static int _spimfis_receive_msg_atomic(struct notifier_block *nb,
				       unsigned long action, void *data)
{
	struct spimfis_master *spimfis;
	int i;

	/* No locking because only one SPI/MFSI transaction at a time */

	spimfis = container_of(nb, struct spimfis_master, mfis_rx_nb);

	if (((uint32_t *)data)[0] != EVIEWITF_MFIS_FCT_MFIS_SPI)
		return NOTIFY_DONE;

	for (i = 0; i < EVIEWITF_MFIS_MSG_SIZE; i++)
		spimfis->mfis_rx_buf[i] = ((uint32_t *)data)[i];

	complete(&spimfis->rx_complete);

	return NOTIFY_STOP;
}

static void _spimfis_format_header(uint32_t *mfis_buf,
				   spimfis_msg_type_t msg_type,
				   uint8_t chip_select)
{
	mfis_buf[0] = EVIEWITF_MFIS_FCT_MFIS_SPI;

	// Reset bits 31-16
	mfis_buf[1] &= (uint32_t)GENMASK(15, 0);
	mfis_buf[1] |= ((uint32_t)msg_type) << 24;
	mfis_buf[1] |= ((uint32_t)chip_select) << 16;
}

static int _spimfis_send_msg(struct spi_device *spidev,
			     uint32_t *mfis_buf)
{
	struct spimfis_master *spimfis = spi_master_get_devdata(spidev->master);
	int ret;

	reinit_completion(&spimfis->rx_complete);

	ret = eviewitf_mfis_send_msg_driver(mfis_buf);
	if (ret) {
		dev_err(spimfis->dev, "Failed to send SPI/MFIS message\n");
		return ret;
	}

	return 0;
}

/* Receive next message and check its type */
static int _spimfis_expect_msg(struct spi_device *spidev,
			       spimfis_msg_type_t msg_type)
{
	struct spimfis_master *spimfis = spi_master_get_devdata(spidev->master);
	spimfis_msg_type_t rx_msg_type;
	uint8_t chip_select;
	int ret;

	/* Only SPI/MFIS messages will trigger this. */
	ret = wait_for_completion_interruptible_timeout(&spimfis->rx_complete,
			msecs_to_jiffies(SPIMFIS_RX_TIMEOUT_MS));
	if (ret == 0)
		return -ETIMEDOUT;
	if (ret < 0)
		return ret;

	rx_msg_type = (spimfis->mfis_rx_buf[1] & (uint32_t)GENMASK(31, 24)) >> 24;
	chip_select = (spimfis->mfis_rx_buf[1] & (uint32_t)GENMASK(23, 16)) >> 16;

	if (chip_select != spidev->chip_select) {
		dev_err(spimfis->dev, "Unexpected SPI/MFIS message recipient (chip select %u)\n", chip_select);
		return -EIO;
	}

	switch (rx_msg_type) {
	case SPIMFIS_MSG_TYPE_ACK:
	case SPIMFIS_MSG_TYPE_MAPPING:
	case SPIMFIS_MSG_TYPE_READ_COMPLETE:
	case SPIMFIS_MSG_TYPE_NACK:
		dev_dbg(spimfis->dev, "%s received\n", spimfis_msg_type_str[rx_msg_type]);
		break;
	default:
		dev_err(spimfis->dev, "Unexpected SPI/MFIS message %u received\n", rx_msg_type);
		return -EIO;
	}

	if (rx_msg_type != msg_type)
		return -EINVAL;

	return 0;
}

static int _spimfis_send_receive_msg(struct spi_device *spidev,
				     spimfis_msg_type_t msg_type,
				     spimfis_msg_type_t expected_msg_type,
				     uint32_t *mfis_buf,
				     struct spi_transfer *transfer)
{
	struct spimfis_master *spimfis = spi_master_get_devdata(spidev->master);
	int ret;

	dev_dbg(spimfis->dev, "%s(): sending %s message (chip select %u)\n",
		__func__, spimfis_msg_type_str[msg_type],  spidev->chip_select);

	_spimfis_format_header(mfis_buf, msg_type, spidev->chip_select);

	mutex_lock(&spimfis->mfis_rx_mutex);

	ret = _spimfis_send_msg(spidev, mfis_buf);
	if (ret) {
		dev_err(spimfis->dev, "Failed to send %s message\n", spimfis_msg_type_str[msg_type]);
		goto unlock;
	}

	ret = _spimfis_expect_msg(spidev, expected_msg_type);
	if (ret) {
		dev_err(spimfis->dev, "Failed to receive %s message\n", spimfis_msg_type_str[expected_msg_type]);
		goto unlock;
	}

	switch (expected_msg_type) {
	case SPIMFIS_MSG_TYPE_MAPPING:
		ret = _spimfis_process_mapping_msg(spidev);
		break;
	case SPIMFIS_MSG_TYPE_READ_COMPLETE:
		BUG_ON(transfer == NULL);
		ret = _spimfis_process_read_complete_msg(spidev, transfer->rx_buf, transfer->len);
		break;
	default:
		ret = 0;
		break;
	}

	if (ret) {
		dev_err(spimfis->dev, "Failed to process %s message\n", spimfis_msg_type_str[expected_msg_type]);
		goto unlock;
	}

unlock:
	mutex_unlock(&spimfis->mfis_rx_mutex);

	return ret;
}

static int _spimfis_send_init_msg(struct spi_device *spidev)
{
	uint32_t mfis_buf[EVIEWITF_MFIS_MSG_SIZE] = { 0 };

	return _spimfis_send_receive_msg(spidev, SPIMFIS_MSG_TYPE_INIT, SPIMFIS_MSG_TYPE_MAPPING, mfis_buf, NULL);
}

static int _spimfis_send_config_msg(struct spi_device *spidev,
				    uint16_t spi_mode,
				    uint32_t spi_max_speed_hz,
				    uint8_t spi_bits_per_word)
{
	uint32_t mfis_buf[EVIEWITF_MFIS_MSG_SIZE] = { 0 };

	mfis_buf[1] |= GENMASK(15, 0) & spi_mode;
	mfis_buf[2] = spi_max_speed_hz;
	mfis_buf[3] |= GENMASK(7, 0) & spi_bits_per_word;
	return _spimfis_send_receive_msg(spidev, SPIMFIS_MSG_TYPE_CONFIG, SPIMFIS_MSG_TYPE_ACK, mfis_buf, NULL);
}

static int _spimfis_send_write_msg(struct spi_device *spidev,
				   struct spi_transfer *transfer)
{
	struct spimfis_device *spimfis_dev = spi_get_ctldata(spidev);
	uint32_t mfis_buf[EVIEWITF_MFIS_MSG_SIZE] = { 0 };

	if (transfer->len > spimfis_dev->tx_buf_len)
		return -ENOMEM;

	memcpy(spimfis_dev->tx_buf, transfer->tx_buf, transfer->len);

	mfis_buf[1] |= GENMASK(15, 0) & transfer->delay_usecs;
	mfis_buf[2] = GENMASK(31, 31) & ((uint32_t)transfer->cs_change) << 31;
	mfis_buf[2] |= GENMASK(30, 0) & transfer->len;
	mfis_buf[3] = spimfis_dev->tx_buf_phys;

	return _spimfis_send_receive_msg(spidev, SPIMFIS_MSG_TYPE_WRITE, SPIMFIS_MSG_TYPE_ACK, mfis_buf, NULL);
}

static int _spimfis_send_write_read_msg(struct spi_device *spidev,
					struct spi_transfer *transfer)
{
	struct spimfis_device *spimfis_dev = spi_get_ctldata(spidev);
	uint32_t mfis_buf[EVIEWITF_MFIS_MSG_SIZE] = { 0 };

	if (transfer->len > spimfis_dev->tx_buf_len ||
	    transfer->len > spimfis_dev->rx_buf_len)
		return -ENOMEM;

	// TODO possible to get rid of memcpy's, i.e. replace the caller's pointers?
	memcpy(spimfis_dev->tx_buf, transfer->tx_buf, transfer->len);

	mfis_buf[1] |= GENMASK(15, 0) & transfer->delay_usecs;
	mfis_buf[2] = GENMASK(31, 31) & ((uint32_t)transfer->cs_change) << 31;
	mfis_buf[2] |= GENMASK(30, 0) & transfer->len;
	mfis_buf[3] = spimfis_dev->tx_buf_phys;

	return _spimfis_send_receive_msg(spidev, SPIMFIS_MSG_TYPE_WRITE_READ, SPIMFIS_MSG_TYPE_READ_COMPLETE, mfis_buf, transfer);
}

static int _spimfis_send_read_msg(struct spi_device *spidev,
				  struct spi_transfer *transfer)
{
	struct spimfis_device *spimfis_dev = spi_get_ctldata(spidev);
	uint32_t mfis_buf[EVIEWITF_MFIS_MSG_SIZE] = { 0 };

	if (transfer->len > spimfis_dev->rx_buf_len)
		return -ENOMEM;

	mfis_buf[1] |= GENMASK(15, 0) & transfer->delay_usecs;
	mfis_buf[2] = GENMASK(31, 31) & ((uint32_t)transfer->cs_change) << 31;
	mfis_buf[2] |= GENMASK(30, 0) & transfer->len;

	return _spimfis_send_receive_msg(spidev, SPIMFIS_MSG_TYPE_READ, SPIMFIS_MSG_TYPE_READ_COMPLETE, mfis_buf, transfer);
}

#ifdef ENABLE_ECHO
static int _spimfis_send_echo_msg(struct spi_device *spidev,
				  struct spi_transfer *transfer)
{
	struct spimfis_device *spimfis_dev = spi_get_ctldata(spidev);
	uint32_t mfis_buf[EVIEWITF_MFIS_MSG_SIZE] = { 0 };

	if (transfer->len > spimfis_dev->tx_buf_len)
		return -ENOMEM;

	memcpy(spimfis_dev->tx_buf, transfer->tx_buf, transfer->len);

	mfis_buf[1] = transfer->len;
	mfis_buf[2] = spimfis_dev->tx_buf_phys;

	return _spimfis_send_receive_msg(spidev, SPIMFIS_MSG_TYPE_ECHO, SPIMFIS_MSG_TYPE_ACK, mfis_buf, transfer);
}
#endif /* ENABLE_ECHO */

static int _spimfis_process_mapping_msg(struct spi_device *spidev)
{
	struct spimfis_master *spimfis = spi_master_get_devdata(spidev->master);
	struct spimfis_device *spimfis_dev = spi_get_ctldata(spidev);
	int ret;

	spimfis_dev->rx_buf_phys = spimfis->mfis_rx_buf[2];
	spimfis_dev->rx_buf_len = spimfis->mfis_rx_buf[3];

	spimfis_dev->tx_buf_phys = spimfis->mfis_rx_buf[4];
	spimfis_dev->tx_buf_len = spimfis->mfis_rx_buf[5];

	dev_dbg(spimfis->dev, "%s(): Rx phys addr: 0x%llX, %u\n", __func__,
		spimfis_dev->rx_buf_phys, spimfis_dev->rx_buf_len);
	dev_dbg(spimfis->dev, "%s(): Tx phys addr: 0x%llX, %u\n", __func__,
		spimfis_dev->tx_buf_phys, spimfis_dev->tx_buf_len);

	spimfis_dev->rx_buf = devm_ioremap(spimfis->dev, spimfis_dev->rx_buf_phys,
					   spimfis_dev->rx_buf_len);
	if (IS_ERR(spimfis_dev->rx_buf)) {
		ret = PTR_ERR(spimfis_dev->rx_buf);
		dev_err(spimfis->dev, "Failed to remap reception buffer\n");
		goto error;
	}

	spimfis_dev->tx_buf = devm_ioremap(spimfis->dev, spimfis_dev->tx_buf_phys,
					   spimfis_dev->tx_buf_len);
	if (IS_ERR(spimfis_dev->tx_buf)) {
		devm_iounmap(spimfis->dev, spimfis_dev->rx_buf);
		ret = PTR_ERR(spimfis_dev->tx_buf);
		dev_err(spimfis->dev, "Failed to remap transmission buffer\n");
		goto error;
	}

	return 0;

error:
	spimfis_dev->rx_buf_phys = 0;
	spimfis_dev->rx_buf_len = 0;
	spimfis_dev->rx_buf = NULL;
	spimfis_dev->tx_buf_phys = 0;
	spimfis_dev->tx_buf_len = 0;
	spimfis_dev->tx_buf = NULL;

	return ret;
}

static int _spimfis_process_read_complete_msg(struct spi_device *spidev,
					      void *rx_buf,
					      size_t rx_len)
{
	struct spimfis_master *spimfis = spi_master_get_devdata(spidev->master);
	struct spimfis_device *spimfis_dev = spi_get_ctldata(spidev);
	uint16_t rcm_len;
	uint32_t rcm_buf_phys;
	size_t offset;

	rcm_len = GENMASK(15, 0) & spimfis->mfis_rx_buf[1];
	rcm_buf_phys = spimfis->mfis_rx_buf[2];

	if (rcm_len > rx_len) {
		dev_err(spimfis->dev, "SPI reception buffer too small\n");
		return -ENOMEM;
	}

	/* Check physical buffer boundaries */
	if (rcm_buf_phys < spimfis_dev->rx_buf_phys ||
	    rcm_buf_phys > spimfis_dev->rx_buf_phys + spimfis_dev->rx_buf_len ||
	    rcm_buf_phys + rcm_len > spimfis_dev->rx_buf_phys + spimfis_dev->rx_buf_len) {
		dev_err(spimfis->dev, "SPI/MFIS reception buffer outside of mapped range for this device\n");
		return -EINVAL;
	}

	// Offset to compute virtual address of start of SPI reception buffer.
	offset = rcm_buf_phys - spimfis_dev->rx_buf_phys;

	memcpy(rx_buf, spimfis_dev->rx_buf + offset, rcm_len);

	return 0;
}

/**
 * spimfis_setup - setup device (called when adding a new device and on ioctls)
 */
static int spimfis_setup(struct spi_device *spidev)
{
	struct spimfis_device *spimfis_dev;
	int ret;

	/* Only alloc on first setup */
	spimfis_dev = spi_get_ctldata(spidev);
	if (spimfis_dev == NULL) {
		spimfis_dev = kzalloc(sizeof(*spimfis_dev), GFP_KERNEL);
		if (spimfis_dev == NULL)
			return -ENOMEM;

		spi_set_ctldata(spidev, spimfis_dev);

		ret = _spimfis_send_init_msg(spidev);
		if (ret) {
			kfree(spimfis_dev);
			spi_set_ctldata(spidev, NULL);
			return ret;
		}
	}

	dev_dbg(&spidev->dev, "%s(): mode = %u, max_speed_hz = %u, bits_per_word = %u\n", __func__,
		spidev->mode, spidev->max_speed_hz, spidev->bits_per_word);

	// TODO should these be cached/synchronized between devices?
	return _spimfis_send_config_msg(spidev, spidev->mode,
					spidev->max_speed_hz,
					spidev->bits_per_word);
}

/**
 * spimfis_cleanup - cleanup device
 */
static void spimfis_cleanup(struct spi_device *spidev)
{
	struct spimfis_device *spimfis_dev = spi_get_ctldata(spidev);

	if (spimfis_dev != NULL) {
		kfree(spimfis_dev);
		spi_set_ctldata(spidev, NULL);
	}
}

/**
 * spimfis_transfer_one - transfer a single spi_transfer
 *
 * It must return 0 if the transfer is finished or 1 if the transfer is still
 * in progress.
 */
static int spimfis_transfer_one(struct spi_master *master,
				struct spi_device *spidev,
				struct spi_transfer *transfer)
{
	struct spimfis_master *spimfis = spi_master_get_devdata(master);
	int ret = 0;

	if (transfer->len <= 0 || (transfer->tx_buf == NULL && transfer->rx_buf == NULL)) {
		dev_err(spimfis->dev, "Trying to send empty SPI/MFIS message\n");
		ret = -EINVAL;
		goto out;
	}

	if (transfer->tx_buf != NULL && transfer->rx_buf != NULL) {
		ret = _spimfis_send_write_read_msg(spidev, transfer);
		if (ret)
			goto out;
	} else if (transfer->tx_buf != NULL && transfer->rx_buf == NULL) {
		ret = _spimfis_send_write_msg(spidev, transfer);
		if (ret)
			goto out;
	} else if (transfer->rx_buf != NULL) {
		ret = _spimfis_send_read_msg(spidev, transfer);
		if (ret)
			goto out;
	}

#ifdef ENABLE_ECHO
	if (transfer->tx_buf != NULL) {
		ret = _spimfis_send_echo_msg(spidev, transfer);
		if (ret)
			goto out;
	}
#endif /* ENABLE_ECHO */

out:
	spi_finalize_current_transfer(master);

	return ret;
}

static const struct of_device_id spimfis_of_match[] = {
	{ .compatible = "spi-mfis" },
	{},
};
MODULE_DEVICE_TABLE(of, spimfis_of_match);

static int spimfis_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct spimfis_master *spimfis;
	int ret;

	master = spi_alloc_master(&pdev->dev, sizeof(struct spimfis_master));
	if (!master) {
		dev_err(&pdev->dev, "spi master allocation failed\n");
		ret = -ENOMEM;
		goto out;
	}
	platform_set_drvdata(pdev, master);

	spimfis = spi_master_get_devdata(master);
	spimfis->dev = &pdev->dev;
	spimfis->master = master;
	mutex_init(&spimfis->mfis_rx_mutex);
	init_completion(&spimfis->rx_complete);

	spimfis->mfis_rx_nb.notifier_call = _spimfis_receive_msg_atomic;
	ret = eviewitf_mfis_notifier_register(&spimfis->mfis_rx_nb);
	if (ret) {
		dev_err(&pdev->dev, "MFIS notifier registration failed\n");
		goto put_master;
	}

	master->dev.of_node = pdev->dev.of_node;
	master->auto_runtime_pm = true;
	master->bus_num = pdev->id;
	master->mode_bits = SPI_MODE_0; // TODO
	master->bits_per_word_mask = SPI_BPW_MASK(8) | SPI_BPW_MASK(16); // TODO
	master->max_speed_hz = 5000000; // TODO
	master->min_speed_hz = 500000; // TODO
	master->num_chipselect = SPIMFIS_NUM_CS;

	master->setup = spimfis_setup;
	master->cleanup = spimfis_cleanup;
	master->transfer_one = spimfis_transfer_one;

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	ret = devm_spi_register_master(&pdev->dev, master);
	if (ret) {
		dev_err(&pdev->dev, "spi master registration failed: %d\n",
			ret);
		goto rpm_disable;
	}

	dev_info(&pdev->dev, "driver initialized\n");

	return 0;

rpm_disable:
	pm_runtime_disable(&pdev->dev);

	eviewitf_mfis_notifier_unregister(&spimfis->mfis_rx_nb);

put_master:
	spi_master_put(master);

out:
	return ret;
}

static int spimfis_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct spimfis_master *spimfis = spi_master_get_devdata(master);

	pm_runtime_disable(&pdev->dev);

	eviewitf_mfis_notifier_unregister(&spimfis->mfis_rx_nb);

	dev_info(&pdev->dev, "driver removed\n");

	return 0;
}

#ifdef CONFIG_PM
static int spimfis_runtime_suspend(struct device *dev)
{
	return 0;
}

static int spimfis_runtime_resume(struct device *dev)
{
	return 0;
}
#endif

#ifdef CONFIG_PM_SLEEP
static int spimfis_suspend(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	int ret;

	ret = spi_master_suspend(master);
	if (ret)
		return ret;

	return pm_runtime_force_suspend(dev);
}

static int spimfis_resume(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	int ret;

	ret = pm_runtime_force_resume(dev);
	if (ret)
		return ret;

	ret = spi_master_resume(master);

	return ret;
}
#endif

static const struct dev_pm_ops spimfis_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(spimfis_suspend, spimfis_resume)
	SET_RUNTIME_PM_OPS(spimfis_runtime_suspend,
			   spimfis_runtime_resume, NULL)
};

static struct platform_driver spimfis_driver = {
	.probe = spimfis_probe,
	.remove = spimfis_remove,
	.driver = {
		.name = SPIMFIS_DRV_NAME,
		.pm = &spimfis_pm_ops,
		.of_match_table = spimfis_of_match,
	},
};

module_platform_driver(spimfis_driver);

MODULE_ALIAS(SPIMFIS_DRV_NAME);
MODULE_DESCRIPTION("SPI/MFIS controller driver module");
MODULE_AUTHOR("Hubert Chaumette <hchaumette@esoftthings.com>");
MODULE_LICENSE("GPL v2");

