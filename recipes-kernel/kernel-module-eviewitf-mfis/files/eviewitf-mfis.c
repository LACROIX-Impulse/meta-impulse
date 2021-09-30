/*
 * eSoftThings eViewItf module based on MFIS to communicate with Cortex R7
 *
 * Copyright (C) 2020 eSoftThings
 *
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/of_platform.h>
#include <linux/io.h>
#include <linux/cdev.h>
#include <linux/irqreturn.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <linux/spinlock.h>

#include "include/linux/eviewitf-mfis.h"
#include "mfis-shared.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* communication registers CONTROL */
#define MFISARIICR0_OFFSET	0x00000400
#define MFISAREICR0_OFFSET	0x00000404
#define MFISAREICR1_OFFSET	0x0000040C
/* communication registers MESSAGE */
#define MFISARIMBR0_OFFSET	0x00000440 /* A53 to R7 */
#define MFISAREMBR0_OFFSET	0x00000460 /* R7 to A53 */
#define MFIS_IRQ_COUNT		8

/* Device file definitions */
#define NB_IOCTL_FILE		1
#define NB_BLEND_FILE		EVIEWITF_MFIS_MAX_BLENDING
#define NB_REAL_CAM_FILE	EVIEWITF_MFIS_MAX_REAL_CAMERA
#define NB_VIRT_CAM_FILE	EVIEWITF_MFIS_MAX_VIRTUAL_CAMERA
#define NB_CAM_FILE			(NB_REAL_CAM_FILE + NB_VIRT_CAM_FILE)
#define NB_DEVICE_FILE		(NB_IOCTL_FILE + NB_BLEND_FILE + NB_CAM_FILE)

#define NB_CAM_BUFFER		3
#define NB_BLENDING_BUFFER	3

/* IRQ parsing definitions */
#define MFIS_GET_IRQ_TYPE(x)		((x & 0x6000) >> 13)
#define MFIS_SET_IRQ_TYPE(x, type)	(x = ((x & 0x9FFF) | (type << 13)))
#define MFIS_GET_IRQ_CAM_FRAME(x)	((x & 0x000E) >> 1)

#define MFIS_IRQ_CONFIG		0x0
#define MFIS_IRQ_CAM_FRAME	0x1
#define MFIS_IRQ_EXTERNAL	0x2

/* Others */
#define ASCII_OFFSET		48
#define READ_FIFO_SIZE		20

#define EVIEWITF_MFIS_SET_FCT_ID(x, fct) (x = ((x & 0xFFFFFF00) | (fct & 0x000000FF)))
#define EVIEWITF_MFIS_SET_FCT_SPACE(x, space)  (x = ((x & 0xFF00FFFF) | ((space & 0x000000FF) << 16)))

struct eviewitf_mfis_camera
{
	uint8_t cam_type;
	uint32_t buffer_size;
	uint32_t width;
	uint32_t height;
	uint16_t dt; /* datatype: ex: ARGB */
	void* buffer_address[NB_BLENDING_BUFFER];
};

struct eviewitf_mfis_blending
{
	uint32_t buffer_size;
	uint32_t width;
	uint32_t height;
	uint16_t dt; /* datatype: ex: ARGB */
	void* buffer_address[NB_CAM_BUFFER];
};

/*******************************************************************************
 * Global variables
 ******************************************************************************/
/* IOCTL global variables */
static dev_t dev;
static struct cdev c_dev[NB_DEVICE_FILE];
static struct class *cl;
static int open_config_count;
/* MFIS communication variables */
static u32 __iomem *mfis_base_addr; /* Base address of peripheral */

static uint32_t val_read[READ_FIFO_SIZE][EVIEWITF_MFIS_MSG_SIZE];
static uint32_t val_read_head;
static uint32_t val_read_tail;
static uint32_t val_read_count;
static char cam_last_frame[NB_REAL_CAM_FILE];
static char cam_irq_count[NB_REAL_CAM_FILE];

static unsigned boot_mode = 0;
module_param(boot_mode, uint, S_IRUGO);
MODULE_PARM_DESC(boot_mode, "boot mode transmitted to Cortex R7 when probbing module");

/* Notifier chain for external MFIS reception */
static ATOMIC_NOTIFIER_HEAD(mfis_receive_notifier);

/* Spin locks and Mutexes */
static DEFINE_MUTEX(send_mutex);
static DEFINE_MUTEX(ioctl_mutex);
static DEFINE_SPINLOCK(fifo_spinlock); /* Protect access to variables used by the message read FIFO */

/* Wait Queues */
static struct wait_queue_head wait_queue_cam_it[NB_REAL_CAM_FILE];
static char wait_queue_cam_flag[NB_REAL_CAM_FILE];
static struct wait_queue_head wait_queue_config_it;

static struct eviewitf_mfis_camera cameras[NB_CAM_FILE];
static struct eviewitf_mfis_blending blendings[NB_BLEND_FILE];
/* Mapping of platform IRQ id/MFIS IRQ number */
static int mfis_irq_id[MFIS_IRQ_COUNT];

static int _eviewitf_mfis_send_msg(uint32_t *val);
static int eviewitf_mfis_send_msg_internal(uint32_t *val);
static void mfis_get_msg_config(void);

static void mfis_unlock_cam_read(int cam_id, uint32_t irq_reg);
static irqreturn_t mfis_handle_irq(int pirq_id, void *ident);
static int mfis_check_R7_irq_state(void);

static long eviewitf_mfis_config_ioctl(struct file *f, unsigned int cmd, unsigned long arg);
static ssize_t eviewitf_mfis_cam_read(struct file *filep, char *buffer, size_t len, loff_t *offset);
static ssize_t eviewitf_mfis_cam_write(struct file *filep, const char *buffer, size_t len, loff_t *offset);
static ssize_t eviewitf_mfis_blend_write(struct file *filep, const char *buffer, size_t len, loff_t *offset);
static unsigned int eviewitf_mfis_cam_poll(struct file *filp, struct poll_table_struct *wait);

static int __init eviewitf_mfis_init(void);
static void __exit eviewitf_mfis_exit(void);

static ssize_t mfis_read_msg_config_fifo(uint32_t *buffer, size_t len)
{
	size_t i;

	if (len > EVIEWITF_MFIS_MSG_SIZE) {
		return -EINVAL;
	}

	spin_lock_irq(&fifo_spinlock);
	if (wait_event_interruptible_lock_irq(wait_queue_config_it, val_read_count, fifo_spinlock) != 0) {
		return -ERESTARTSYS;
	}

	for (i = 0; i < len; i++) {
		buffer[i] = val_read[val_read_tail][i];
	}

	val_read_count--;
	val_read_tail = (val_read_tail + 1) % READ_FIFO_SIZE;
	spin_unlock_irq(&fifo_spinlock);
	return (ssize_t)len;
}

static ssize_t mfis_write_msg_config_fifo(uint32_t *buffer, size_t len)
{
	size_t i;

	if (len > EVIEWITF_MFIS_MSG_SIZE) {
		return -EINVAL;
	}

	spin_lock(&fifo_spinlock);
	if (val_read_count == READ_FIFO_SIZE) {
		spin_unlock(&fifo_spinlock);
		return -ENOMEM;
	}

	for (i = 0; i < len; i++) {
		val_read[val_read_head][i] = buffer[i];
	}

	val_read_count++;
	val_read_head = (val_read_head + 1) % READ_FIFO_SIZE;
	spin_unlock(&fifo_spinlock);

	wake_up_interruptible(&wait_queue_config_it);

	return (ssize_t)len;
}

/*******************************************************************************
 *
 * Function Name: mfis_set_boot_mode
 *
 * Purpose:
 *
 * Input Parameters:     N/A
 *
 * Output Parameters:    N/A
 *
 * Comment: None
 *
 ******************************************************************************/
static int mfis_set_boot_mode(void)
{
	uint32_t val_write[EVIEWITF_MFIS_MSG_SIZE];
	uint32_t val_read[EVIEWITF_MFIS_MSG_SIZE];

	val_write[0] = FCT_SET_BOOT_MODE;
	val_write[1] = boot_mode;
	/* Send message to R7 to start init procedure */
	eviewitf_mfis_send_msg_internal(val_write);
    mfis_read_msg_config_fifo(val_read, EVIEWITF_MFIS_MSG_SIZE);

	if( val_read[1] != FCT_RETURN_OK) {
		pr_debug("Cannot set boot mode %d\n", boot_mode);
	}
	else {
		pr_debug("Boot mode %d\n", boot_mode);
	}

	return 0;
}

/*******************************************************************************
 *
 * Function Name: mfis_retrieve_mapping
 *
 * Purpose:
 *
 * Input Parameters:     N/A
 *
 * Output Parameters:    N/A
 *
 * Comment: None
 *
 ******************************************************************************/
static int mfis_retrieve_mapping(void)
{
	int i;
	uint32_t val_write[EVIEWITF_MFIS_MSG_SIZE];
	uint32_t val_read[EVIEWITF_MFIS_MSG_SIZE];
	int init_done = 0;

	val_write[0] = FCT_INIT;
	/* Send message to R7 to start init procedure */
	eviewitf_mfis_send_msg_internal(val_write);

	while ( init_done == 0 ) {
		/* Read next message from eView */
		mfis_read_msg_config_fifo(val_read, EVIEWITF_MFIS_MSG_SIZE);

		switch ( MFIS_REQUEST_GET_FCT_ID(val_read[0]) ) {
			case FCT_INIT:
				if( val_read[1] != FCT_RETURN_OK) {
					return -EFAULT;
				}
				break;
			case FCT_INIT_CAM: {
				/* Received parameters for one of the cameras */
				uint8_t cam_id = val_read[1] & 0xFF;
				if ( cam_id < NB_CAM_FILE ) {
					cameras[cam_id].cam_type = (val_read[1] & 0x0000FF00) >> 8;
					cameras[cam_id].dt = (val_read[1] & 0xFFFF0000) >> 16;
					cameras[cam_id].width = val_read[2];
					cameras[cam_id].height = val_read[3];
					cameras[cam_id].buffer_size = val_read[7];
					for ( i = 0; i < NB_CAM_BUFFER; i++) {
						/* If we try to met a null address everything will crash */
						if (val_read[4+i] != 0) {
							cameras[cam_id].buffer_address[i] = memremap(val_read[4+i], cameras[cam_id].buffer_size, MEMREMAP_WB);
						}
						pr_debug("Camera %d, physical address %x, virtual address %p of size %d\n",
							cam_id, val_read[4+i], cameras[cam_id].buffer_address[i], cameras[cam_id].buffer_size);
					}
				}
				break;
			}
			case FCT_INIT_BLENDING: {
				/* Received parameters for one of the blendings */
				uint8_t blending_id = val_read[1] & 0xFF;
				if ( blending_id < NB_BLEND_FILE ) {
					blendings[blending_id].dt = (val_read[1] & 0xFFFF0000) >> 16;
					blendings[blending_id].width = val_read[2];
					blendings[blending_id].height = val_read[3];
					blendings[blending_id].buffer_size = val_read[7];
					for ( i = 0; i < NB_BLENDING_BUFFER; i++) {
						/* If we try to met a null address everything will crash */
						if (val_read[4+i] != 0) {
							blendings[blending_id].buffer_address[i] = memremap(val_read[4+i], blendings[blending_id].buffer_size, MEMREMAP_WB);
						}
						pr_debug("Blending %d, physical address %x, virtual address %p of size %d\n",
							blending_id, val_read[4+i], blendings[blending_id].buffer_address[i], blendings[blending_id].buffer_size);
					}
				}
				break;
			}
			case FCT_INIT_END:
				/* No more init messages to be received */
				init_done = 1;
				break;
		}
	}

	return 0;
}

/*******************************************************************************
 *
 * Function Name: _eviewitf_mfis_send_msg
 *
 * Purpose: Write message in MFIS message registers and raise IRQ_1 for R7
 *
 * Input Parameters: - uint32_t* val: array of val to send to R7
 *
 * Output Parameters:    N/A
 *
 * Comment: Private function.
 *
 ******************************************************************************/
static int _eviewitf_mfis_send_msg(uint32_t *val)
{
	int i;
	void *addr;

	/* Can be called from several devices */
	if (mutex_lock_interruptible(&send_mutex) != 0) {
		return -EINTR;
	}
	/* Wait for previous IRQ to be handled */
	while (mfis_check_R7_irq_state() != 0) {
		/* busy wait */
	}
	/* Fill 8 message registers */
	for (i = 0; i < EVIEWITF_MFIS_MSG_SIZE; i++) {
		addr = (void __force *)mfis_base_addr + MFISARIMBR0_OFFSET + sizeof(u32) * i;
		iowrite32(val[i], (void __iomem *)addr);
	}

	/* Send IRQ 1 to R7 */
	addr = (void __force *)mfis_base_addr + MFISARIICR0_OFFSET;
	iowrite32(1, (void __iomem *)addr);
	mutex_unlock(&send_mutex);

	return 0;
}

/*******************************************************************************
 *
 * Function Name: eviewitf_mfis_send_msg_internal
 *
 * Purpose: Write message from this driver in MFIS message registers and raise IRQ_1 for R7
 *
 * Input Parameters: - uint32_t* val: array of val to send to R7
 *
 * Output Parameters:    N/A
 *
 * Comment: Public function.
 *
 ******************************************************************************/
static int eviewitf_mfis_send_msg_internal(uint32_t *val)
{
	EVIEWITF_MFIS_SET_FCT_SPACE(val[0], MFIS_SPACE_DRIVER);
	return _eviewitf_mfis_send_msg(val);
}

/*******************************************************************************
 *
 * Function Name: eviewitf_mfis_send_msg_driver
 *
 * Purpose: Write message from external driver in MFIS message registers and raise IRQ_1 for R7
 *
 * Input Parameters: - uint32_t* val: array of val to send to R7
 *
 * Output Parameters:    N/A
 *
 * Comment: Public function.
 *
 ******************************************************************************/
int eviewitf_mfis_send_msg_driver(uint32_t *val)
{
	EVIEWITF_MFIS_SET_FCT_SPACE(val[0], MFIS_SPACE_EXTERNAL);
	return _eviewitf_mfis_send_msg(val);
}
EXPORT_SYMBOL(eviewitf_mfis_send_msg_driver);

/*******************************************************************************
 *
 * Function Name: eviewitf_mfis_send_msg
 *
 * Purpose: Write message from user space in MFIS message registers and raise IRQ_1 for R7
 *
 * Input Parameters: - uint32_t* val: array of val to send to R7
 *
 * Output Parameters:    N/A
 *
 * Comment: Public function.
 *
 ******************************************************************************/
int eviewitf_mfis_send_msg(uint32_t *val)
{
	EVIEWITF_MFIS_SET_FCT_SPACE(val[0], MFIS_SPACE_USER);
	return _eviewitf_mfis_send_msg(val);
}

/*******************************************************************************
 *
 * Function Name: mfis_get_msg_config
 *
 * Purpose: Copy MFIS message registers in rx buffer and raise flag
 *
 * Input Parameters:     N/A
 *
 * Output Parameters:    N/A
 *
 * Comment: None
 *
 ******************************************************************************/
static void mfis_get_msg_config(void)
{
	int i;
	void *addr;
	uint32_t buffer[EVIEWITF_MFIS_MSG_SIZE];

	/* Get 8 message registers */
	for (i = 0; i < EVIEWITF_MFIS_MSG_SIZE; i++) {
		addr = (void __force *)mfis_base_addr + MFISAREMBR0_OFFSET + sizeof(u32) * i;

		buffer[i] = (uint32_t) ioread32((void __iomem *)addr);
	}
	mfis_write_msg_config_fifo(buffer, EVIEWITF_MFIS_MSG_SIZE);
}

/*******************************************************************************
 *
 * Function Name: mfis_unlock_cam_read
 *
 * Purpose: Unlock the camera read() working queues
 *
 * Input Parameters: - int cam_id : camera ID to unlock
 *                   - uint32_t irq_reg : irq register to parse
 *
 * Output Parameters:    N/A
 *
 * Comment: None
 *
 ******************************************************************************/
static void mfis_unlock_cam_read(int cam_id, uint32_t irq_reg)
{
	/* Update ID of the last frame received and increase IRQ couonter*/
	cam_last_frame[cam_id] = (char)(MFIS_GET_IRQ_CAM_FRAME(irq_reg));
	cam_irq_count[cam_id]++;

	wait_queue_cam_flag[cam_id] = 1;
	wake_up_interruptible(&wait_queue_cam_it[cam_id]);
}

/*******************************************************************************
 *
 * Function Name: mfis_handle_irq
 *
 * Purpose: Handle all IRQs from R7
 *
 * Input Parameters:     - int pirq_id : Platform IRQ ID to handle
 *                       - void *ident : Cookie used in request_irq
 *
 * Output Parameters:    N/A
 *
 * Comment: None
 *
 ******************************************************************************/
static irqreturn_t mfis_handle_irq(int pirq_id, void *ident)
{
	void *addr;
	uint32_t irq_reg;
	int irq_id;
	int i;
	uint32_t notifier_buf[EVIEWITF_MFIS_MSG_SIZE] = { 0 };

	if (strcmp(THIS_MODULE->name, ident) != 0)
		return IRQ_NONE;

	for (irq_id = 0; irq_id < MFIS_IRQ_COUNT; irq_id++)
		if (mfis_irq_id[irq_id] == pirq_id)
			break;
	if (irq_id >= MFIS_IRQ_COUNT)
		return IRQ_NONE;

	/* Read IRQ register.
	IRQ registers of R7 and A53 are interlaced (so we multiply by 2). */
	addr = (void __force *)mfis_base_addr + MFISAREICR0_OFFSET + (sizeof(u32) * irq_id * 2);
	irq_reg = (uint32_t) ioread32((void __iomem *)addr);

	/* Parse and handle IRQ */
	switch (MFIS_GET_IRQ_TYPE(irq_reg)) {
	case MFIS_IRQ_CONFIG:
		/* New config message received : copy it in RX buffer */
		mfis_get_msg_config();
		break;

	case MFIS_IRQ_CAM_FRAME:
		/* New camera frame received : unlock the read() */
		mfis_unlock_cam_read(irq_id, irq_reg);
		break;

	case MFIS_IRQ_EXTERNAL:
		/* Always signal external modules. Callee should check the first word to see whether he is concerned. */
		for (i = 0; i < EVIEWITF_MFIS_MSG_SIZE; i++) {
			addr = (void __force *)mfis_base_addr + MFISAREMBR0_OFFSET + i * sizeof(notifier_buf[0]);
			notifier_buf[i] = (uint32_t)ioread32((void __iomem *)addr);
		}
		/* Clean space info before sending */
		EVIEWITF_MFIS_SET_FCT_SPACE(notifier_buf[0], 0);
		atomic_notifier_call_chain(&mfis_receive_notifier, 0, notifier_buf);
		break;

	default:
		break;
	}

	/* Clear IRQ from R7 */
	addr = (void __force *)mfis_base_addr + MFISAREICR0_OFFSET + (sizeof(u32) * irq_id * 2);
	iowrite32(0, (void __iomem *)addr);

	return IRQ_HANDLED;
}

/*******************************************************************************
 *
 * Function Name: mfis_check_R7_irq_state
 *
 * Purpose: Return state of IRQ1 on R7 side.
 *
 * Input Parameters:     N/A
 *
 * Output Parameters:    N/A
 *
 * Comment: None
 *
 ******************************************************************************/
static int mfis_check_R7_irq_state(void)
{
	void *addr;

	/* Return IRQ state on R7 side (if 1 it means IRQ has not been handled by R7) */
	addr = (void __force *)mfis_base_addr + MFISARIICR0_OFFSET;
	return (int) ioread32((void __iomem *)addr);
}

/*******************************************************************************
 *
 * Function Name: eviewitf_mfis_notifier_register
 *
 * Purpose:              Register external reception handler.
 *
 * Input Parameters:     struct notifier_block *nb: callback inside
 *
 * Output Parameters:    N/A
 *
 * Comment:              The callback shall not block
 *
 ******************************************************************************/
int eviewitf_mfis_notifier_register(struct notifier_block *nb)
{
	return atomic_notifier_chain_register(&mfis_receive_notifier, nb);
}
EXPORT_SYMBOL(eviewitf_mfis_notifier_register);

/*******************************************************************************
 *
 * Function Name: eviewitf_mfis_notifier_unregister
 *
 * Purpose:              Unregister external reception handler.
 *
 * Input Parameters:     struct notifier_block *nb: callback inside
 *
 * Output Parameters:    N/A
 *
 * Comment:              None
 *
 ******************************************************************************/
int eviewitf_mfis_notifier_unregister(struct notifier_block *nb)
{
	return atomic_notifier_chain_unregister(&mfis_receive_notifier, nb);
}
EXPORT_SYMBOL(eviewitf_mfis_notifier_unregister);

/*******************************************************************************
 *
 * Function Name: eviewitf_mfis_config_open/close
 *
 * Purpose:
 *
 * Input Parameters:     N/A
 *
 * Output Parameters:    N/A
 *
 * Comment: None
 *
 ******************************************************************************/
static int eviewitf_mfis_config_open(struct inode *i, struct file *f)
{
	if (mutex_lock_interruptible(&ioctl_mutex) != 0) {
		return -EINTR;
	}
	if (open_config_count) {
		mutex_unlock(&ioctl_mutex);
		/* Already open */
		return -EBUSY;
	}
	open_config_count++;
	mutex_unlock(&ioctl_mutex);
	return 0;
}

static int eviewitf_mfis_config_close(struct inode *i, struct file *f)
{
	if (mutex_lock_interruptible(&ioctl_mutex) != 0) {
		return -EINTR;
	}
	/* Release the device */
	open_config_count--;
	mutex_unlock(&ioctl_mutex);
	return 0;
}

/*******************************************************************************
 *
 * Function Name: eviewitf_mfis_config_ioctl
 *
 * Purpose: Handle IOCTL handler
 *
 * Input Parameters:     struct file *f device file concrned by the IOCTL command
 *                       unsigned int cmd IOCTL command
 *                       unsigned long arg IN/OUT IOCTL argument
 *
 * Output Parameters:    N/A
 *
 * Comment: None
 *
 ******************************************************************************/
static long eviewitf_mfis_config_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	int i;
	uint32_t message[EVIEWITF_MFIS_MSG_SIZE];

	/* Check magic number */
	if (_IOC_TYPE(cmd) != EVIEWITF_MFIS_IOC_MAGIC) return -ENOTTY;

	switch (cmd) {
	/* Start deprecated */
	case EVIEWITF_MFIS_WR_VALUE:
		/* Get message from user space */
		if (copy_from_user(message, (int32_t *)arg, sizeof(message))) {
			return -EACCES;
		}

		/* Send message to R7, add MFIS_SPACE_USER info */
		eviewitf_mfis_send_msg(message);

		break;
	case EVIEWITF_MFIS_RD_VALUE:
		/* Wait for IRQ from R7 */
		mfis_read_msg_config_fifo(message, EVIEWITF_MFIS_MSG_SIZE);
		/* Clear space info */
		EVIEWITF_MFIS_SET_FCT_SPACE(message[0], 0);

		/* Send message to user space */
		if (copy_to_user((int32_t *)arg, message, sizeof(message))) {
			return -EACCES;
		}

		break;
	/* Stop deprecated */
	case EVIEWITF_MFIS_FCT:
		/* Get message from user space */
		if (copy_from_user(message, (uint32_t *)arg, sizeof(message))) {
			return -EACCES;
		}

		/* Send message to R7, add MFIS_SPACE_USER info */
		eviewitf_mfis_send_msg(message);

		/* Wait for IRQ from R7 */
		mfis_read_msg_config_fifo(message, EVIEWITF_MFIS_MSG_SIZE);
		/* Clear space info */
		EVIEWITF_MFIS_SET_FCT_SPACE(message[0], 0);

		/* Send message to user space */
		if (copy_to_user((uint32_t *)arg, message, sizeof(message))) {
			return -EACCES;
		}

		break;
	case EVIEWITF_MFIS_CAMERA_ATTRIBUTES: {
		struct eviewitf_mfis_camera_attributes* attributes = (struct eviewitf_mfis_camera_attributes*) arg;

		for (i = 0; i < NB_CAM_FILE; i++) {
			attributes[i].cam_type = cameras[i].cam_type + 1; /* mfis to eviewitf-mfis convertion */
			attributes[i].buffer_size = cameras[i].buffer_size;
			attributes[i].width = cameras[i].width;
			attributes[i].height = cameras[i].height;
			attributes[i].dt = cameras[i].dt;
			if( attributes[i].buffer_size == 0 ) {
				attributes[i].cam_type = EVIEWITF_MFIS_CAM_TYPE_NONE;
			}
		}
		break;
	}
	case EVIEWITF_MFIS_BLENDING_ATTRIBUTES: {
		struct eviewitf_mfis_blending_attributes* attributes = (struct eviewitf_mfis_blending_attributes*) arg;

		for (i = 0; i < NB_BLEND_FILE; i++) {
			attributes[i].buffer_size = blendings[i].buffer_size;
			attributes[i].width = blendings[i].width;
			attributes[i].height = blendings[i].height;
			attributes[i].dt = blendings[i].dt;
		}
		break;
	}
	default:
		return -EINVAL;
	}

	return 0;
}

/*******************************************************************************
 *
 * Function Name: eviewitf_mfis_cam_open/close
 *
 * Purpose:
 *
 * Input Parameters:     N/A
 *
 * Output Parameters:    N/A
 *
 * Comment: None
 *
 ******************************************************************************/
static int eviewitf_mfis_cam_open(struct inode *i, struct file *f)
{
	return 0;
}

static int eviewitf_mfis_cam_close(struct inode *i, struct file *f)
{
	return 0;
}

/*******************************************************************************
 *
 * Function Name: eviewitf_mfis_cam_read
 *
 * Purpose:
 *
 * Input Parameters:     N/A
 *
 * Output Parameters:    N/A
 *
 * Comment: None
 *
 ******************************************************************************/
static ssize_t eviewitf_mfis_cam_read(struct file *filep, char *buffer, size_t len, loff_t *offset)
{
	/* file iminor is used to determine wich cam file is called.
	   The cam file numbers start at 0. It is defined in the probe fct */
	int cam_read_id = iminor(filep->f_inode);
	int last_frame = cam_last_frame[cam_read_id];
	int uncopyed_bytes = 0;
	size_t checked_len = 0;
	wait_queue_cam_flag[cam_read_id] = 0;
	/* Prepare TX buffer with last cam ID and number of IRQ since last read */
	if (len == 0) {
		return -EINVAL;
	}

	if ((last_frame < 0) || (last_frame >= NB_CAM_BUFFER)) {
		pr_err("Virtual camera buffer does not exist\n");
		return -EACCES;
	}

	if (cameras[cam_read_id].buffer_address[last_frame] == 0) {
		return -EACCES;
	}

	/* Do not copy data outside buffer */
	if (len > (cameras[cam_read_id].buffer_size - *offset)) {
		checked_len = cameras[cam_read_id].buffer_size - *offset;
	} else  {
		checked_len = len;
	}

	/* Copy usig current offset */
	uncopyed_bytes = copy_to_user(buffer, (void *)(cameras[cam_read_id].buffer_address[last_frame] + *offset), checked_len);

	/* Change offset */
	*offset = *offset + checked_len - uncopyed_bytes;

	/* Compute real uncopyed_bytes */
	uncopyed_bytes += len - checked_len;

	return (ssize_t)(len - uncopyed_bytes);
}

/*******************************************************************************
 *
 * Function Name: eviewitf_mfis_cam_write
 *
 * Purpose: Write in a virtual camera
 *
 * Input Parameters:     filep
 *                         Char device file
 *                       buffer
 *                         Buffer of data from the user
 *                       len
 *                         Length to write
 *                       offset
 *                         Offset to start the write (unused here)
 *
 * Output Parameters:    N/A
 *
 * Return:               Length wrote (in bytes)
 *
 * Comment: None
 *
 ******************************************************************************/
static ssize_t eviewitf_mfis_cam_write(struct file *filep, const char *buffer, size_t len, loff_t *offset)
{
	int cam_read_id;
	static int buffer_ids[NB_CAM_FILE - NB_VIRT_CAM_FILE] = {0};
	int buffer_id;
	uint32_t val_write[EVIEWITF_MFIS_MSG_SIZE];
	int uncopyed_bytes = 0;
	size_t checked_len = 0;

	if (len == 0) {
		return -EINVAL;
	}

	/* File iminor is used to determine wich cam file is called */
	cam_read_id = iminor(filep->f_inode);

	/* Only the virtual cameras should be writable (between NB_VIRT_CAM_FILE and NB_CAM_FILE) */
	if ((NB_REAL_CAM_FILE > cam_read_id) || (NB_CAM_FILE <= cam_read_id)) {
		pr_err("Only the virtual cameras are writable\n");
		return -EACCES;
	}

	/* Check buffer ID */
	buffer_id = buffer_ids[cam_read_id - NB_REAL_CAM_FILE];
	if ((buffer_id < 0) || (buffer_id >= NB_CAM_BUFFER)) {
		pr_err("Virtual camera buffer does not exist\n");
		return -EACCES;
	}

	/* Check if the buffer exists */
	if (cameras[cam_read_id].buffer_address[buffer_id] == 0) {
		pr_err("Virtual camera buffer does not exist\n");
		return -EACCES;
	}

	/* Do not write data outside buffer */
	if (len > (cameras[cam_read_id].buffer_size - *offset)) {
		checked_len = cameras[cam_read_id].buffer_size - *offset;
	} else  {
		checked_len = len;
	}

	uncopyed_bytes = copy_from_user(cameras[cam_read_id].buffer_address[buffer_id] + *offset, (void *)buffer, checked_len);

	/* Compute real uncopyed_bytes */
	uncopyed_bytes += len - checked_len;

	/* Send a MFIS message to indicate eView that a new frame as been written (with cam ID and buffer ID) */
	val_write[0] = FCT_UPDATE_STREAMER;
	val_write[1] = cam_read_id;
	val_write[2] = buffer_id;
	eviewitf_mfis_send_msg_internal(val_write);

	/* Set offset back to 0 as we flip the current buffer */
	*offset = 0;

	/* Update the buffer id */
	buffer_ids[cam_read_id - NB_REAL_CAM_FILE]++;
	if (NB_CAM_BUFFER == buffer_ids[cam_read_id - NB_REAL_CAM_FILE]) {
		buffer_ids[cam_read_id - NB_REAL_CAM_FILE] = 0;
	}

	/* Return the number of bytes written */
	return (len - uncopyed_bytes);
}

/*******************************************************************************
 *
 * Function Name: eviewitf_mfis_blend_open
 *
 * Purpose: Open function
 *
 * Input Parameters:     N/A
 *
 * Output Parameters:    N/A
 *
 * Comment: None
 *
 ******************************************************************************/
static int eviewitf_mfis_blend_open(struct inode *i, struct file *f)
{
	/* Nothing to do */
	return 0;
}

/*******************************************************************************
 *
 * Function Name: eviewitf_mfis_blend_close
 *
 * Purpose: Close function
 *
 * Input Parameters:     N/A
 *
 * Output Parameters:    N/A
 *
 * Comment: None
 *
 ******************************************************************************/
static int eviewitf_mfis_blend_close(struct inode *i, struct file *f)
{
	/* Nothing to do */
	return 0;
}

/*******************************************************************************
 *
 * Function Name: eviewitf_mfis_blend_write
 *
 * Purpose: Write in the blending
 *
 * Input Parameters:     filep
 *                         Char device file
 *                       buffer
 *                         Buffer of data from the user
 *                       len
 *                         Length to write
 *                       offset
 *                         Offset to start the write (unused here)
 *
 * Output Parameters:    N/A
 *
 * Return:               Length wrote (in bytes)
 *
 * Comment: None
 *
 ******************************************************************************/
static ssize_t eviewitf_mfis_blend_write(struct file *filep, const char *buffer, size_t len, loff_t *offset)
{
	int blend_device_id;
	static int blend_buffer_id[NB_BLEND_FILE] = {0};
	uint32_t val_rw[EVIEWITF_MFIS_MSG_SIZE];
	int uncopyed_bytes = 0;
	size_t checked_len = 0;

	blend_device_id = iminor(filep->f_inode) - NB_CAM_FILE;
	if (len == 0) {
		return -EINVAL;
	}

	/* Check the device id */
	if ((blend_device_id  < 0) || (blend_device_id >= NB_BLEND_FILE)) {
		pr_err("Inconsistent device id for blending\n");
		return -EINVAL;
	}

	/* Check buffer id */
	if ((blend_buffer_id[blend_device_id] < 0) || (blend_buffer_id[blend_device_id] >= NB_BLENDING_BUFFER)) {
		pr_err("Blending buffer id invalid %d\n", blend_buffer_id[blend_device_id]);
		return -EACCES;
	}

	/* Check if the buffer exists */
	if ( blendings[blend_device_id].buffer_address[blend_buffer_id[blend_device_id]] == 0 ) {
		pr_err("Blending buffer does not exist\n");
		return -EACCES;
	}

	/* Do not write data outside buffer */
	if (len > (blendings[blend_device_id].buffer_size - *offset)) {
		checked_len = blendings[blend_device_id].buffer_size - *offset;
	} else  {
		checked_len = len;
	}

	uncopyed_bytes = copy_from_user(blendings[blend_device_id].buffer_address[blend_buffer_id[blend_device_id]], (void *)buffer, checked_len);

	/* Compute real uncopyed_bytes */
	uncopyed_bytes += len - checked_len;


	/* Send a MFIS message to indicate eView that a new frame as been written (with cam ID and buffer ID) */
	val_rw[0] = FCT_UPDATE_BLENDING;
	val_rw[1] = blend_device_id;
	val_rw[2] = blend_buffer_id[blend_device_id];
	eviewitf_mfis_send_msg_internal(val_rw);

	/* Set offset back to 0 as we flip the current buffer */
	*offset = 0;

	/* Update the buffer id */
	blend_buffer_id[blend_device_id]++;
	if (NB_BLENDING_BUFFER == blend_buffer_id[blend_device_id]) {
		blend_buffer_id[blend_device_id] = 0;
	}

	/* Return the number of bytes written */
	return (len - uncopyed_bytes);
}

/*******************************************************************************
 *
 * Function Name: eviewitf_mfis_cam_poll
 *
 * Purpose:
 *
 * Input Parameters:     N/A
 *
 * Output Parameters:    N/A
 *
 * Comment: None
*******************************************************************************/
static unsigned int eviewitf_mfis_cam_poll(struct file *filp, struct poll_table_struct *wait)
{
	int cam_read_id = iminor(filp->f_inode);
	unsigned int mask = 0;

	poll_wait(filp, &wait_queue_cam_it[cam_read_id], wait);

	/* check if data has been read */
	if (wait_queue_cam_flag[cam_read_id]) {
		mask |= POLLIN | POLLRDNORM;
		/* Force file position update */
		filp->f_pos = 0;
	} else {
		return 0;
	}

	return mask;
}

/*******************************************************************************
 * File operations structure
 ******************************************************************************/
static struct file_operations mfis_fops = {
	.owner          = THIS_MODULE,
	.open           = eviewitf_mfis_config_open,
	.release        = eviewitf_mfis_config_close,
	.unlocked_ioctl = eviewitf_mfis_config_ioctl
};

static const struct file_operations mfis_cam_fops = {
	.owner   = THIS_MODULE,
	.llseek  = default_llseek,
	.open    = eviewitf_mfis_cam_open,
	.release = eviewitf_mfis_cam_close,
	.read    = eviewitf_mfis_cam_read,
	.write   = eviewitf_mfis_cam_write,
	.poll    = eviewitf_mfis_cam_poll
};

static const struct file_operations mfis_blend_fops = {
	.owner   = THIS_MODULE,
	.llseek  = default_llseek,
	.open    = eviewitf_mfis_blend_open,
	.release = eviewitf_mfis_blend_close,
	.write   = eviewitf_mfis_blend_write,
};

static const struct of_device_id eviewitf_mfis_of_match[] = {
	{ .compatible = "eviewitf-mfis" },
	{ },
};
MODULE_DEVICE_TABLE(of, eviewitf_mfis_of_match);

/*******************************************************************************
 *
 * Function Name: eviewitf_mfis_probe
 *
 * Purpose: Driver probe
 *
 * Input Parameters:     N/A
 *
 * Output Parameters:    N/A
 *
 * Comment: None
 *
 ******************************************************************************/
static int eviewitf_mfis_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev_ret;
	int i;
	struct resource *res;
	char irqname[16];

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL)
		dev_err(&pdev->dev, "Failed to get platform resource\n");

	dev_dbg(&pdev->dev, "addr = 0x%llx, size = 0x%llx\n", res->start, resource_size(res));

	/* map MFIS register */
	mfis_base_addr = (u32 __iomem *)devm_ioremap_nocache(&pdev->dev,
							     res->start,
							     resource_size(res));
	if (!mfis_base_addr) {
		dev_err(&pdev->dev, "Failed to remap register.\n");
		ret = -ENOMEM;
		goto out;
	}

	ret = of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);
	if (ret)
		goto out;

	pm_runtime_enable(&pdev->dev);

	/* ----- Register IRQs ----- */

	for (i = 0; i < MFIS_IRQ_COUNT; i++) {
		mfis_irq_id[i] = platform_get_irq(pdev, i);
		snprintf(irqname, sizeof(irqname), "mfis_irq%d", i);
		ret = devm_request_irq(&pdev->dev, mfis_irq_id[i], mfis_handle_irq, IRQF_SHARED, irqname, THIS_MODULE->name);
		if (ret != 0) {
			dev_err(&pdev->dev, "cannot register %s %d\n", irqname, mfis_irq_id[i]);
			goto rpm_disable;
		}
	}

	/* ----- Register file device to system ---- */

	/* Allocate file number for all devices */
	ret = alloc_chrdev_region(&dev, 0, NB_DEVICE_FILE, "mfis_api");
	if (ret < 0) {
		dev_err(&pdev->dev, "cannot allocate major number\n");
		goto rpm_disable;
	}

	/* Create class for the devices */
	cl = class_create(THIS_MODULE, "mfis_class");
	if (IS_ERR(cl)) {
		ret = PTR_ERR(cl);
		goto unregister_char_device;
	}

	/* Init cdev for CAM devices */
	for (i = 0; i < NB_CAM_FILE; i++) {
		cdev_init(&c_dev[i], &mfis_cam_fops);
		ret = cdev_add(&c_dev[i], MKDEV(MAJOR(dev), MINOR(dev) + i), 1);
		if (ret < 0) {
			dev_err(&pdev->dev, "cannot add device to system\n");
			goto destroy_device_class;
		}
	}

	/* Init cdev for the blending device */
	for (i = 0; i < NB_BLEND_FILE; i++) {
		cdev_init(&c_dev[NB_CAM_FILE + i], &mfis_blend_fops);
		ret = cdev_add(&c_dev[NB_CAM_FILE + i], MKDEV(MAJOR(dev), MINOR(dev) + NB_CAM_FILE + i), 1);
		if (ret < 0) {
			dev_err(&pdev->dev, "cannot add device to system\n");
			goto destroy_device_class;
		}
	}

	/* Init cdev for IOCTL device */
	cdev_init(&c_dev[NB_CAM_FILE + NB_BLEND_FILE], &mfis_fops);
	ret = cdev_add(&c_dev[NB_CAM_FILE + NB_BLEND_FILE], MKDEV(MAJOR(dev), MINOR(dev) + NB_CAM_FILE + NB_BLEND_FILE), 1);
	if (ret < 0) {
		dev_err(&pdev->dev, "cannot add device to system\n");
		goto destroy_device_class;
	}

	/* Create camera devices */
	for (i = 0; i < NB_CAM_FILE; i++) {
		dev_ret = device_create(cl, NULL, MKDEV(MAJOR(dev), MINOR(dev) + i), NULL, "mfis_cam%d", i);
		if (IS_ERR(dev_ret)) {
			dev_err(&pdev->dev, "cannot create file device\n");
			ret = PTR_ERR(dev_ret);
			goto delete_cdev;
		}
	}

	/* Create the blending device O1 O2 O3*/
	for (i = 0; i < NB_BLEND_FILE; i++) {
		dev_ret = device_create(cl, NULL, MKDEV(MAJOR(dev), MINOR(dev) + NB_CAM_FILE + i), NULL, "mfis_O%d", i + 2);
		if (IS_ERR(dev_ret)) {
			dev_err(&pdev->dev, "cannot create file device\n");
			ret = PTR_ERR(dev_ret);
			goto destroy_cam;
		}
	}
	/* Create IOCTL device */
	dev_ret = device_create(cl, NULL, MKDEV(MAJOR(dev), MINOR(dev) + NB_CAM_FILE + NB_BLEND_FILE), NULL, "mfis_ioctl");
	if (IS_ERR(dev_ret)) {
		dev_err(&pdev->dev, "cannot create file device\n");
		ret = PTR_ERR(dev_ret);
		goto destroy_blending;
	}

	for (i = 0; i < NB_REAL_CAM_FILE; i++) {
		init_waitqueue_head(&wait_queue_cam_it[i]);
	}

	init_waitqueue_head(&wait_queue_config_it);

	/* Set boot mode, do not check return value, it may not be supported by R7 */
	mfis_set_boot_mode();

	/* Retrieve mapping from eView */
	ret = mfis_retrieve_mapping();

	dev_info(&pdev->dev, "eviewitf-mfis driver probed\n");

	return ret;

	/* Error handling */

destroy_blending:
	/* Destroy blending devices */
	for (i = 0; i < NB_BLEND_FILE; i++) {
		device_destroy(cl, MKDEV(MAJOR(dev), MINOR(dev) + NB_CAM_FILE + i));
	}

destroy_cam:
	/* Destroy cam devices */
	for (i = 0; i < NB_CAM_FILE; i++) {
		device_destroy(cl, MKDEV(MAJOR(dev), MINOR(dev) + i));
	}

delete_cdev:
	for (i = 0; i < NB_DEVICE_FILE; i++) {
		cdev_del(&c_dev[i]);
	}

destroy_device_class:
	class_destroy(cl);

unregister_char_device:
	unregister_chrdev_region(dev, 1);

rpm_disable:
	pm_runtime_disable(&pdev->dev);

out:
	return ret;
}

/*******************************************************************************
 *
 * Function Name: eviewitf_mfis_remove
 *
 * Purpose: Driver remove
 *
 * Input Parameters:     N/A
 *
 * Output Parameters:    N/A
 *
 * Comment: None
 *
 ******************************************************************************/
static int eviewitf_mfis_remove(struct platform_device *pdev)
{
	int i;

	/* Destroy cam devices */
	for (i = 0; i < NB_CAM_FILE; i++) {
		device_destroy(cl, MKDEV(MAJOR(dev), MINOR(dev) + i));
	}

	/* Destroy blending devices */
	for (i = 0; i < NB_BLEND_FILE; i++) {
		device_destroy(cl, MKDEV(MAJOR(dev), MINOR(dev) + NB_CAM_FILE + i));
	}

	/* Destroy IOCTL device */
	device_destroy(cl, MKDEV(MAJOR(dev), MINOR(dev) + NB_CAM_FILE + NB_BLEND_FILE));

	for (i = 0; i < NB_DEVICE_FILE; i++) {
		cdev_del(&c_dev[i]);
	}

	class_destroy(cl);

	unregister_chrdev_region(dev, 1);

	pm_runtime_disable(&pdev->dev);

	dev_info(&pdev->dev, "eviewitf-mfis driver removed\n");

	return 0;
}

/*******************************************************************************
 * Platform Driver structure
 ******************************************************************************/
static struct platform_driver eviewitf_mfis_driver = {
	.probe	= eviewitf_mfis_probe,
	.remove	= eviewitf_mfis_remove,
	.driver	= {
		.name		= "eviewitf-mfis",
		.of_match_table	= eviewitf_mfis_of_match,
	},
};

/*******************************************************************************
 * Initialization and De-initialization functions
 ******************************************************************************/
static int __init eviewitf_mfis_init(void)
{
	return platform_driver_register(&eviewitf_mfis_driver);
}
core_initcall(eviewitf_mfis_init);

static void __exit eviewitf_mfis_exit(void)
{
	platform_driver_unregister(&eviewitf_mfis_driver);
}
module_exit(eviewitf_mfis_exit);

/*******************************************************************************
 * Module properties
 ******************************************************************************/
MODULE_LICENSE("GPL v2");

