/* *
 * Copyright (C) 2006 Texas Instruments Inc
 *
 * This program is free software you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option)any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not,write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */
/* davinci_resizer.c file */

/*Header files*/
#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>		/*     kmalloc() */
#include <linux/fs.h>		/*     everything... */
#include <linux/errno.h>	/*     error codes     */
#include <linux/types.h>	/*     size_t */
#include <linux/cdev.h>
#include <linux/fb.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/err.h>
#include <linux/devfs_fs_kernel.h>	/* for devfs */
#include <linux/device.h>
#include <asm/arch/davinci_resizer_hw.h>
#include <asm/arch/davinci_resizer.h>

#define	DRIVERNAME	"DaVinciResizer"

MODULE_LICENSE("GPL");

/*Global structute shared between all applications*/
/*struct device_params device_config;*/
device_params_t device_config;
/* For registeration of	charatcer device*/
static struct cdev c_dev;
/* device structure	to make	entry in device*/
static dev_t dev;
/* for holding device entry*/
struct device *rsz_device = NULL;

/* inline function to free reserver pages  */
void inline rsz_free_pages(unsigned long addr, unsigned long bufsize)
{
	unsigned long size;
	unsigned long tempaddr;
	tempaddr = addr;
	if (!addr)
		return;
	size = PAGE_SIZE << (get_order(bufsize));
	while (size > 0) {
		ClearPageReserved(virt_to_page(addr));
		addr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}
	free_pages(tempaddr, get_order(bufsize));
}

/*
=====================malloc_buff===========================
Function to	allocate memory	to input and output	buffers
*/
int malloc_buff(rsz_reqbufs_t * reqbuff, channel_config_t * rsz_conf_chan)
{
	/* For looping purpose */
	int buf_index = ZERO;
	/* For calculating the difference between new buffers to be allocated */
	int diff;
	/* for pointing to input or output buffer pointer */
	int *buf_ptr, *buf_start;
	/* To calculate no of max buffers; */
	int maxbuffers;
	/* to calculate number of buffers allocated */
	unsigned int numbuffers = ZERO;
	/* for storing buffer size */
	int *buf_size;
	/* to free the number of allocared buffers */
	int free_index;
	/* to make sure buffer pointer never swapped */
	unsigned long adr;
	unsigned long size;

	dev_dbg(rsz_device, __FUNCTION__ "E\n");

	/* assigning the buf_ptr to input buffer which is array of void
	   pointer */
	if (reqbuff->buf_type == RSZ_BUF_IN) {
		dev_dbg(rsz_device, "Input buffer requested \n");
		buf_ptr = (int *)rsz_conf_chan->input_buffer;
		buf_size = &rsz_conf_chan->in_bufsize;
		maxbuffers = MAX_INPUT_BUFFERS;
	}

	/* assigning the buf_ptr to output buffer which is array of
	   void pointer */
	else if (reqbuff->buf_type == RSZ_BUF_OUT) {
		dev_dbg(rsz_device, "Output buffer requested \n");
		buf_ptr = (int *)rsz_conf_chan->output_buffer;
		buf_size = &rsz_conf_chan->out_bufsize;
		maxbuffers = MAX_OUTPUT_BUFFERS;
	} else {
		dev_dbg(rsz_device, "Invalid type \n");
		return -EINVAL;
	}

	/* Check the request for number of buffers */
	if (reqbuff->count > maxbuffers)
		return -EINVAL;

	/* Counting the number of  buffers allocated */
	dev_dbg(rsz_device, "The requested size of buffer is %d \n",
		reqbuff->size);
	buf_start = buf_ptr;
	while (*(buf_ptr) != (int)NULL && numbuffers < maxbuffers) {
		numbuffers++;
		buf_ptr++;
	}

	buf_ptr = buf_start;

	/* Free all     the     buffers if the count is zero */
	if (reqbuff->count == FREE_BUFFER) {
		/* Free all     the     buffers */
		for (buf_index = ZERO; buf_index < numbuffers; buf_index++) {
			/* free memory allocate for the image */
			dev_dbg(rsz_device,
				"Free all the allocated buffers \n");
			/* Free buffers using free_pages */
			rsz_free_pages(*buf_ptr, *buf_size);

			/* assign buffer zero to indicate its free */
			*buf_ptr = (int)NULL;
			buf_ptr++;
		}
		return SUCESS;
	}

	/* If buffers are previously allocated, size has to be same */
	if (numbuffers) {

		if (reqbuff->size != *buf_size) {
			for (buf_index = ZERO; buf_index < numbuffers;
			     buf_index++) {
				/* free memory allocate for the image */
				/* Free buffers using free_pages */
				rsz_free_pages(*buf_ptr, *buf_size);

				/* assign buffer zero to indicate its free */
				*buf_ptr = (int)NULL;
				buf_ptr++;
			}
			numbuffers = ZERO;
			buf_ptr = buf_start;
		}

	}

	/* get the difference to know how mnay buffers to allocate */
	dev_dbg(rsz_device, "The no of requested buffers are %d \n ",
		reqbuff->count);
	diff = numbuffers - reqbuff->count;
	if (diff > ZERO) {
		buf_ptr = buf_ptr + reqbuff->count;
		for (buf_index = reqbuff->count; buf_index < numbuffers;
		     buf_index++) {
			/* if difference is positive than deallocate that
			   much memory of input buff */

			/* free buffer using free_pages */
			rsz_free_pages(*buf_ptr, *buf_size);

			/* assign buffer zero to indicate its free */
			*buf_ptr = (int)NULL;
			buf_ptr++;
		}
	} else {
		/* make the difference positive */
		diff = reqbuff->count - numbuffers;
		buf_ptr = buf_ptr + numbuffers;
		for (buf_index = numbuffers; buf_index < reqbuff->count;
		     buf_index++) {

			/* assign memory to buffer */
			*buf_ptr =
			    (int)(__get_free_pages
				  (GFP_KERNEL | GFP_DMA,
				   get_order(reqbuff->size)));

			if (!(*buf_ptr)) {

				buf_ptr = (buf_ptr - (buf_index - numbuffers));

				for (free_index = numbuffers;
				     free_index < buf_index; free_index++) {

					rsz_free_pages(*buf_ptr, *buf_size);
					buf_ptr++;

				}
				dev_dbg(rsz_device,
					"requestbuffer:not enough memory");
				return -ENOMEM;
			}

			adr = *buf_ptr;
			size = PAGE_SIZE << (get_order(reqbuff->size));
			while (size > 0) {
				/* make sure the frame buffers
				   are never swapped out of     memory */
				SetPageReserved(virt_to_page(adr));
				adr += PAGE_SIZE;
				size -= PAGE_SIZE;
			}

			buf_ptr++;
		}
	}

	/* set the buffer size to requested size */
	/* this will be useful only when numbuffers = 0 */
	*buf_size = reqbuff->size;

	dev_dbg(rsz_device, __FUNCTION__ "L\n");

	return SUCESS;
}				/*     end     of function     Main_buff */

/*====================get_buf_address===========================*/
/* Function to query the  physical address of the buffer  requested by index*/

int get_buf_address(rsz_buffer_t * buffer, channel_config_t * rsz_conf_chan)
{
	int buffer_index = 0;

	dev_dbg(rsz_device, __FUNCTION__ "E\n");

	if (buffer == NULL)
		return -EINVAL;

	if (buffer->buf_type == RSZ_BUF_IN) {

		/* Check the request for number of input buffers */
		if (buffer->index > MAX_INPUT_BUFFERS)
			return -EINVAL;
		/*count number of input buffer allocated */
		while ((rsz_conf_chan->input_buffer[buffer_index] != NULL)
		       && (buffer_index < MAX_INPUT_BUFFERS)) {
			buffer_index++;
		}
		/*checking the index requested */
		if (buffer->index >= buffer_index) {
			dev_dbg(rsz_device,
				"Requested buffer not allocated \n");
			return -EINVAL;
		}

		/* assignning the  input address to offset which will be
		   used in mmap */
		buffer->offset =
		    ((int)(rsz_conf_chan->input_buffer[buffer->index]));
		dev_dbg(rsz_device, "The query input offset is %x",
			buffer->offset);

	}

	else if (buffer->buf_type == RSZ_BUF_OUT) {

		/* Check the request for number of output buffers */
		if (buffer->index > MAX_OUTPUT_BUFFERS)
			return -EINVAL;

		/* counting     number of output buffers */
		while ((rsz_conf_chan->output_buffer[buffer_index] != NULL)
		       && (buffer_index < MAX_OUTPUT_BUFFERS)) {
			buffer_index++;
		}
		/* checking the index requested */
		if (buffer->index >= buffer_index) {
			dev_dbg(rsz_device,
				"Requested buffer not allocated \n");
			return -EINVAL;
		}

		/* assignning the output address to offset which will be
		   used in mmap */
		buffer->offset =
		    ((int)(rsz_conf_chan->output_buffer[buffer->index]));

		dev_dbg(rsz_device, "The query output offset is %x",
			buffer->offset);

	} else {
		dev_dbg(rsz_device, "	Invalid	input type \n");
		return -EINVAL;
	}

	/* look up physical     address of the buffer */
	buffer->offset = virt_to_phys((void *)buffer->offset);
	dev_dbg(rsz_device, "the physical offset returned after query \
						is %x", buffer->offset);

	dev_dbg(rsz_device, __FUNCTION__ "L\n");

	return SUCESS;

}				/*End of function      getBufferAddress */

/*
=====================rsz_start===========================
 This function enable the resize bit after doing the hardware register
 configuration after which resizing	will be	carried	on.
*/
int rsz_start(rsz_resize_t * resize, channel_config_t * rsz_conf_chan)
{

	/* Holds the input address to the resizer */
	int in_address;
	/* Holds the output     address to resizer */
	int out_address;
	/* Conatains the input put and output buffer allocated size */
	int out_bufsize, in_bufsize;

	/* Conatins the pitch and vertical size of input and output image */
	int in_vsize, in_pitch, out_vsize, out_pitch;
	/* holds the return value; */
	int ret;
	/* For calculating the number of input buffers allocated */
	int buffer_in_index = ZERO;

	/* For calculating the number of output buffers allocated */
	int buffer_out_index = ZERO;

	dev_dbg(rsz_device, __FUNCTION__ "E\n");
	/* checking     the     configuartion status */
	if (rsz_conf_chan->config_state) {
		dev_dbg(rsz_device, "State not configured \n");
		return -EINVAL;
	}

	/* Taking the inpitch of the image */
	in_pitch =
	    rsz_conf_chan->register_config.rsz_sdr_inoff
	    & ~(RSZ_SDR_INOFF_OFFSET_MASK);
	/* Taking the out pitch of image */
	in_vsize =
	    ((rsz_conf_chan->register_config.rsz_in_size
	      & ~(RSZ_IN_SIZE_VERT_MASK)) >> RSZ_IN_SIZE_VERT_SHIFT);

	in_bufsize = in_vsize * in_pitch;

	/*getting the outpitch */
	out_pitch =
	    rsz_conf_chan->register_config.rsz_sdr_outoff
	    & ~(RSZ_SDR_OUTOFF_OFFSET_MASK);
	/* getting the vertical size  */
	out_vsize =
	    ((rsz_conf_chan->register_config.rsz_out_size
	      & ~(RSZ_OUT_SIZE_VERT_MASK)) >> RSZ_OUT_VSIZE_SHIFT);

	out_bufsize = out_vsize * out_pitch;

	if (resize->in_buf.index < ZERO) {
		/* assignning the address to the register configuration */
		if (resize->in_buf.size >= in_bufsize) {
			if (resize->in_buf.offset % 32)
				return -EINVAL;

			rsz_conf_chan->register_config.rsz_sdr_inadd =
			    resize->in_buf.offset;
		} else {
			dev_err(rsz_device, " invalid size \n");
			return -EINVAL;
		}
	} else {
		if (resize->in_buf.index > MAX_INPUT_BUFFERS)
			return -EINVAL;
		/*count number of input buffer allocated */
		while ((rsz_conf_chan->input_buffer[buffer_in_index] !=
			NULL) && (buffer_in_index < MAX_INPUT_BUFFERS)) {
			buffer_in_index++;
		}
		/*checking the index requested */
		if (resize->in_buf.index >= buffer_in_index) {
			dev_dbg(rsz_device,
				"Requested buffer not allocated \n");
			return -EINVAL;
		}

		in_address = virt_to_phys(((void *)
					   rsz_conf_chan->
					   input_buffer[resize->in_buf.index]));

		rsz_conf_chan->register_config.rsz_sdr_inadd = in_address;
	}

	if (resize->out_buf.index < ZERO) {
		if (resize->out_buf.size >= out_bufsize) {
			if (resize->out_buf.offset % 32)
				return -EINVAL;

			rsz_conf_chan->register_config.rsz_sdr_outadd =
			    resize->out_buf.offset;
		} else {
			dev_err(rsz_device, "Invalid	output size \n");
			return -EINVAL;
		}
	} else {
		if (resize->out_buf.index > MAX_OUTPUT_BUFFERS)
			return -EINVAL;
		/*count number of input buffer allocated */
		while ((rsz_conf_chan->output_buffer[buffer_out_index] !=
			NULL) && (buffer_out_index < MAX_OUTPUT_BUFFERS)) {
			buffer_out_index++;
		}
		/*checking the index requested */
		if (resize->out_buf.index >= buffer_out_index) {
			dev_dbg(rsz_device,
				"Requested buffer not allocated \n");
			return -EINVAL;
		}
		out_address = virt_to_phys(((void *)
					    (rsz_conf_chan->
					     output_buffer[resize->out_buf.
							   index])));
		rsz_conf_chan->register_config.rsz_sdr_outadd = out_address;
	}

	/* Channel is busy */
	dev_dbg(rsz_device, "\nThe physical add in rsz start is %x \n",
		rsz_conf_chan->register_config.rsz_sdr_inadd);

	rsz_conf_chan->status = CHANNEL_BUSY;

	/* Function call to add the entry of application in array */
	ret = add_to_array(rsz_conf_chan);

	/*Function call to set up the hardware */
	rsz_hardware_setup(rsz_conf_chan);

	dev_dbg(rsz_device, "After Hardware Setup PCR = %x", regr(PCR));

	/* Initialize the interrupt ISR to ZER0 */
	device_config.sem_isr.done = ZERO;
	/*Function call to enable resizer hardware */
	ret = rsz_enable(rsz_conf_chan);

	dev_dbg(rsz_device, "After ENABLE PCR = %x", regr(PCR));

	/* Waiting for resizing to be complete */
	wait_for_completion_interruptible(&(device_config.sem_isr));

	rsz_conf_chan->status = CHANNEL_FREE;

	if (rsz_writebuffer_status() != 0) {
		dev_err(rsz_device, "Error: Resizer write buffer overflow: \n");
	}

	delete_from_array(rsz_conf_chan);

	dev_dbg(rsz_device, __FUNCTION__ "L\n");

	return ret;
}				/*End of function Start_Resize */

/*
=====================add_to_array===========================
 Function to add the current channel configuration into	array
according to priority.
*/
int add_to_array(channel_config_t * rsz_conf_chan)
{
	int array_index, device_index;

	dev_dbg(rsz_device, __FUNCTION__ "E\n");

	/* locking the configuartion aaray */
	down_interruptible(&device_config.array_sem);

	/* Add configuration to the     queue according to its priority */
	if (device_config.array_count == EMPTY) {
		/* If array     empty insert at top     position */
		dev_dbg(rsz_device, "First request for resizing \n");
		device_config.channel_configuration[device_config.array_count]
		    = rsz_conf_chan;
	} else {
		/* Check the priority and insert according to the priority */
		/* it will start from first     index */
		for (array_index = SECONDENTRY;
		     array_index < device_config.array_count; array_index++) {
			if (device_config.
			    channel_configuration[array_index]->priority <
			    rsz_conf_chan->priority)
				break;
		}
		/* Shift all the elements one step down in array */
		/* IF firstelement and second have same prioroty than insert */
		/* below first */
		for (device_index = device_config.array_count;
		     device_index > array_index; device_index--) {
			device_config.channel_configuration[device_index] =
			    device_config.
			    channel_configuration[device_index - NEXT];
		}

		device_config.channel_configuration[array_index] =
		    rsz_conf_chan;
	}

	/* incrementing number of requests for resizing */
	device_config.array_count++;
	dev_dbg(rsz_device, "The total request for resizing are %d",
		device_config.array_count);

	if (device_config.array_count != SECONDENTRY) {
		up(&device_config.array_sem);

		/* if the request is pending that lock the request */
#ifdef CONFIG_PREEMPT_RT
		wait_for_completion_interruptible
		    (&(rsz_conf_chan->channel_sem));
#else
		down_interruptible(&(rsz_conf_chan->channel_sem));
#endif

	} else {
		up(&device_config.array_sem);
	}

	dev_dbg(rsz_device, __FUNCTION__ "L\n");

	return SUCESS;
}				/*  end of function addToarray */

/*
=====================delete_from_array===========================
 Function	to delete the processed	array entry	form the array
*/
int delete_from_array(channel_config_t * rsz_conf_chan)
{
	int array_index = FIRSTENTRY, device_index;

	dev_dbg(rsz_device, __FUNCTION__ "E\n");

	down_interruptible(&(device_config.array_sem));

	/*shift the     entried in array */
	if (device_config.array_count != SECONDENTRY) {
		/* decrementing the     request count */
		device_config.array_count--;

		/* Shift all the elements one step up in array */
		for (device_index = array_index;
		     device_index < device_config.array_count; device_index++) {

			device_config.channel_configuration[device_index] =
			    device_config.
			    channel_configuration[device_index + NEXT];
		}
		/* making last entry NULL; */
		device_config.channel_configuration[device_index + NEXT] = NULL;
	}
	/* remove the top entry */
	else {
		dev_dbg(rsz_device, "\n Removing the first request");
		device_config.array_count--;
		device_config.channel_configuration[FIRSTENTRY] = NULL;
	}

	if (device_config.array_count != FIRSTENTRY) {
		/* Get config having highest priority in array
		   resizer_device.config
		   and unlock config.sem of that config */

		dev_dbg(rsz_device,
			"Releasing array lock of the	second entry\n");
#ifdef CONFIG_PREEMPT_RT
		complete(&(device_config.channel_configuration
			   [FIRSTENTRY]->channel_sem));
#else
		up(&(device_config.channel_configuration
		     [FIRSTENTRY]->channel_sem));
#endif
		up(&(device_config.array_sem));
	} else {
		dev_dbg(rsz_device, "Releasing array lock	\n");
		up(&(device_config.array_sem));
	}

	dev_dbg(rsz_device, __FUNCTION__ "L\n");

	return SUCESS;
}				/*     end     of function     deleteFromarray */

int rsz_set_params(rsz_params_t * params, channel_config_t * rsz_conf_chan)
{
	int coeffcounter;
	int hrsz = ZERO;
	int vrsz = ZERO;
	int alignment = ZERO;
	int hsize;
	int vsize;

	dev_dbg(rsz_device, __FUNCTION__ "E\n");

	/* calculating the horizontal and vertical ratio */
	vrsz = (params->in_vsize - NUM_D2TAPS) * RATIO_MULTIPLIER /
	    (params->out_vsize - 1);
	hrsz = ((params->in_hsize - NUM_D2TAPS) * RATIO_MULTIPLIER) /
	    (params->out_hsize - 1);

	/* recalculating Horizontal     ratio */
	if (hrsz <= DOWN_RSZ_RATIO) {	/* 4-tap     8-phase filter */
		hrsz = (params->in_hsize - NUM_TAPS) * RATIO_MULTIPLIER
		    / (params->out_hsize - 1);
		if (hrsz > DOWN_RSZ_RATIO)
			hrsz = DOWN_RSZ_RATIO;
		if (params->hstph > NUM_PHASES)
			return -EINVAL;
	} else if (hrsz >= UP_RSZ_RATIO1 && hrsz <= DOWN_RSZ_RATIO1) {
		/* 7-tap        4-phase filter */
		if (params->hstph > NUM_D2PH)
			return -EINVAL;
	}

	/* recalculating vertical ratio */
	if (vrsz <= DOWN_RSZ_RATIO) {	/* 4-tap     8-phase filter */
		vrsz = (params->in_vsize - NUM_TAPS) * RATIO_MULTIPLIER /
		    (params->out_vsize - 1);
		if (vrsz > DOWN_RSZ_RATIO)
			vrsz = DOWN_RSZ_RATIO;
		if (params->vstph > NUM_PHASES)
			return -EINVAL;
	} else if (vrsz >= UP_RSZ_RATIO1 && vrsz <= DOWN_RSZ_RATIO1) {
		if (params->vstph > NUM_D2PH)
			return -EINVAL;
	}

	/* Fiiling the input pitch in the structure */
	if ((params->in_pitch) % ALIGN32) {
		dev_err(rsz_device, "Inavlid input pitch	%d \n",
			params->in_pitch);
		return -EINVAL;
	}
	rsz_conf_chan->register_config.rsz_sdr_inoff =
	    ((params->in_pitch) & ~(RSZ_SDR_INOFF_OFFSET_MASK));

	/* If vertical upsizing then */
	if (vrsz < 256) {
		/* checking     for     both types of format */
		if (params->inptyp == RSZ_INTYPE_PLANAR_8BIT) {
			alignment = ALIGNMENT;
		} else if (params->inptyp == RSZ_INTYPE_YCBCR422_16BIT) {
			alignment = (ALIGNMENT / 2);
		} else {
			dev_err(rsz_device, "Invalid input type	\n");
		}
		/* errror checking for output size */
		if (!(((params->out_hsize % PIXEL_EVEN) == ZERO)
		      && (params->out_hsize % alignment) == ZERO)) {
			dev_err(rsz_device, "wrong hsize	\n");

			return -EINVAL;
		}
	}
	if (hrsz >= UP_RSZ_RATIO && hrsz <= DOWN_RSZ_RATIO) {
		if (params->out_hsize > MAX_IMAGE_WIDTH) {
			dev_err(rsz_device, "wrong width	\n");
			return -EINVAL;
		}

	} else if (hrsz >= UP_RSZ_RATIO1 && hrsz <= DOWN_RSZ_RATIO1) {
		if (params->out_hsize > MAX_IMAGE_WIDTH_HIGH) {
			dev_err(rsz_device, "wrong width	\n");
			return -EINVAL;
		}
	} else {
		dev_err(rsz_device,
			"horizontal scaling ratio invalid: %d, %d, %dn",
			hrsz, params->in_hsize, params->out_hsize);
		return -EINVAL;
	}
	if (vrsz < UP_RSZ_RATIO || vrsz > DOWN_RSZ_RATIO1) {
		dev_err(rsz_device, "vertical scaling ratio invalid:%d,%d,%d\n",
			vrsz, params->in_vsize, params->out_vsize);
		return -EINVAL;
	}
	rsz_conf_chan->register_config.rsz_out_size =
	    (params->out_hsize & ~(RSZ_OUT_SIZE_HORZ_MASK));

	rsz_conf_chan->register_config.rsz_out_size |=
	    ((params->out_vsize << RSZ_OUT_VSIZE_SHIFT) &
	     ~(RSZ_OUT_SIZE_VERT_MASK));

	dev_dbg(rsz_device, "The	outpitch in driver is %d",
		params->out_pitch);
	if ((params->out_pitch) % ALIGN32) {
		dev_err(rsz_device, "Inavlid	output pitch \n");
		return -EINVAL;
	}
	rsz_conf_chan->register_config.rsz_sdr_outoff =
	    params->out_pitch & ~(RSZ_SDR_OUTOFF_OFFSET_MASK);

	rsz_conf_chan->register_config.rsz_cnt = 0;
	/* clear the rsz_cnt register */

	/* Configuring the chrominance algorithm */
	if (params->cbilin) {
		rsz_conf_chan->register_config.rsz_cnt =
		    BITSET(rsz_conf_chan->register_config.rsz_cnt,
			   SET_BIT_CBLIN);
		dev_dbg(rsz_device, "Setting chrominance algorithm bit \n");
	}

	/* Configuring the input source */
	if (INPUT_RAM) {
		dev_dbg(rsz_device, "Setting Input source as Ram \n");
		rsz_conf_chan->register_config.rsz_cnt =
		    BITSET(rsz_conf_chan->register_config.rsz_cnt,
			   SET_BIT_INPUTRAM);
	}
	/* Configuring the input type */
	if (params->inptyp == RSZ_INTYPE_PLANAR_8BIT) {
		dev_dbg(rsz_device, "Setting pic format as 8 bit planar \n");
		rsz_conf_chan->register_config.rsz_cnt =
		    BITSET(rsz_conf_chan->register_config.rsz_cnt,
			   SET_BIT_INPTYP);
	} else {
		dev_dbg(rsz_device,
			"Setting pic format as 16 bit color seperated\n");
		rsz_conf_chan->register_config.rsz_cnt =
		    BITRESET(rsz_conf_chan->register_config.rsz_cnt,
			     SET_BIT_INPTYP);

		/* Configuring the chrominace position type */
		if (params->pix_fmt == RSZ_PIX_FMT_UYVY) {

			rsz_conf_chan->register_config.rsz_cnt =
			    BITSET(rsz_conf_chan->register_config.rsz_cnt,
				   SET_BIT_YCPOS);
		} else if (params->pix_fmt == RSZ_PIX_FMT_YUYV) {
			rsz_conf_chan->register_config.rsz_cnt =
			    BITRESET(rsz_conf_chan->register_config.rsz_cnt,
				     SET_BIT_YCPOS);
		}

	}

	/* checking the validity of the horizontal phase value */
	if (hrsz >= UP_RSZ_RATIO && hrsz <= DOWN_RSZ_RATIO) {
		if (params->hstph > NUM_PHASES)
			return -EINVAL;
	} else if (hrsz >= UP_RSZ_RATIO && hrsz <= DOWN_RSZ_RATIO) {
		if (params->hstph > NUM_D2PH)
			return -EINVAL;
	}

	rsz_conf_chan->register_config.rsz_cnt |=
	    ((params->hstph << RSZ_HSTP_SHIFT) & ~(RSZ_HSTPH_MASK));

	/* checking     the     validity of     the     vertical phase value */
	if (vrsz >= UP_RSZ_RATIO && hrsz <= DOWN_RSZ_RATIO) {
		if (params->vstph > NUM_PHASES)
			return -EINVAL;
	} else if (vrsz >= UP_RSZ_RATIO && vrsz <= DOWN_RSZ_RATIO) {
		if (params->vstph > NUM_D2PH)
			return -EINVAL;
	}

	rsz_conf_chan->register_config.rsz_cnt |=
	    ((params->vstph << RSZ_VSTPH_SHIFT) & ~(RSZ_VSTPH_MASK));

	/* if input is from ram that vertical pixel should be zero */
	if (INPUT_RAM) {
		params->vert_starting_pixel = ZERO;
	}

	/* Configuring the starting pixel in vertical direction */
	rsz_conf_chan->register_config.rsz_in_start =
	    (params->vert_starting_pixel << RSZ_IN_SIZE_VERT_SHIFT)
	    & ~(RSZ_IN_START_VERT_ST_MASK);

	/* if input is 8 bit that start pixel should be <= to than 31 */
	if (params->inptyp == RSZ_INTYPE_PLANAR_8BIT) {
		if (params->horz_starting_pixel > MAX_HORZ_PIXEL_8BIT)
			return -EINVAL;
	}
	/* if input     is 16 bit that start pixel should be <= than 15 */
	if (params->inptyp == RSZ_INTYPE_YCBCR422_16BIT) {
		if (params->horz_starting_pixel > MAX_HORZ_PIXEL_16BIT)
			return -EINVAL;
	}

	/* Configuring the      starting pixel in horizontal direction */
	rsz_conf_chan->register_config.rsz_in_start |=
	    params->horz_starting_pixel & ~(RSZ_IN_START_HORZ_ST_MASK);

	for (coeffcounter = ZERO; coeffcounter < MAX_COEF_COUNTER;
	     coeffcounter++) {
		/* Configuration of     horizontal coefficients */
		rsz_conf_chan->register_config.
		    rsz_coeff_horz[coeffcounter] =
		    (params->hfilt_coeffs[2 * coeffcounter]
		     & ~(RSZ_FILTER_COEFF0_MASK));

		/* Configuration of     horizontal coefficients */

		rsz_conf_chan->register_config.
		    rsz_coeff_horz[coeffcounter] |=
		    ((params->hfilt_coeffs[2 * coeffcounter + NEXT]
		      << RSZ_FILTER_COEFF_SHIFT) & ~(RSZ_FILTER_COEFF1_MASK));

		/* Configuration of     Vertical coefficients */
		rsz_conf_chan->register_config.
		    rsz_coeff_vert[coeffcounter] =
		    (params->
		     vfilt_coeffs[2 *
				  coeffcounter] & ~(RSZ_FILTER_COEFF0_MASK));

		/* Configuration of Vertical coefficients */

		rsz_conf_chan->register_config.
		    rsz_coeff_vert[coeffcounter] |=
		    ((params->
		      vfilt_coeffs[2 * coeffcounter +
				   NEXT] << RSZ_FILTER_COEFF_SHIFT) &
		     ~(RSZ_FILTER_COEFF1_MASK));
	}
	/* Coefficinets of parameters for luma :- algo configuration */
	rsz_conf_chan->register_config.rsz_yehn =
	    ((params->yenh_params.type << RSZ_YENH_TYPE_SHIFT) &
	     ~(RSZ_YEHN_ALGO_MASK));

	/* Coefficinets of parameters for luma :- core configuration */
	if (params->yenh_params.type) {
		rsz_conf_chan->register_config.rsz_yehn |=
		    params->yenh_params.core & ~(RSZ_YEHN_CORE_MASK);

		/* Coefficinets of parameters for luma :- gain configuration */

		rsz_conf_chan->register_config.rsz_yehn |=
		    ((params->yenh_params.gain << RSZ_YENH_GAIN_SHIFT)
		     & ~(RSZ_YEHN_GAIN_MASK));

		/* Coefficinets of parameters for luma :- gain configuration */
		rsz_conf_chan->register_config.rsz_yehn |=
		    ((params->yenh_params.slop << RSZ_YENH_SLOP_SHIFT)
		     & ~(RSZ_YEHN_SLOP_MASK));
	}

	/* Configuring the horizonatl ratio */
	rsz_conf_chan->register_config.rsz_cnt |= ((hrsz - 1) & ~RSZ_HRSZ_MASK);

	/* Configuring the vertical     ratio */
	rsz_conf_chan->register_config.rsz_cnt |=
	    (((vrsz - 1) << RSZ_VRSZ_SHIFT) & ~RSZ_VRSZ_MASK);

	if (hrsz <= 512) {	/*4-tap filter */
		hsize =
		    ((32 * params->hstph + (params->out_hsize - 1) * hrsz +
		      16) >> 8) + 7;
	} else {
		hsize =
		    ((64 * params->hstph + (params->out_hsize - 1) * hrsz +
		      32) >> 8) + 7;
	}
	dev_dbg(rsz_device, "hsize = %d\n", hsize);
	if (vrsz <= 512) {	/*4-tap filter */
		vsize =
		    ((32 * params->vstph + (params->out_vsize - 1) * vrsz +
		      16) >> 8) + 4;
	} else {
		vsize =
		    ((64 * params->vstph + (params->out_vsize - 1) * vrsz +
		      32) >> 8) + 7;
	}
	dev_dbg(rsz_device, "vsize = %d\n", vsize);
	dev_dbg(rsz_device, "hrsz	= %d, vrsz = %d,\n", hrsz, vrsz);

	/* Configuring the Horizontal size of inputframn in MMR */
	rsz_conf_chan->register_config.rsz_in_size = hsize;
	/*params->in_hsize; */

	rsz_conf_chan->register_config.rsz_in_size |=
	    ((vsize /*params->in_vsize */  << RSZ_IN_SIZE_VERT_SHIFT)
	     & ~(RSZ_IN_SIZE_VERT_MASK));

	/*Setting the configuration status */
	dev_dbg(rsz_device, "Resizer State configured \n");
	rsz_conf_chan->config_state = STATE_CONFIGURED;

	dev_dbg(rsz_device, __FUNCTION__ "L\n");

	return SUCESS;
}				/*End of rsz_Set_Params */

/* End of rsz_Set_Params*/
/*
=====================rsz_Get_Params===========================
 Function to get the parameters	values
*/
int rsz_get_params(rsz_params_t * params, channel_config_t * rsz_conf_chan)
{
	int coeffcounter;

	dev_dbg(rsz_device, __FUNCTION__ "E\n");

	if (rsz_conf_chan->config_state) {
		dev_dbg(rsz_device, "	state not configured \n");
		return -EINVAL;
	}

	/* getting the horizontal size */
	params->in_hsize =
	    rsz_conf_chan->register_config.rsz_in_size &
	    ~(RSZ_IN_SIZE_HORZ_MASK);
	/* getting the vertical size */
	params->in_vsize =
	    ((rsz_conf_chan->register_config.rsz_in_size
	      & ~(RSZ_IN_SIZE_VERT_MASK)) >> RSZ_IN_SIZE_VERT_SHIFT);

	/* getting the input pitch */
	params->in_pitch =
	    rsz_conf_chan->register_config.rsz_sdr_inoff
	    & ~(RSZ_SDR_INOFF_OFFSET_MASK);

	/* getting the output horizontal size */
	params->out_hsize =
	    rsz_conf_chan->register_config.rsz_out_size
	    & ~(RSZ_OUT_SIZE_HORZ_MASK);

	/* getting the vertical size   */
	params->out_vsize =
	    ((rsz_conf_chan->register_config.rsz_out_size
	      & ~(RSZ_OUT_SIZE_VERT_MASK)) >> RSZ_OUT_VSIZE_SHIFT);

	/* getting the output pitch */
	params->out_pitch =
	    rsz_conf_chan->register_config.rsz_sdr_outoff
	    & ~(RSZ_SDR_OUTOFF_OFFSET_MASK);

	/* getting the chrominance algorithm  */
	params->cbilin =
	    ((rsz_conf_chan->register_config.rsz_cnt
	      & RSZ_CNT_CBILIN_MASK) >> SET_BIT_CBLIN);

	/* getting the input type */
	params->inptyp =
	    ((rsz_conf_chan->register_config.rsz_cnt
	      & RSZ_CNT_INPTYP_MASK) >> SET_BIT_INPTYP);
	/* getting the  starting pixel in horizontal direction */
	params->horz_starting_pixel =
	    ((rsz_conf_chan->register_config.rsz_in_start
	      & ~(RSZ_IN_START_HORZ_ST_MASK)));
	/* getting the  starting pixel in vertical direction */
	params->vert_starting_pixel =
	    ((rsz_conf_chan->register_config.rsz_in_start
	      & ~(RSZ_IN_START_VERT_ST_MASK)) >> RSZ_IN_SIZE_VERT_SHIFT);

	/* getting the horizontal starting phase */
	params->hstph =
	    ((rsz_conf_chan->register_config.rsz_cnt
	      & ~(RSZ_HSTPH_MASK) >> RSZ_HSTP_SHIFT));

	/* getting the vertical starting phase */
	params->vstph =
	    ((rsz_conf_chan->register_config.rsz_cnt
	      & ~(RSZ_VSTPH_MASK) >> RSZ_VSTPH_SHIFT));

	for (coeffcounter = ZERO; coeffcounter < MAX_COEF_COUNTER;
	     coeffcounter++) {
		/* getting the horizontal coefficients 0 */
		params->hfilt_coeffs[2 * coeffcounter] =
		    rsz_conf_chan->register_config.rsz_coeff_horz[coeffcounter]
		    & ~(RSZ_FILTER_COEFF0_MASK);

		/* getting the horizontal coefficients 1 */
		params->hfilt_coeffs[2 * coeffcounter + NEXT] =
		    ((rsz_conf_chan->register_config.
		      rsz_coeff_horz[coeffcounter]
		      & ~(RSZ_FILTER_COEFF1_MASK)) >> RSZ_FILTER_COEFF_SHIFT);

		/* getting the vertical coefficients 0 */
		params->vfilt_coeffs[2 * coeffcounter] =
		    rsz_conf_chan->register_config.rsz_coeff_vert[coeffcounter]
		    & ~(RSZ_FILTER_COEFF0_MASK);

		/* getting the vertical coefficients 1 */
		params->vfilt_coeffs[2 * coeffcounter + NEXT] =
		    ((rsz_conf_chan->register_config.
		      rsz_coeff_vert[coeffcounter]
		      & ~(RSZ_FILTER_COEFF1_MASK)) >> RSZ_FILTER_COEFF_SHIFT);

	}

	/* getting the parameters for luma :- algo */
	params->yenh_params.type =
	    ((rsz_conf_chan->register_config.rsz_yehn
	      & ~(RSZ_YEHN_ALGO_MASK)) >> RSZ_YENH_TYPE_SHIFT);

	/* getting the parameters for luma :- core      */
	params->yenh_params.core =
	    (rsz_conf_chan->register_config.rsz_yehn & ~(RSZ_YEHN_CORE_MASK));

	/* Coefficinets of parameters for luma :- gain  */
	params->yenh_params.gain =
	    ((rsz_conf_chan->register_config.rsz_yehn
	      & ~(RSZ_YEHN_GAIN_MASK)) >> RSZ_YENH_GAIN_SHIFT);

	/* Coefficinets of parameters for luma :- SLOP configuration */
	params->yenh_params.slop =
	    ((rsz_conf_chan->register_config.rsz_yehn
	      & ~(RSZ_YEHN_SLOP_MASK)) >> RSZ_YENH_SLOP_SHIFT);

	/* getting the input type */
	params->pix_fmt =
	    ((rsz_conf_chan->register_config.rsz_cnt
	      & RSZ_CNT_PIXFMT_MASK) >> SET_BIT_YCPOS);

	if (params->pix_fmt)
		params->pix_fmt = RSZ_PIX_FMT_UYVY;
	else
		params->pix_fmt = RSZ_PIX_FMT_YUYV;

	dev_dbg(rsz_device, __FUNCTION__ "L\n");
	return SUCESS;
}

void rsz_calculate_crop(channel_config_t * rsz_conf_chan,
			rsz_cropsize_t * cropsize)
{
	int luma_enable;
	dev_dbg(rsz_device, __FUNCTION__ "E\n");

	cropsize->hcrop = ZERO;
	cropsize->vcrop = ZERO;

	luma_enable =
	    ((rsz_conf_chan->register_config.rsz_yehn
	      & ~(RSZ_YEHN_ALGO_MASK)) >> RSZ_YENH_TYPE_SHIFT);

	/* Luma enhancement reduces image width 1 pixels from left,right */
	if (luma_enable) {
		cropsize->hcrop += 2;
	}

	dev_dbg(rsz_device, __FUNCTION__ "L\n");;
}

/*
=====================free_buff===========================
 this	function free the input	and output buffers alloated
*/
int free_buff(channel_config_t * rsz_conf_chan)
{
	int buffercounter = ZERO;

	dev_dbg(rsz_device, __FUNCTION__ "E\n");

	/* Free all     the     input buffers */
	while (rsz_conf_chan->input_buffer[buffercounter] != NULL
	       && buffercounter < MAX_INPUT_BUFFERS) {
		/* free the     memory */
		rsz_free_pages((unsigned long)rsz_conf_chan->input_buffer
			       [buffercounter], rsz_conf_chan->in_bufsize);
		/* assign buffer zero to indicate its free */
		rsz_conf_chan->input_buffer[buffercounter] = NULL;
		buffercounter++;
	}
	buffercounter = ZERO;
	/* free all the output buffers */
	while (rsz_conf_chan->output_buffer[buffercounter] != NULL
	       && buffercounter < MAX_INPUT_BUFFERS) {
		/* free the memory */
		rsz_free_pages((unsigned long)rsz_conf_chan->output_buffer
			       [buffercounter], rsz_conf_chan->out_bufsize);
		/*  assign buffer zero to indicate its  free */
		rsz_conf_chan->output_buffer[buffercounter] = NULL;
		buffercounter++;
	}

	dev_dbg(rsz_device, __FUNCTION__ "L\n");

	return SUCESS;
}

/*
=====================rsz_open===========================
This function creates a channels.
*/
static int rsz_open(struct inode *inode, struct file *filp)
{
	channel_config_t *rsz_conf_chan;
	int buffercounter;

	dev_dbg(rsz_device, __FUNCTION__ "E\n");

	if (filp->f_flags == O_NONBLOCK)
		return -1;
	/* if usage counter is greater than maximum supported channels
	   return error */
	if (device_config.module_usage_count >= MAX_CHANNELS) {
		dev_err(rsz_device,
			"\n modules usage count is greater than supported ");
		return -EBUSY;
	}

	/* allocate     memory for a new configuration */
	rsz_conf_chan = kmalloc(sizeof(channel_config_t), GFP_KERNEL);

	if (rsz_conf_chan == NULL) {
		dev_err(rsz_device,
			"\n cannot allocate memory ro channel config");
		return -ENOMEM;
	}

	dev_dbg(rsz_device,
		"Malloc Done for channel configuration structure\n");

	/* zeroing register     config */
	memset(&(rsz_conf_chan->register_config), ZERO,
	       sizeof(resizer_config_t));

	/* increment usage counter */
	/* Lock the     global variable and increment the counter */
	down_interruptible(&device_config.device_mutex);
	device_config.module_usage_count++;
	up(&device_config.device_mutex);

	/*STATE_NOT_CONFIGURED and priority to zero */
	rsz_conf_chan->config_state = STATE_NOT_CONFIGURED;

	/* Set priority to lowest for that configuration channel */
	rsz_conf_chan->priority = MIN_PRIORITY;

	rsz_conf_chan->status = CHANNEL_FREE;
	/*Set configuration     structure's    input_buffer and output_buffer */
	/*pointers to NULL */

	for (buffercounter = ZERO; buffercounter < MAX_INPUT_BUFFERS; buffercounter++) {	/* Help to initialize the input buffer to zero */
		rsz_conf_chan->input_buffer[buffercounter] = NULL;
	}

	for (buffercounter = ZERO; buffercounter < MAX_OUTPUT_BUFFERS;
	     buffercounter++) {
		/* Help to initialize the output buffer to zero */
		rsz_conf_chan->output_buffer[buffercounter] = NULL;

	}
	dev_dbg(rsz_device, "Initializing	of channel done	\n");

	/* Initializing of application mutex */
#ifdef CONFIG_PREEMPT_RT
	init_completion(&(rsz_conf_chan->channel_sem));
	rsz_conf_chan->channel_sem.done = 0;
#else
	init_MUTEX_LOCKED(&(rsz_conf_chan->channel_sem));
#endif
	init_MUTEX(&(rsz_conf_chan->chanprotection_sem));
	/* taking the configuartion     structure in private data */
	filp->private_data = rsz_conf_chan;

	dev_dbg(rsz_device, __FUNCTION__ "L\n");

	return SUCESS;

}				/*     End     of resizer open */

/*
=====================rsz_release===========================
 The Function	is used	to release the number of resources occupied
 by the channel
*/
static int rsz_release(struct inode *inode, struct file *filp)
{

	/* get the configuratin of this channel from private_date member of
	   file */
	int ret = 0;
	channel_config_t *rsz_conf_chan =
	    (channel_config_t *) filp->private_data;

	dev_dbg(rsz_device, __FUNCTION__ "E\n");

	ret = down_trylock(&(rsz_conf_chan->chanprotection_sem));
	if (ret != 0) {

		dev_dbg(rsz_device, "Channel in use", ret);
		return -EBUSY;
	}

	/* It will free all the input and output buffers */
	free_buff(rsz_conf_chan);

	/* Decrements the module usage count; */
	/* Lock the global variable and decrement variable */
	down_interruptible(&device_config.device_mutex);
	device_config.module_usage_count--;
	up(&device_config.device_mutex);

	kfree(rsz_conf_chan);

	dev_dbg(rsz_device, __FUNCTION__ "L\n");;

	up(&(rsz_conf_chan->chanprotection_sem));

	return SUCESS;
}				/*  End     of function     resizer_release */

/*
=====================rsz_mmap===========================
Function to map device memory into user	space
 */ static int rsz_mmap(struct file *filp, struct vm_area_struct *vma)
{

	/* get the configuratin of this channel from private_date
	   member of file */
	/* for looping purpuse */
	int buffercounter = ZERO;

	/* for checking purpose */
	int flag = ZERO;
	/* Hold number of input and output buffer allocated */
	int in_numbuffers = ZERO, out_numbuffers = ZERO;
	int buffer_offset;

	unsigned int offset = vma->vm_pgoff << PAGE_SHIFT;

	channel_config_t *rsz_conf_chan =
	    (channel_config_t *) filp->private_data;

	dev_dbg(rsz_device, __FUNCTION__ "E\n");

	/* Count the number of input buffers allocated */
	while ((rsz_conf_chan->input_buffer[buffercounter]) != NULL) {
		in_numbuffers++;
		buffercounter++;
	}
	buffercounter = ZERO;

	/* To Count the number of output buffers allocated */
	while ((rsz_conf_chan->output_buffer[buffercounter]) != NULL) {
		out_numbuffers++;
		buffercounter++;
	}

	/*Find the input address which  is to be mapped */
	for (buffercounter = ZERO; buffercounter < in_numbuffers;
	     buffercounter++) {
		buffer_offset =
		    virt_to_phys(rsz_conf_chan->input_buffer[buffercounter]);
		if (buffer_offset == offset) {
			flag = ADDRESS_FOUND;
			break;
		}
	}
	/*Find the output address which is to be mapped */
	if (flag == ZERO) {
		for (buffercounter = ZERO; buffercounter < out_numbuffers;
		     buffercounter++) {
			buffer_offset =
			    virt_to_phys(rsz_conf_chan->
					 output_buffer[buffercounter]);
			if (buffer_offset == offset) {
				flag = ADDRESS_FOUND;
				break;
			}
		}
	}
	/* The address to be mapped     is not found so return error */

	if (flag == ZERO)
		return -EAGAIN;

	dev_dbg(rsz_device, "The address mapped via mmap");
	/* map the address from user space to kernel space */
	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			    vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		return -EAGAIN;
	}

	dev_dbg(rsz_device, __FUNCTION__ "L\n");

	return SUCESS;

}				/*     End     of Function     resizer_mmap */

/*
=====================rsz_ioctl===========================
This function	will process IOCTL commands sent by
the application	and
control the device IO operations.
*/
static int rsz_ioctl(struct inode *inode, struct file *file,
		     unsigned int cmd, unsigned long arg)
{
	int ret = ZERO;
	/*get the configuratin of this channel from
	   private_date member of file */
	channel_config_t *rsz_conf_chan =
	    (channel_config_t *) file->private_data;

	/* Create the structures of
	   different parameters passed by user */
	rsz_priority_t *prio;
	rsz_status_t *status;
	rsz_resize_t *resize;

	dev_dbg(rsz_device, __FUNCTION__ "E\n");

	ret = down_trylock(&(rsz_conf_chan->chanprotection_sem));
	if (ret != 0) {

		dev_dbg(rsz_device, "Channel in use", ret);
		return -EBUSY;
	}

	/* Before decoding check for correctness of cmd */
	if (_IOC_TYPE(cmd) != RSZ_IOC_BASE) {
		dev_err(rsz_device, "Bad	command	Value \n");
		return -1;
	}
	if (_IOC_NR(cmd) > RSZ_IOC_MAXNR) {
		dev_err(rsz_device, "Bad	command	Value\n");
		return -1;
	}

	/*veryfying     access permission of commands */
	if (_IOC_DIR(cmd) & _IOC_READ)
		ret = !access_ok(VERIFY_WRITE, (void *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		ret = !access_ok(VERIFY_READ, (void *)arg, _IOC_SIZE(cmd));
	if (ret) {
		dev_err(rsz_device, "access denied\n");
		return -1;	/*error in access */
	}

	/* switch according     value of cmd */
	switch (cmd) {
		/*This ioctl is used to request frame buffers to be
		   allocated by the RSZ module. The allocated buffers
		   are channel  specific and can be     addressed
		   by indexing */
	case RSZ_REQBUF:

		/* Function to allocate the memory to input
		   or output buffer. */
		ret = malloc_buff((rsz_reqbufs_t *) arg, rsz_conf_chan);
		break;

		/*This ioctl is used to query the physical address of a
		   particular frame buffer. */
	case RSZ_QUERYBUF:

		/* Function     to query the  physical address of
		   the buffer  requested by index. */
		ret = get_buf_address((rsz_buffer_t *) arg, rsz_conf_chan);
		break;

		/* This ioctl is used to set the priority of the current
		   logical channel. If multiple resizing     tasks from multiple
		   logical channels are currently *pending, the task
		   associated with the  highest priority logical channel
		   will be executed first. */
	case RSZ_S_PRIORITY:

		dev_dbg(rsz_device, "\n resizer_Priority:start");
		prio = (rsz_priority_t *) arg;
		/* Check the prioroty range and assign the priority */
		if (prio->priority > MAX_PRIORITY ||
		    prio->priority < MIN_PRIORITY)
			return -EINVAL;
		else {
			rsz_conf_chan->priority = prio->priority;
		}
		dev_dbg(rsz_device, "\n resizer_Priority:end");
		break;
		/* This ioctl is used to get the priority of
		   the current logic channel */
	case RSZ_G_PRIORITY:

		dev_dbg(rsz_device, "\n Get resizer_Priority:start");
		prio = (rsz_priority_t *) arg;
		/* Get the priority     from the channel */
		prio->priority = rsz_conf_chan->priority;
		dev_dbg(rsz_device, "\n Get resizer_Priority:end");
		break;

		/* This ioctl is used to set the parameters
		   of the Resizer hardware, including input and output
		   image size, horizontal    and     vertical poly-phase
		   filter coefficients,luma enchancement filter coefficients etc */
	case RSZ_S_PARAM:

		/* Function to set the hardware configuration */
		ret = rsz_set_params((rsz_params_t *) arg, rsz_conf_chan);
		break;

		/*This ioctl is used to get the Resizer hardware settings
		   associated with the current logical channel represented
		   by fd. */
	case RSZ_G_PARAM:
		/* Function to get the hardware configuration */
		ret = rsz_get_params((rsz_params_t *) arg, rsz_conf_chan);
		break;

		/* This ioctl is used to check the current status
		   of the Resizer hardware */
	case RSZ_G_STATUS:
		status = (rsz_status_t *) arg;
		status->chan_busy = rsz_conf_chan->status;
		status->hw_busy = isbusy();
		status->src = INPUT_RAM;
		break;

		/*This ioctl submits a resizing task specified by the
		   rsz_resize structure.The call can either be blocked until
		   the task is completed or returned immediately based
		   on the value of the blocking argument in the rsz_resize
		   structure. If  it is blocking, the     status of the task
		   can be checked by calling ioctl   RSZ_G_STATUS. Only one task
		   can  be outstanding for each logical channel. */
	case RSZ_RESIZE:

		dev_dbg(rsz_device, "Beofre rsz_resize: PCR =%x", regr(PCR));
		resize = (rsz_resize_t *) arg;

		ret = rsz_start((rsz_resize_t *) arg, rsz_conf_chan);
		break;

	case RSZ_GET_CROPSIZE:

		rsz_calculate_crop(rsz_conf_chan, (rsz_cropsize_t *) arg);
		break;
	case RSZ_S_EXP:
		dev_dbg(rsz_device, "Before rsz_s_exp:SDR_REQ_EXP = %x",
			regr(SDR_REQ_EXP));
		rsz_set_exp(*((int *)arg));
		break;

	default:
		dev_dbg(rsz_device, "resizer_ioctl: Invalid Command Value");
		ret = -EINVAL;
	}

	dev_dbg(rsz_device, __FUNCTION__ "L\n");
	up(&(rsz_conf_chan->chanprotection_sem));

	return ret;
}				/*End of function IOCTL */

static struct file_operations rsz_fops = {
	.owner = THIS_MODULE,.open = rsz_open,.release =
	    rsz_release,.mmap = rsz_mmap,.ioctl = rsz_ioctl,
};

/*
=====================rsz_isr===========================
Function to register the Resizer character device	driver
*/
irqreturn_t rsz_isr(int irq, void *dev_id, struct pt_regs *regs)
{

	dev_dbg(rsz_device, __FUNCTION__ "E\n");

	/* to suggest that resizing     has     been completed */
	complete(&(device_config.sem_isr));

	dev_dbg(rsz_device, __FUNCTION__ "L\n");

	return IRQ_HANDLED;
}
static void resizer_platform_release(struct device *device)
{
	/* This is called when the reference count goes to zero */
}
static int __init resizer_probe(struct device *device)
{
	rsz_device = device;
	return 0;
}
static int resizer_remove(struct device *device)
{
	return 0;
}
static struct class_simple *rsz_class = NULL;

static struct platform_device resizer_device = {
	.name = "davinci_resizer",.id = 2,.dev = {
						  .release =
						  resizer_platform_release,}
};

static struct device_driver resizer_driver = {
	.name = "davinci_resizer",
	.bus = &platform_bus_type,
	.probe = resizer_probe,
	.remove = resizer_remove,
};

/*
=====================rsz_init===========================
function to	register resizer character driver
*/
static int __init rsz_init(void)
{

	int result;

	device_config.module_usage_count = ZERO;
	device_config.array_count = ZERO;

	/* Register     the     driver in the kernel */

	result = alloc_chrdev_region(&dev, ZERO, 1, DRIVER_NAME);
	if (result < ZERO) {
		printk("\nDaVinciresizer: Module intialization failed.\
		could not register character device");
		return -ENODEV;
	}
	/* Initialize of character device */
	cdev_init(&c_dev, &rsz_fops);
	c_dev.owner = THIS_MODULE;
	c_dev.ops = &rsz_fops;

	/* addding character device */
	result = cdev_add(&c_dev, dev, 1);

	if (result) {
		printk("NOtICE \nDaVinciresizer:Error %d adding Davinciresizer\
				 ..error no:", result);
		unregister_chrdev_region(dev, 1);
		return result;
	}

	/* registeration of     character device */
	register_chrdev(MAJOR(dev), DRIVER_NAME, &rsz_fops);

	/* register driver as a platform driver */
	if (driver_register(&resizer_driver) != 0) {
		unregister_chrdev_region(dev, 1);
		cdev_del(&c_dev);
		return -EINVAL;
	}

	/* Register the drive as a platform device */
	if (platform_device_register(&resizer_device) != 0) {
		driver_unregister(&resizer_driver);
		unregister_chrdev_region(dev, 1);
		unregister_chrdev(MAJOR(dev), DRIVER_NAME);
		cdev_del(&c_dev);
		return -EINVAL;
	}

	rsz_class = class_simple_create(THIS_MODULE, "davinci_resizer");

	if (!rsz_class) {

		platform_device_unregister(&resizer_device);
		cdev_del(&c_dev);
		unregister_chrdev(MAJOR(dev), DRIVER_NAME);

		return -EIO;
	}

	/* make entry in the devfs */
	result = devfs_mk_cdev(dev, S_IFCHR | S_IRUGO | S_IWUSR,
			       "%s%d", "davinci_resizer", 0);

	if (result < ZERO) {
		printk("\nresizer_init:	error in devfs_register_chrdev");
		cdev_del(&c_dev);
		unregister_chrdev(MAJOR(dev), DRIVER_NAME);
		class_simple_destroy(rsz_class);
		return result;
	}

	/* register     simple device class     */
	class_simple_device_add(rsz_class, dev, NULL, "davinci_resizer");

	init_completion(&(device_config.sem_isr));

	device_config.sem_isr.done = ZERO;

	/* Initialize the device mutex */
	init_MUTEX(&device_config.array_sem);
	init_MUTEX(&device_config.device_mutex);

	/* Set up the Interrupt handler for     resizer interrupt */

	result = request_irq(IRQ_RSZINT, rsz_isr, SA_INTERRUPT,
			     "dm644xresizer", (void *)NULL);
	if (result < ZERO) {
		printk("Cannot initialize IRQ \n");
		platform_device_unregister(&resizer_device);
		unregister_chrdev(MAJOR(dev), DRIVER_NAME);
		return result;
	}
	rsz_set_exp(0xe);

	return SUCESS;
}				/* End   of function  resizer_init */

/*
=====================rsz_cleanup===========================
Function	is called by the kernel. It	unregister the device.
*/
void __exit rsz_cleanup(void)
{

	unregister_chrdev_region(dev, 1);

	/* remove simple class device */
	class_simple_device_remove(dev);

	/* remove prev device from devfs */
	devfs_remove("%s%d", "davinci_resizer", ZERO);

	/* destroy simple class */
	class_simple_destroy(rsz_class);

	/* Remove platform driver */
	driver_unregister(&resizer_driver);

	/* disable interrupt */
	free_irq(IRQ_RSZINT, (void *)NULL);

	/* remove platform device */
	platform_device_unregister(&resizer_device);

	cdev_del(&c_dev);

	/* unregistering the driver     from the kernel */
	unregister_chrdev(MAJOR(dev), DRIVER_NAME);

}				/* End  of function   resizer_cleanup */

module_init(rsz_init) module_exit(rsz_cleanup)
