/* 
 * Copyright (C) 2006 Texas Instruments Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option)any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

/* davinci_previewer.c file */

/* include Linux files */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>	/* printk() */
#include <linux/slab.h>		/* kmalloc() */
#include <linux/fs.h>		/* everything... */
#include <linux/errno.h>	/* error codes */
#include <linux/types.h>	/* size_t */
#include <linux/cdev.h>		/* Used for struct cdev */
#include <linux/dma-mapping.h>	/* For class_simple_create */
#include <linux/interrupt.h>	/* For IRQ_HANDLED and irqreturn_t */
#include <asm/uaccess.h>	/* for VERIFY_READ/VERIFY_WRITE/
				   copy_from_user */
#include <linux/devfs_fs_kernel.h>	/* for devfs */
#include <asm/semaphore.h>

#include <asm/arch/davinci_previewer_hw.h>
#include <asm/arch/davinci_previewer.h>
#include <linux/device.h>

#ifdef __KERNEL__

struct device *prev_dev;

/* inline function to free reserver pages  */
void inline prev_free_pages(unsigned long addr, unsigned long bufsize)
{
	unsigned long size, ad = addr;
	size = PAGE_SIZE << (get_order(bufsize));
	if (!addr)
		return;
	while (size > 0) {
		ClearPageReserved(virt_to_page(addr));
		addr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}
	free_pages(ad, get_order(bufsize));
}

/* prev_calculate_crop: This function is used to calculate frame size 
   reduction depending on the features enabled by the application. */
void prev_calculate_crop(struct prev_params *config, struct prev_cropsize *crop)
{
	dev_dbg(prev_dev, __FUNCTION__ "E\n");

	if (!config || !crop) {

		dev_err(prev_dev, "\nErron in argument");
		return;
	}

	/* initialize hcrop and vcrop to zero */
	crop->hcrop = crop->vcrop = 0;

	/* Horizontal medial filter reduces image width by 4 pixels 
	   2 right and 2 left */
	if (config->features & PREV_HORZ_MEDIAN_FILTER) {
		crop->hcrop += 4;
	}
	/* Noise filter reduces image height and width 2 pixels from 
	   top, left, right and bottom */
	if (config->features & PREV_NOISE_FILTER) {
		crop->hcrop += 4;
		crop->vcrop += 4;
	}
	/* CFA Interpolation reduces image height and width 2 pixels 
	   from top, left, right and bottom */
	if (config->features & PREV_CFA) {
		crop->hcrop += 4;
		crop->vcrop += 4;
	}
	/* Luma enhancement reduces image width 1 pixels from left, right */
	if (config->features & (PREV_LUMA_ENHANCE | PREV_CHROMA_SUPPRESS)) {
		crop->hcrop += 2;
	}
	dev_dbg(prev_dev, "prev_calculate_crop L\n");
}

/* getstatus: This function will return status of the hardware in 
              prev_status structure.*/
int get_status(struct prev_status *status)
{
	dev_dbg(prev_dev, "get_status E\n");
	if (!status) {
		dev_err(prev_dev, "get_status:invalid parameter\n");
		return -EINVAL;
	}
	status->hw_busy = isbusy();

	dev_dbg(prev_dev, "get_status L\n");
	return 0;
}

/* previewer_isr: It is interrupt handler for PRVINT interrupt. 
 It will be called when previewer completes processing of one 
 frame and writes data to the DDR. It unblocks the PREV_PREVIEWER 
 ioctl which is waiting for the processing to be completed */
irqreturn_t previewer_isr(int irq, void *device_id, struct pt_regs * regs)
{
	struct prev_device *prevdevice = (struct prev_device *)device_id;
	dev_dbg(prev_dev, __FUNCTION__ "E\n");

	/* indicate the completion ofr frame processing */
	if (prevdevice)
		complete(&(prevdevice->wfc));

	dev_dbg(prev_dev, __FUNCTION__ "L\n");
	return IRQ_HANDLED;
}

int request_buffer(struct prev_device *device, struct prev_reqbufs *reqbufs)
{
	struct prev_buffer *buffer = NULL;
	int count = 0;
	unsigned long adr;
	u32 size;

	dev_dbg(prev_dev, __FUNCTION__ "E\n");
	if (!reqbufs || !device) {
		dev_err(prev_dev, "request_buffer: error in argument\n");
		return -EINVAL;
	}

	/* if number of buffers requested is more then support return error */
	if (reqbufs->count > MAX_BUFFER) {
		dev_err(prev_dev, "request_buffer: invalid buffer count\n");
		return -EINVAL;
	}

	/* if buf_type is input then allocate buffers for input */
	if (reqbufs->buf_type == PREV_BUF_IN) {
		/*if buffer count is zero, free all the buffers */
		if (reqbufs->count == 0) {
			/* free all the buffers */
			for (count = 0; count < device->in_numbuffers; count++) {
				/* free memory allocate for the image */
				if (device->in_buff[count]) {
					adr =
					    (unsigned long)device->
					    in_buff[count]->offset;
					if (adr)
						prev_free_pages((unsigned long)
								phys_to_virt
								(adr),
								device->in_buff
								[count]->size);

					/* free the memory allocated 
					   to prev_buffer */
					kfree(device->in_buff[count]);

					device->in_buff[count] = NULL;
				}
			}
			device->in_numbuffers = 0;
			return 0;
		}

		/* free the extra buffers */
		if (device->in_numbuffers > reqbufs->count &&
		    reqbufs->size == device->in_buff[0]->size) {
			for (count = reqbufs->count;
			     count < device->in_numbuffers; count++) {
				/* free memory allocate for the image */
				if (device->in_buff[count]) {
					adr = device->in_buff[count]->offset;
					if (adr)
						prev_free_pages((unsigned long)
								phys_to_virt
								(adr),
								device->in_buff
								[count]->size);

					/* free the memory allocated 
					   to prev_buffer */
					kfree(device->in_buff[count]);

					device->in_buff[count] = NULL;
				}
			}
			device->in_numbuffers = reqbufs->count;
			return 0;
		}
		/* if size requested is different from already allocated, 
		   free memory of all already allocated buffers */
		if (device->in_numbuffers) {
			if (reqbufs->size != device->in_buff[0]->size) {
				for (count = 0;
				     count < device->in_numbuffers; count++) {
					if (device->in_buff[count]) {
						adr =
						    device->
						    in_buff[count]->offset;
						if (adr)
							prev_free_pages((unsigned long)
									phys_to_virt
									(adr),
									device->
									in_buff
									[count]->
									size);

						kfree(device->in_buff[count]);

						device->in_buff[count] = NULL;
					}
				}
				device->in_numbuffers = 0;
			}
		}

		/* allocate the buffer */
		for (count = device->in_numbuffers; count < reqbufs->count;
		     count++) {
			/* Allocate memory for struct prev_buffer */
			buffer =
			    kmalloc(sizeof(struct prev_buffer), GFP_KERNEL);

			/* if memory allocation fails then return error */
			if (!buffer) {
				/* free all the buffers */
				while (--count >= device->in_numbuffers) {
					adr = device->in_buff[count]->offset;
					if (adr)
						prev_free_pages((unsigned long)
								phys_to_virt
								(adr),
								device->in_buff
								[count]->size);
					kfree(device->in_buff[count]);
					device->in_buff[count] = NULL;
				}
				dev_err(prev_dev, "request_buffer:not \
					enough memory\n");
				return -ENOMEM;
			}

			/* assign buffer's address in configuration */
			device->in_buff[count] = buffer;

			/* set buffers index and buf_type,size parameters */
			buffer->index = count;
			buffer->buf_type = PREV_BUF_IN;
			buffer->size = reqbufs->size;
			/* allocate memory for buffer of size passed 
			   in reqbufs */
			buffer->offset =
			    (unsigned long)__get_free_pages(GFP_KERNEL |
							    GFP_DMA,
							    get_order
							    (reqbufs->size));

			/* if memory allocation fails, return error */
			if (!(buffer->offset)) {
				/* free all the buffer's space */
				kfree(buffer);
				device->in_buff[count] = NULL;
				while (--count >= device->in_numbuffers) {
					adr = device->in_buff[count]->offset;
					if (adr)
						prev_free_pages((unsigned long)
								phys_to_virt
								(adr),
								device->in_buff
								[count]->size);
					kfree(device->in_buff[count]);
					device->in_buff[count] = NULL;
				}
				dev_err(prev_dev, "request_buffer:not \
					enough memory\n");

				return -ENOMEM;
			}

			adr = (unsigned long)buffer->offset;
			size = PAGE_SIZE << (get_order(reqbufs->size));
			while (size > 0) {
				/* make sure the frame buffers 
				   are never swapped out of memory */
				SetPageReserved(virt_to_page(adr));
				adr += PAGE_SIZE;
				size -= PAGE_SIZE;
			}
			/* convert vertual address to physical */
			buffer->offset = (unsigned long)
			    virt_to_phys((void *)(buffer->offset));
		}
		device->in_numbuffers = reqbufs->count;
	}
	/* if buf_type is output then allocate buffers for output */
	else if (reqbufs->buf_type == PREV_BUF_OUT) {
		if (reqbufs->count == 0) {
			/* free all the buffers */
			for (count = 0; count < device->out_numbuffers; count++) {
				/* free memory allocate for the image */
				if (device->out_buff[count]) {
					adr = device->out_buff[count]->offset;
					if (adr)
						prev_free_pages((unsigned long)
								phys_to_virt
								(adr),
								device->out_buff
								[count]->size);

					/* free the memory allocated to 
					   prev_buffer */
					kfree(device->out_buff[count]);

					device->out_buff[count] = NULL;
				}
			}
			device->out_numbuffers = 0;

			return 0;
		}
		/* free the buffers */
		if (device->out_numbuffers > reqbufs->count &&
		    reqbufs->size == device->out_buff[0]->size) {
			for (count = reqbufs->count;
			     count < device->out_numbuffers; count++) {
				/* free memory allocate for the image */
				if (device->out_buff[count]) {
					adr = device->out_buff[count]->offset;
					if (adr)
						prev_free_pages((unsigned long)
								phys_to_virt
								(adr),
								device->out_buff
								[count]->size);

					/* free the memory allocated to 
					   prev_buffer */
					kfree(device->out_buff[count]);

					device->out_buff[count] = NULL;
				}
			}
			device->out_numbuffers = reqbufs->count;

			return 0;
		}
		/* if size requested is different from already allocated, 
		   free memory of all already allocated buffers */
		if (device->out_numbuffers) {
			if (reqbufs->size != device->out_buff[0]->size) {
				for (count = 0;
				     count < device->out_numbuffers; count++) {
					if (device->out_buff[count]) {
						adr =
						    device->
						    out_buff[count]->offset;

						if (adr)
							prev_free_pages((unsigned long)
									phys_to_virt
									(adr),
									device->
									out_buff
									[count]->
									size);

						kfree(device->out_buff[count]);

						device->out_buff[count] = NULL;
					}
				}
				device->out_numbuffers = 0;
			}
		}

		/* allocate the buffer */
		for (count = device->out_numbuffers;
		     count < reqbufs->count; count++) {
			/* Allocate memory for struct prev_buffer */
			buffer =
			    kmalloc(sizeof(struct prev_buffer), GFP_KERNEL);

			/* if memory allocation fails then return error */
			if (!buffer) {
				/* free all the buffers */
				while (--count >= device->out_numbuffers) {
					adr = device->out_buff[count]->offset;
					if (adr)
						prev_free_pages((unsigned long)
								phys_to_virt
								(adr),
								device->out_buff
								[count]->size);
					kfree(device->out_buff[count]);
					device->out_buff[count] = NULL;
				}

				dev_err(prev_dev, "request_buffer:not enough \
					memory\n");

				return -ENOMEM;
			}

			/* assign buffer's address out configuration */
			device->out_buff[count] = buffer;

			/* set buffers outdex and buf_type,size parameters */
			buffer->index = count;
			buffer->buf_type = PREV_BUF_OUT;
			buffer->size = reqbufs->size;
			/* allocate memory for buffer of size passed 
			   in reqbufs */
			buffer->offset =
			    (unsigned long)__get_free_pages(GFP_KERNEL |
							    GFP_DMA,
							    get_order
							    (reqbufs->size));

			/* if memory allocation fails, return error */
			if (!(buffer->offset)) {
				/* free all the buffer's space */
				kfree(buffer);
				device->out_buff[count] = NULL;
				while (--count >= device->out_numbuffers) {
					adr = device->out_buff[count]->offset;
					if (adr)
						prev_free_pages((unsigned long)
								phys_to_virt
								(adr),
								device->out_buff
								[count]->size);
					kfree(device->out_buff[count]);
					device->out_buff[count] = NULL;
				}
				dev_err(prev_dev, "request_buffer:not \
					enough memory\n");

				return -ENOMEM;
			}

			adr = (unsigned long)buffer->offset;
			size = PAGE_SIZE << (get_order(reqbufs->size));
			while (size > 0) {
				/* make sure the frame buffers 
				   are never swapped out of memory */
				SetPageReserved(virt_to_page(adr));
				adr += PAGE_SIZE;
				size -= PAGE_SIZE;
			}
			/* convert vertual address to physical */
			buffer->offset = (unsigned long)
			    virt_to_phys((void *)(buffer->offset));
		}
		device->out_numbuffers = reqbufs->count;
	} else {
		dev_err(prev_dev, "request_buffer: invalid buffer type\n");

		return -EINVAL;
	}

	dev_dbg(prev_dev, __FUNCTION__ "L\n");
	return 0;
}

/* querybuffer: This function will query the buffer’s physical address
     whose index is passed in prev_buffer. it will store that address 
	in prev_buffer. */
int query_buffer(struct prev_device *device, struct prev_buffer *buffer)
{
	dev_dbg(prev_dev, __FUNCTION__ "E\n");

	if (!buffer || !device) {
		dev_err(prev_dev, "query_buffer: error in argument\n");
		return -EINVAL;
	}

	/* if buf_type is input buffer then get offset of input buffer */
	if (buffer->buf_type == PREV_BUF_IN) {
		/* error checking for wrong index number */
		if (buffer->index >= device->in_numbuffers) {
			dev_err(prev_dev, "query_buffer: invalid index");

			return -EINVAL;
		}

		/* get the offset and size of the buffer and store 
		   it in buffer */
		buffer->offset = device->in_buff[buffer->index]->offset;
		buffer->size = device->in_buff[buffer->index]->size;
	}
	/* if buf_type is output buffer then get offset of output buffer */
	else if (buffer->buf_type == PREV_BUF_OUT) {
		/* error checking for wrong index number */
		if (buffer->index >= device->out_numbuffers) {
			dev_err(prev_dev, "query_buffer: invalid index\n");

			return -EINVAL;
		}
		/* get the offset and size of the buffer and store 
		   it in buffer */
		buffer->offset = device->out_buff[buffer->index]->offset;
		buffer->size = device->out_buff[buffer->index]->size;
	} else {
		dev_err(prev_dev, "query_buffer: invalid buffer type\n");

		return -EINVAL;
	}

	dev_dbg(prev_dev, __FUNCTION__ "L\n");
	return 0;
}

int validate_params(struct prev_params *params)
{
	struct prev_cropsize crop;

	dev_dbg(prev_dev, __FUNCTION__ "E\n");

	if (!params) {
		dev_err(prev_dev, "validate_params:error in argument");
		return -EINVAL;
	}

	prev_calculate_crop(params, &crop);

	/* check whether down sampling rate is one of the supported */
	if (params->sample_rate != DOWN_SAMPLE_RATE1
	    && params->sample_rate != DOWN_SAMPLE_RATE2
	    && params->sample_rate != DOWN_SAMPLE_RATE3
	    && params->sample_rate != DOWN_SAMPLE_RATE4)
		/* if not return error */
	{
		return -EINVAL;
	}

	/* check for valid values of pixel size */
	if (params->size_params.pixsize != PREV_INWIDTH_8BIT
	    && params->size_params.pixsize != PREV_INWIDTH_10BIT) {
		return -EINVAL;
	}

	/* check whether size of the image is within limit */
	if ((params->size_params.hsize) > 1280 + crop.hcrop
	    || (params->size_params.hsize) < 0) {
		return -EINVAL;
	}

	if ((params->size_params.vsize) > 1920 + crop.vcrop
	    || (params->size_params.vsize) < 0) {
		return -EINVAL;
	}

	/* check for valid values output pixel format */
	if (params->pix_fmt != PREV_PIXORDER_YCBYCR &&
	    PREV_PIXORDER_YCRYCB != params->pix_fmt &&
	    PREV_PIXORDER_CBYCRY != params->pix_fmt &&
	    PREV_PIXORDER_CRYCBY != params->pix_fmt) {
		return -EINVAL;
	}
	/* dark frame capture and subtract should not be enabled 
	   at the same time */
	if ((params->features & PREV_DARK_FRAME_SUBTRACT) &&
	    (params->features & PREV_DARK_FRAME_CAPTURE))
		return -EINVAL;
	/* check to see dark frame address should not be null */
	if (params->features & PREV_DARK_FRAME_SUBTRACT)
		if (!(params->dark_frame_addr)
		    || (params->dark_frame_pitch % 32)) {
			return -EINVAL;
		}

	/* check to see lens shading shift value should not be greater 
	   than 7 */
	if (params->features & PREV_LENS_SHADING)
		if (params->lens_shading_sift > 7 || !(params->dark_frame_addr)
		    || (params->dark_frame_pitch % 32)) {
			return -EINVAL;
		}

	/* if pitch is zero assign it to the width of the image */
	if (params->size_params.in_pitch <= 0
	    || params->size_params.in_pitch % 32) {
		dev_err(prev_dev, "\nvalidate_params:error in pitch");
		return -EINVAL;
	}

	if (params->size_params.out_pitch <= 0
	    || params->size_params.out_pitch % 32) {
		dev_err(prev_dev, "\nvalidate_params:error in pitch");
		return -EINVAL;
	}

	dev_dbg(prev_dev, __FUNCTION__ "L\n");
	return 0;
}

/* This function is used to free memory allocated to buffers */
int free_buffers(struct prev_device *device)
{
	int i;
	unsigned long adr;
	dev_dbg(prev_dev, __FUNCTION__ "E\n");
	if (!device) {
		dev_err(prev_dev, "\nfree_buffers:error in argument");
		return -EINVAL;
	}
	/* free memory allocated to in buffers */
	for (i = 0; i < device->in_numbuffers; i++) {
		if (device->in_buff[i]) {
			adr = device->in_buff[i]->offset;
			if (adr)
				prev_free_pages((unsigned long)
						phys_to_virt(adr),
						device->in_buff[i]->size);

			kfree(device->in_buff[i]);

			device->in_buff[i] = NULL;
		}
	}
	device->in_numbuffers = 0;
	/* free memory allocated to out buffers */
	for (i = 0; i < device->out_numbuffers; i++) {
		if (device->out_buff[i]) {
			adr = device->out_buff[i]->offset;
			if (adr)
				prev_free_pages((unsigned long)
						phys_to_virt(adr),
						device->out_buff[i]->size);

			kfree(device->out_buff[i]);

			device->out_buff[i] = NULL;
		}
	}

	device->out_numbuffers = 0;
	dev_dbg(prev_dev, __FUNCTION__ "L\n");
	return 0;
}

/* preview: This function is used to submit previewing task to the 
		Previewer hardware */
int preview(struct prev_device *device, struct prev_convert *convert)
{
	int bpp, size, cropsize;
	unsigned long in_addr, out_addr;
	struct prev_cropsize crop;
	dev_dbg(prev_dev, __FUNCTION__ "E\n");

	/* error checking */
	if (!convert || !device) {
		dev_err(prev_dev, "preview: invalid convert parameters\n");

		return -EINVAL;
	}

	/* Call prev_calculate_crop to calculate size reduction in 
	   input image */
	prev_calculate_crop(device->params, &crop);

	/* Calculate bytes per pixel */
	if (device->params->size_params.pixsize == PREV_INWIDTH_8BIT)
		bpp = 1;
	else
		bpp = 2;

	size = device->params->size_params.hsize *
	    device->params->size_params.vsize * bpp;

	cropsize =
	    2 * (crop.vcrop * device->params->size_params.hsize +
		 crop.hcrop * (device->params->size_params.vsize - crop.vcrop));

	/* configure input buffer's address */
	/* If index member of in_buff of arg is less than 0 then */
	if (convert->in_buff.index < 0) {
		/* If size member of in_buff of arg is less than the size 
		   specified in size_params member of prev_params */
		if (convert->in_buff.size < size)
			return -EINVAL;

		/* Check for 32 byte aligned address */
		if (convert->in_buff.offset % 32 || !convert->in_buff.offset)
			return -EINVAL;

		/* Set address in RSDR_ADDR */
		in_addr = convert->in_buff.offset;
	} else {
		/* Check for valid index */
		if (convert->in_buff.index > device->in_numbuffers) {
			dev_err(prev_dev, "\ninvalid index");
			return -EINVAL;
		}

		/* check for size validity */
		if (size > device->in_buff[convert->in_buff.index]->size) {
			dev_err(prev_dev, "\nsize incorrect size = %d", size);
			return -EINVAL;
		}

		in_addr =
		    (unsigned long)device->in_buff[convert->in_buff.
						   index]->offset;
	}

	if (convert->out_buff.index < 0) {
		/* If size member of in_buff of arg is less than the size 
		   specified in size_params member of prev_params */
		if (convert->out_buff.size < (2 * size / bpp - cropsize))
			return -EINVAL;

		/* Check for 32 byte aligned address */
		if (convert->out_buff.offset % 32 || !convert->out_buff.offset)
			return -EINVAL;

		/* Set address in WSDR_ADDR */
		out_addr = convert->out_buff.offset;
	} else {
		/* Check for valid index */
		if (convert->out_buff.index > device->out_numbuffers) {
			dev_err(prev_dev, "\ninvalid index");
			return -EINVAL;
		}

		/* check for size validity */
		if ((2 * size / bpp - cropsize) >
		    device->out_buff[convert->out_buff.index]->size) {
			dev_err(prev_dev, "\nsize incorrect size");
			return -EINVAL;
		}

		out_addr =
		    (unsigned long)device->out_buff[convert->out_buff.
						    index]->offset;
	}

	/* Set RADR_OFFSET to width of the image and
	   Set WADR_OFFSET to height of the image – 2 * hcrop
	 */
	set_rsdr_offset(device->params->size_params.in_pitch);
	set_wsdr_offset(device->params->size_params.out_pitch);

	/* Set register RSDR_ADDR from in_vertref and
	   Set register WSDR_ADDR from out_vertre */
	set_size(device->params->size_params.hstart,
		 device->params->size_params.vstart,
		 device->params->size_params.hsize,
		 device->params->size_params.vsize);

	set_address(in_addr, out_addr);

	/* Set input source to DDRAM */
	set_input_source(1);

	/* Set one shot mode */
	set_oneshot_mode();

	/* enable previewer which starts previewing */
	previewer_enable();

	/* wait untill processing is not completed */
	wait_for_completion_interruptible(&(device->wfc));

	dev_dbg(prev_dev, __FUNCTION__ "L\n");
	return 0;
}

#endif				/* End of ifdef __KERNEL__ */

#define DRIVERNAME  "DaVinciPreviewer"

/* global object of prev_device structure */
struct prev_device prevdevice = { 0 };

/* Functions */
int previewer_open(struct inode *inode, struct file *filp)
{
	struct prev_params *config = NULL;
	struct prev_device *device = &prevdevice;

	dev_dbg(prev_dev, __FUNCTION__ "E\n");

	if (device->opened || filp->f_flags & O_NONBLOCK) {
		dev_err
		    (prev_dev, "previewer_open: device is already openend\n");
		return -EBUSY;
	}

	/* allocate memory for a new configuration */
	if ((config = kmalloc(sizeof(struct prev_params), GFP_KERNEL)) == NULL) {
		return -ENOMEM;
	}

	/* store the pointer of prev_params in private_data member of file 
	   and params member of prev_device */
	filp->private_data = config;

	/* initialize mutex to 0 */
	prevdevice.params = config;
	prevdevice.opened = 1;
	prevdevice.in_numbuffers = 0;
	prevdevice.out_numbuffers = 0;
	init_completion(&(prevdevice.wfc));
	prevdevice.wfc.done = 0;
	init_MUTEX(&(prevdevice.sem));

	dev_dbg(prev_dev, __FUNCTION__ "L\n");
	return 0;
}

int previewer_release(struct inode *inode, struct file *filp)
{
	/* get the configuratin from private_date member of file */
	struct prev_params *config = filp->private_data;
	struct prev_device *device = &prevdevice;
	dev_dbg(prev_dev, __FUNCTION__ "E\n");

	/* call free_buffers to free memory allocated to buffers */
	free_buffers(device);

	/* free the memory allocated to configuration */
	if (config)
		kfree(config);

	/* Assign null to private_data member of file and params 
	   member of device */
	filp->private_data = device->params = NULL;

	/* change the device status to available */
	device->opened = 0;

	dev_dbg(prev_dev, __FUNCTION__ "L\n");
	return 0;
}

int previewer_mmap(struct file *filp, struct vm_area_struct *vma)
{
	/* get the address of global object of prev_device structure */
	struct prev_device *device = &prevdevice;
	int i, flag = 0;
	/* get the page offset */
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	dev_dbg(prev_dev, __FUNCTION__ "E\n");

	/* page offset passed in mmap should one from input buffers */
	for (i = 0; i < device->in_numbuffers; i++) {
		if (device->in_buff[i]->offset == offset) {
			flag = 1;
			break;
		}
	}

	/* page offset passed in mmap should one from output buffers */
	if (flag == 0) {
		for (i = 0; i < device->out_numbuffers; i++) {
			if (device->out_buff[i]->offset == offset) {
				flag = 1;
				break;
			}
		}
	}

	/* if it is not set offset is not available in input/output buffers */
	if (flag == 0)
		return -EAGAIN;

	/* map buffers address space from kernel space to user space */
	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			    vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		return -EAGAIN;
	}

	dev_dbg(prev_dev, __FUNCTION__ "L\n");
	return 0;
}
int previewer_ioctl(struct inode *inode, struct file *file,
		    unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct prev_params params;
	struct prev_convert conv;
	/* get the address of global object of prev_device structure */
	struct prev_device *device = &prevdevice;

	dev_dbg(prev_dev, __FUNCTION__ "E\n");

	/* Before decoding check for correctness of cmd */
	if (_IOC_TYPE(cmd) != PREV_IOC_BASE) {
		dev_err(prev_dev, "Bad command Value \n");
		return -1;
	}
	if (_IOC_NR(cmd) > PREV_IOC_MAXNR) {
		dev_err(prev_dev, "Bad command Value\n");
		return -1;
	}

	/* Verify accesses       */
	if (_IOC_DIR(cmd) & _IOC_READ)
		ret = !access_ok(VERIFY_WRITE, (void *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		ret = !access_ok(VERIFY_READ, (void *)arg, _IOC_SIZE(cmd));
	if (ret) {
		dev_err(prev_dev, "access denied\n");
		return -1;	/*error in access */
	}

	/* switch according value of cmd */
	switch (cmd) {
		/* if case is to query for buffer address */
	case PREV_QUERYBUF:
		/* call query buffer which will return buffer address */
		down_interruptible(&(device->sem));
		ret = query_buffer(device, (struct prev_buffer *)arg);
		up(&(device->sem));
		break;

		/* if case is to request buffers */
	case PREV_REQBUF:
		/* call request buffer to allocate buffers */
		down_interruptible(&(device->sem));
		ret = request_buffer(device, (struct prev_reqbufs *)arg);
		up(&(device->sem));
		break;

		/* if case is to set configuration parameters */
	case PREV_SET_PARAM:

		down_interruptible(&(device->sem));
		/* copy the parameters to the configuration */
		if (copy_from_user
		    (&params, (struct prev_params *)arg,
		     sizeof(struct prev_params)))
			/* if it fails return error */
		{
			up(&(device->sem));
			return -EFAULT;
		}
		/* check for errors */

		ret = validate_params(&params);
		if (ret < 0) {
			up(&(device->sem));
			return ret;
		}

		/* copy the values to devce params */
		if (device->params)
			memcpy(device->params, &params,
			       sizeof(struct prev_params));
		else {
			dev_err(prev_dev, "\nPreviewer_ioctl:error in \
					device->params");
			up(&(device->sem));
			return -EINVAL;
		}

		ret = previewer_hw_setup(device->params);
		up(&(device->sem));
		break;

		/* if case is to get configuration parameters */
	case PREV_GET_PARAM:

		/* copy the parameters from the configuration */
		if (copy_to_user
		    ((struct prev_params *)arg, (device->params),
		     sizeof(struct prev_params)))
			/* if copying fails return error */
			ret = -EFAULT;
		break;

		/* if the case is to get status */
	case PREV_GET_STATUS:
		/* call getstatus function to get the status in arg */
		ret = get_status((struct prev_status *)arg);
		break;

		/* if the case is to do previewing */
	case PREV_PREVIEW:
		/* call preview function to do preview */
		if (copy_from_user(&conv, (struct prev_convert *)arg,
				   sizeof(struct prev_convert)))
			return -EFAULT;

		down_interruptible(&(device->sem));
		ret = preview(device, &conv);
		up(&(device->sem));
		break;
	case PREV_GET_CROPSIZE:
		prev_calculate_crop(device->params,
				    (struct prev_cropsize *)arg);
		break;
	case PREV_SET_EXP:

		prev_set_exp(*((int *)arg));
		break;

	default:
		dev_err(prev_dev, "previewer_ioctl: Invalid Command Value\n");
		ret = -EINVAL;
	}

	dev_dbg(prev_dev, __FUNCTION__ "L\n");
	return ret;
}
static void previewer_platform_release(struct device *device)
{
	/* This is called when the reference count goes to zero */
}

static int __init previewer_probe(struct device *device)
{
	prev_dev = device;
	return 0;
}

static int previewer_remove(struct device *device)
{
	return 0;
}

/* global variable of type file_operations containing function 
pointers of file operations */
static struct file_operations prev_fops = {
	.owner = THIS_MODULE,
	.open = previewer_open,
	.release = previewer_release,
	.mmap = previewer_mmap,
	.ioctl = previewer_ioctl,
};

/* global variable of type cdev to register driver to the kernel */
static struct cdev cdev;

/* global variable which keeps major and minor number of the driver in it */
static dev_t dev;

static struct class_simple *prev_class = NULL;

static struct platform_device previewer_device = {
	.name = "davinci_previewer",
	.id = 2,
	.dev = {
		.release = previewer_platform_release,
		}
};

static struct device_driver previewer_driver = {
	.name = "davinci_previewer",
	.bus = &platform_bus_type,
	.probe = previewer_probe,
	.remove = previewer_remove,
};
int __init previewer_init(void)
{
	int result;

	/* Register the driver in the kernel */
	/* dynmically get the major number for the driver using 
	   alloc_chrdev_region function */
	result = alloc_chrdev_region(&dev, 0, 1, DRIVERNAME);

	/* if it fails return error */
	if (result < 0) {
		printk("DaVinciPreviewer: Module intialization \
                failed. could not register character device\n");
		return -ENODEV;
	} else {
	}

	/* initialize cdev with file operations */
	cdev_init(&cdev, &prev_fops);

	cdev.owner = THIS_MODULE;
	cdev.ops = &prev_fops;

	/* add cdev to the kernel */
	result = cdev_add(&cdev, dev, 1);

	if (result) {
		unregister_chrdev_region(dev, 1);
		printk("DaVinciPreviewer: Error adding \
		DavinciPreviewer .. error no:%d\n", result);
		return -EINVAL;
	}

	/* register character driver to the kernel */
	register_chrdev(MAJOR(dev), DRIVERNAME, &prev_fops);

	/* register driver as a platform driver */
	if (driver_register(&previewer_driver) != 0) {
		unregister_chrdev_region(dev, 1);
		cdev_del(&cdev);
		return -EINVAL;
	}

	/* Register the drive as a platform device */
	if (platform_device_register(&previewer_device) != 0) {
		driver_unregister(&previewer_driver);
		unregister_chrdev_region(dev, 1);
		unregister_chrdev(MAJOR(dev), DRIVERNAME);
		cdev_del(&cdev);
		return -EINVAL;
	}

	prev_class = class_simple_create(THIS_MODULE, "davinci_previewer");
	if (!prev_class) {
		printk("previewer_init: error in creating device class\n");
		driver_unregister(&previewer_driver);
		platform_device_unregister(&previewer_device);
		unregister_chrdev_region(dev, 1);
		unregister_chrdev(MAJOR(dev), DRIVERNAME);
		cdev_del(&cdev);
		return -EIO;
	}

	/* make entry in the devfs */
	result = devfs_mk_cdev(dev, S_IFCHR | S_IRUGO | S_IWUSR, "%s%d",
			       "davinci_previewer", 0);

	if (result < 0) {
		printk("previewer_init: error in devfs_register_chrdev\n");
		unregister_chrdev_region(dev, 1);
		class_simple_destroy(prev_class);
		unregister_chrdev(MAJOR(dev), DRIVERNAME);
		cdev_del(&cdev);
		return result;
	}

	/* register simple device class */
	class_simple_device_add(prev_class, dev, NULL, "davinci_previewer");

	/* Set up the Interrupt handler for PRVINT interrupt */
	result = request_irq(IRQ_PRVUINT, previewer_isr, SA_INTERRUPT,
			     "dm644xpreviewer", (void *)&prevdevice);

	if (result < 0) {
		printk("previewer_init:cannot get irq\n");

		unregister_chrdev_region(dev, 1);
		class_simple_device_remove(dev);
		devfs_remove("%s%d", "davinci_previewer", 0);
		class_simple_destroy(prev_class);
		driver_unregister(&previewer_driver);
		platform_device_unregister(&previewer_device);
		cdev_del(&cdev);
		unregister_chrdev(MAJOR(dev), DRIVERNAME);
		return -EINVAL;
	}

	prevdevice.opened = 0;
	prev_set_exp(0x2c0);

	return 0;
}

void __exit previewer_cleanup(void)
{

	/* remove major number allocated to this driver */
	unregister_chrdev_region(dev, 1);

	/* remove simple class device */
	class_simple_device_remove(dev);

	/* remove prev device from devfs */
	devfs_remove("%s%d", "davinci_previewer", 0);

	/* destroy simple class */
	class_simple_destroy(prev_class);

	/* Remove platform driver */
	driver_unregister(&previewer_driver);

	/* remove platform device */
	platform_device_unregister(&previewer_device);

	/* disable interrupt */
	free_irq(IRQ_PRVUINT, &prevdevice);

	cdev_del(&cdev);

	/* unregistering the driver from the kernel */
	unregister_chrdev(MAJOR(dev), DRIVERNAME);

}

module_init(previewer_init)
    module_exit(previewer_cleanup)
    MODULE_LICENSE("GPL");
