/*
 * IDCC Serial driver
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License version
 *	2 as published by the Free Software Foundation.
 *
 * This driver was originally based on the ACM driver by Armin Fuerst 
 * (which was based on a driver by Brad Keryan)
 *
 *
 */

#include <linux/hwmon.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <asm/irq.h>
#include <linux/spi/idcc6071.h>
#include <linux/completion.h>
#include <mach/gpio.h>
#include <linux/delay.h>
#include <linux/major.h>

//#define SKIP_CHECK_SRDY
//#define SRDY_EVENT
#define MAX_INTERRUPT_LOOP 100 
#define QUEUE_SIZE         256
#define QUEUE_FLOW_CONTROL  16
/*
 * Version Information
 */
#define DRIVER_AUTHOR "Interdigital, http://www.interdigital.com"
#define DRIVER_DESC "Interdigital SPI Layer"
static int debug = 0;

#define SPI_TTY_MINORS 7
#define MAX_FRAME_SIZE 1528

static unsigned int g_appCounter=0;

//#define IDCC_DEBUG
#ifdef IDCC_DEBUG
#define idcc_printk(f...) printk(f)
#else
#define idcc_printk(f...) do {} while (0)
#endif

struct idcc6071_frhdr_bits {
	unsigned short rts:7;
	unsigned short more:7;
	unsigned short dsr:1;
	unsigned short dcd:1;
};

struct idcc6071_chhdr_bits {
	unsigned short chid: 3;
	unsigned short size: 12;
	unsigned short next:1;
};

union idcc6071_spi_frhdr {
   	struct idcc6071_frhdr_bits data;
	unsigned short value;
};

union idcc6071_spi_chhdr {
	struct idcc6071_chhdr_bits data;
	unsigned short value;
};

#define FRAME_HEADER (sizeof(union idcc6071_spi_frhdr))
#define CHANNEL_HEADER (sizeof(union idcc6071_spi_chhdr))
#define MAX_DATA_SIZE (MAX_FRAME_SIZE - FRAME_HEADER - CHANNEL_HEADER)



typedef struct idcc6071_msg {
	struct list_head queue;
	unsigned char *buf;
	int actual_len;
	int transfer_len;
        unsigned int appCounter;
}idcc6071_msg_t;

struct idcc6071_private_data {
	struct spi_device     *spi;
	struct completion     tx_completion;
	struct completion     srdy_completion;
	struct list_head      msgq;
	struct tty_struct     *tty;
	spinlock_t            lock;
	int                   open_cnt;
        
        struct workqueue_struct *transfer_wq;
	struct work_struct    transfer_work;
        struct list_head      read_pool;
        struct list_head      read_queue;
        
        struct tasklet_struct push;
        unsigned int          n_read;
        unsigned int          read_pool_count;
        unsigned int          throttle;
};

static struct idcc6071_private_data *gpriv;

#ifdef IDCC_DEBUG 
void memdump(unsigned char * data, unsigned int num_bytes, 
	     unsigned short offset )
{
    unsigned int i,j,l;
    unsigned char tmp_str[100];
    unsigned char tmp_str1[10];
    for (i = 0; i < num_bytes; i += 16)    {
        unsigned short n ;      
        tmp_str[0]='\0';
        n = i+offset ;   
        for (j=0; j<4; j++) {
            l=n%16;
            if (l>=10)              
                tmp_str[3-j]=(unsigned char)('A'+l-10);
            else        
                tmp_str[3-j]=(unsigned char)(l+'0');
            n >>= 4 ;       
        }       
        tmp_str[4]='\0';
        strcat ( (char *)tmp_str, ": ");        
        /*          Output the hex bytes        */
        for (j = i; j < (i+16); j++) {          
            int m ;           
            if (j < num_bytes)  {
                m=((unsigned int)((unsigned char)*(data+j)))/16 ;
                if (m>=10)              
                    tmp_str1[0]='A'+(unsigned char)m-10;    
                else                  
                    tmp_str1[0]=(unsigned char)m+'0';
                m=((unsigned int)((unsigned char)*(data+j)))%16 ;
                if (m>=10)                  
                    tmp_str1[1]='A'+(unsigned char)m-10;
                else                    
                    tmp_str1[1]=(unsigned char)m+'0';
                tmp_str1[2]='\0';               
                strcat ((char *)tmp_str, (char *)tmp_str1);      
                strcat ((char *)tmp_str, " ");
            }            
            else {  
                strcat((char *)tmp_str,"   ");       
            }        
        }        
        strcat((char *)tmp_str, "  ");      
        l=(unsigned short)strlen((char *)tmp_str);

        /*         * Output the ASCII bytes        */       
        for (j = i; j < (i+16); j++){
            if (j < num_bytes){
                char c = *(data+j);           
                if (c < ' ' || c > 'z') 
                    c = '.';
                tmp_str[l++]=c;            
            }            
            else
                tmp_str[l++]=' ';
        }
        tmp_str[l++]='\n';        tmp_str[l++]='\0';
        printk("%s", tmp_str);
    }
}
#else
void memdump(unsigned char * data, unsigned int num_bytes, unsigned short offset )
{
	return;
}
#endif

EXPORT_SYMBOL_GPL(memdump);

static int idcc6071_data_avail(struct idcc6071_private_data *priv)
{
    int ret = 0;
    unsigned long flags;

    spin_lock_irqsave(&priv->lock, flags);
    ret = list_empty(&priv->msgq);
    spin_unlock_irqrestore(&priv->lock, flags);
    return (!ret);
}

static unsigned char *alloc_spi_frame(void)
{
	unsigned char* ptr = (unsigned char *)
	    kmalloc(MAX_FRAME_SIZE, GFP_KERNEL);
	memset(ptr, 0, MAX_FRAME_SIZE);
	return ptr;
}

static void free_spi_frame(unsigned char *p)
{
	if (p) {
		kfree(p);
	}
}

static int get_header_len(unsigned char *pFrame) 
{
	union idcc6071_spi_chhdr *pChHdr;

	// Step 1. Read the channel header to get the size
	if (!pFrame)
		return 0;

	pChHdr = (union idcc6071_spi_chhdr *)(pFrame + FRAME_HEADER);
	idcc_printk("ChID = %d\n", pChHdr->data.chid);
	idcc_printk("Size = %d\n", pChHdr->data.size);
	idcc_printk("Next = %d\n", pChHdr->data.next);

	return pChHdr->data.size;
}

static void fill_header(struct idcc6071_private_data *priv,
			int chid, 
			unsigned char *p, 
			int data_len)
{
	union idcc6071_spi_chhdr *pChHdr;
	union idcc6071_spi_frhdr *pFrHdr;

	pFrHdr = (union idcc6071_spi_frhdr *)p;
	pChHdr = (union idcc6071_spi_chhdr *)(p + FRAME_HEADER);

	// Step 1. Setup frame header
#if 0
	if (priv->throttle) {
	    pFrHdr->data.rts = 0x7f & (~(0<<(chid)));
	    printk("\nSetting CTS to %d - flow control\n", pFrHdr->data.rts);
	}
	else
#endif
	    pFrHdr->data.rts = 0x7f & (~(1<<(chid)));


	
	pFrHdr->data.more = 0;
	pFrHdr->data.dsr = 0;
	pFrHdr->data.dcd = 0;

	// Step 2. Setup channel header
	pChHdr->data.chid = chid & 0x7;
	pChHdr->data.size = data_len & 0xfff;
	pChHdr->data.next = 0;

	return;
	
}

static unsigned char *build_tx_spi_frame(struct idcc6071_private_data *priv,
					 struct idcc6071_msg *msg)
{
	unsigned char *p = NULL;
	int len = 0;
	// 1. Read from the list
	// 2. Allocate memory for Tx spi frame : need double check
	// if frame is not allocated, it is dummy frame, then need to alloc
	if (msg)
	{
		p = msg->buf;
		len = msg->actual_len;
	}
	else
	{
		p = alloc_spi_frame();
		len = 0; // since dummy, no data is being sent
	}

	// 3. Put the header: only 1 channel at this time ...
	fill_header(priv, 0, p, len);
	
	// 4. return the frame
	return p;
}


static irqreturn_t idcc6071_irq(int irq, void *handle)
{
	struct idcc6071_private_data *priv = (struct idcc6071_private_data *)handle;

        // 1. Check SRDY level
	int level = omap_get_gpio_datain(SPI_GPIO_SRDY);
        unsigned long flags;

	//idcc_printk("Get IDCC6071_IRQ with level = %d\n", level);
	
	
	// 2. If SRDY level is high, schedule the work
	// Even though it may happen in the middle of work queue,
	// I believe it should be okay. Let it process one more time but dummy work.
	// we can find out a synchronize way later.

	if (level) {

	    // If data is not available, let the write function queue the work
	    spin_lock_irqsave(&priv->lock, flags);
	    if (!idcc6071_data_avail(priv) && (!priv->throttle))
	        queue_work(priv->transfer_wq, &priv->transfer_work);
	    spin_unlock_irqrestore(&priv->lock, flags);

	    // No matter what, we set the srdy_complete event
	    complete(&priv->srdy_completion);
	  
	}
	
	return IRQ_HANDLED;
}

static idcc6071_msg_t *get_free_read_buffer(struct idcc6071_private_data *priv)
{
    struct idcc6071_msg *msg = NULL;
    unsigned long       flags;

    if (!priv) {
        printk("%s Fatal error\n",__func__);
	return NULL;
    }

    idcc_printk("get_free_read_buffer enter\n");
    spin_lock_irqsave(&priv->lock, flags);
    
    if (priv->read_pool_count < QUEUE_FLOW_CONTROL) {
        priv->throttle = 1;
	printk("\n%s Read Queue running low...%d left\n", __func__, 
	       priv->read_pool_count);
	if (priv->read_pool_count == 0) {
	    return NULL;
	}
    }
    
    msg = list_first_entry(&(priv->read_pool), struct idcc6071_msg, queue);

    list_del_init(&msg->queue);
    priv->read_pool_count--;

    spin_unlock_irqrestore(&priv->lock, flags);

    idcc_printk("get_free_read_buf exit msg: %p, msg->buf: %p read_pool_count %d\n",
	   msg, 
	   msg->buf,
	   priv->read_pool_count);

    return msg;

}

static void free_read_buffer(struct idcc6071_private_data *priv,
			struct idcc6071_msg *msg)
{
    unsigned long       flags;

    if (!priv) {
        printk("%s Fatal error\n",__func__);
	return;
    }

    spin_lock_irqsave(&priv->lock, flags);
    list_add_tail(&msg->queue, &priv->read_pool);
    priv->read_pool_count++;
    spin_unlock_irqrestore(&priv->lock, flags);
    idcc_printk("Freed msg: %p, buf: %p read_pool_count %d\n", 
	   msg, 
	   msg->buf, 
	   priv->read_pool_count);
    return;
}


/*
 * RX tasklet takes data out of the RX queue and hands it up to the TTY
 * layer until it refuses to take any more data (or is throttled back).
 * Then it issues reads for any further data.
 *
 * If the RX queue becomes full enough that no usb_request is queued,
 * the OUT endpoint may begin NAKing as soon as its FIFO fills up.
 * So QUEUE_SIZE packets plus however many the FIFO holds (usually two)
 * can be buffered before the TTY layer's buffers (currently 64 KB).
 */
static void idcc6071_rx_push(unsigned long _priv)
{
    struct idcc6071_private_data *priv = 
      (struct idcc6071_private_data *)_priv;
    struct tty_struct       *tty;
    struct list_head        *read_queue = &priv->read_queue;
    struct idcc6071_msg     *msg = NULL;
    //bool		     disconnect = false;
    bool		     do_push = false;
    char		     *packet = NULL;
    unsigned	             size = 0;
    unsigned	             n = 0;
    int		             count = 0;
    unsigned long            flags;

    idcc_printk("tasklet start...\n");
    /* hand any queued data to the tty */
    spin_lock_irqsave(&priv->lock, flags);
    tty = priv->tty;
    
    while (!list_empty(read_queue)) {
      
	msg = list_first_entry(read_queue, struct idcc6071_msg, queue);
      
	/* discard data if tty was closed */
	if (!tty)
	    goto recycle;

	/* leave data queued if tty was rx throttled */
	if (test_bit(TTY_THROTTLED, &tty->flags))
	    break;
	
	/* push data to (open) tty */	
	packet = msg->buf + FRAME_HEADER + CHANNEL_HEADER;
	size = msg->actual_len;
	
	
	/* we may have pushed part of this packet already... */
	n = priv->n_read;
	if (n) {
	    packet += n;
	    size -= n;
	    printk("Advancing %d bytes\n", n);
	}
	
	memdump(packet, 10, 0);

	count = tty_insert_flip_string(tty, packet, size);
	if (count)
	    do_push = true;
	if (count != size) {
	    /* stop pushing; TTY layer can't handle more */
	    priv->n_read += count;
	    printk("Incomplete rx block (fill/total) %d/%d\n",
		   count, msg->actual_len);
	    break;
	}
	priv->n_read = 0;


recycle:
	//Move data to free queue.
	list_move_tail(&msg->queue, &priv->read_pool);
	priv->read_pool_count++;
    }

    /* Push from tty to ldisc; this is immediate with low_latency, and
     * may trigger callbacks to this driver ... so drop the spinlock.
     */
    if (tty && do_push) {
        spin_unlock_irqrestore(&priv->lock, flags);
	tty_flip_buffer_push(tty);
	wake_up_interruptible(&tty->read_wait);
	spin_lock_irqsave(&priv->lock, flags);
	
	/*tty may have been closed */
	tty = priv->tty;
    }


    /* We want our data queue to become empty ASAP, keeping data
     * in the tty and ldisc (not here).  If we couldn't push any
     * this time around, there may be trouble unless there's an
     * implicit tty_unthrottle() call on its way...
     *
     * REVISIT we should probably add a timer to keep the tasklet
     * from starving ... but it's not clear that case ever happens.
     */
    if (!list_empty(read_queue) && tty) {
        if (!test_bit(TTY_THROTTLED, &tty->flags)) {
	    if (do_push)
	        tasklet_schedule(&priv->push);
	    else
	        printk("WARNING: IDCC6071_SPI RX not scheduled?\n");
	}
    }

    spin_unlock_irqrestore(&priv->lock, flags);
}


static void idcc6071_read_complete(struct idcc6071_private_data *priv, 
				   struct idcc6071_msg* msg)
{
    unsigned long       flags;
  
    msg->actual_len = get_header_len(msg->buf);

    spin_lock_irqsave(&priv->lock, flags);
    list_add_tail(&msg->queue, &priv->read_queue);

    //Schedule our tasklet
    tasklet_schedule(&priv->push);
    spin_unlock_irqrestore(&priv->lock, flags);

    idcc_printk("read_complete done\n");
}

/************************************************************************
 * Driver SPI interface functions
 * **********************************************************************/
static void idcc6071_work_thread(struct work_struct *work)
{
	int dataAvail;
	unsigned char *tx_frame;
	struct idcc6071_msg *rx_msg;
	struct spi_message message;
	struct spi_transfer xfer;
	struct idcc6071_private_data *priv = 
	  container_of(work, struct idcc6071_private_data, 
		       transfer_work);
	struct idcc6071_msg *idcc_msg = NULL;
	unsigned long flags;

	idcc_msg = NULL;
	
	// 1. Check if data is available
	//    it can be from a queue list
	dataAvail = idcc6071_data_avail(priv);
	
	// 2. Get the list entry
	if (dataAvail) {
    	    spin_lock_irqsave(&priv->lock, flags);
	    idcc_msg = list_first_entry(&priv->msgq, 
				       struct idcc6071_msg, 
				       queue);	  
	    spin_unlock_irqrestore(&priv->lock, flags);
	}
	
	// 3. Build the spi frame 
	tx_frame = build_tx_spi_frame(priv, idcc_msg);
	rx_msg   = (struct idcc6071_msg *)get_free_read_buffer(priv);
	
	if (!rx_msg) {
	    printk("Bailing out...no buffers left\n");
	    return;
	}
	
	// 4. Pull up MRDY
	omap_set_gpio_dataout(SPI_GPIO_MRDY, 1);
	
	
	// 5. Check if SRDY is set
	wait_for_completion(&priv->srdy_completion);
	
	INIT_COMPLETION((priv->srdy_completion));

	//
	// 6. Delete the msg from the msgq
	if (idcc_msg) {
    	    spin_lock_irqsave(&priv->lock, flags);
	    list_del(&idcc_msg->queue);
	    spin_unlock_irqrestore(&priv->lock, flags);
	}
	
	idcc_printk("Tx Frame:\n");
	
	  
	// 7. Send out the frame (spi_sync)
	spi_message_init(&message);
	memset(&xfer, 0, sizeof(xfer));
	xfer.tx_buf = (void *)tx_frame;
	xfer.rx_buf = (void *)(rx_msg->buf);
	xfer.len = MAX_FRAME_SIZE;
	
	if (idcc_msg) {
	    message.state = (void *)idcc_msg->appCounter;
	}
	else {
	    message.state = (void *)0;
	}
	
	spi_message_add_tail(&xfer, &message);
	spi_sync(priv->spi, &message);
	
	// 12. Notify the write function, it is complete
	if (idcc_msg) {
	    idcc_msg->transfer_len = idcc_msg->actual_len;
	    complete(&priv->tx_completion);
	}

	// 11. Dispatch the spi frame 
	if (priv->open_cnt > 0) {
	    idcc6071_read_complete(priv, rx_msg);
	}
	else {
	    list_move_tail(&rx_msg->queue, &priv->read_pool);
	    priv->read_pool_count++;
	}
	    

	// 12. Pull down the MRDY line
	omap_set_gpio_dataout(SPI_GPIO_MRDY, 0);
	
	// 13. Finally destroy the tx, rx frame
	// Only destroy if create in here, otherwise, do that in write
	if (!idcc_msg)
	    free_spi_frame(tx_frame);
	
	
	idcc_printk("exit the work thread\n");
	return;
}

static int idcc6071_alloc_buffers(struct list_head *head)
{
    int			i;
    struct idcc6071_msg *msg;

    /* Pre-allocate up to QUEUE_SIZE transfers, but if we can't
     * do quite that many this time, don't fail ... we just won't
     * be as speedy as we might otherwise be.
     */
    for (i = 0; i < QUEUE_SIZE; i++) {
        msg = kmalloc(sizeof(struct idcc6071_msg), GFP_KERNEL);
	if (!msg)
	    return list_empty(head) ? -ENOMEM : 0;
	msg->buf = alloc_spi_frame();
	if (!msg->buf)
	    return list_empty(head) ? -ENOMEM : 0;

	list_add_tail(&msg->queue, head);
	idcc_printk("alloc_buffers msg: %p msg->buf: %p\n",msg, msg->buf);
    }
    
    return 0;

}

static int __devinit idcc6071_probe(struct spi_device *spi)
{
	struct idcc6071_private_data *priv;
	int err = 0;
	int level = 0;
	int    ret = 0;

	printk("IDCC6071_PROBE with IRQ = %d MaxFrameSize %d\n", spi->irq, MAX_FRAME_SIZE);
	/* we can reference touch screen ads7846 to configure the SPI */
	if (!spi->irq) {
		dev_dbg(&spi->dev, "no IRQ?\n");
		return -ENODEV;
	}

	priv = (struct idcc6071_private_data *)kmalloc(sizeof(struct idcc6071_private_data), GFP_KERNEL);
	if (!priv) {
		printk("No Memory\n");
		err = -ENOMEM;
		goto err1;
	}

	dev_set_drvdata(&spi->dev, priv);

	// init work queue
	INIT_WORK(&priv->transfer_work, idcc6071_work_thread);
	priv->transfer_wq = create_singlethread_workqueue("idcc6071");
	if (!priv->transfer_wq) {
		printk("Busy work queue\n");
		err = -EBUSY;
		goto err1;
	}
	
	init_completion(&priv->tx_completion);

	init_completion(&priv->srdy_completion);

	INIT_LIST_HEAD(&priv->msgq);
	spin_lock_init(&priv->lock);

	tasklet_init(&priv->push, idcc6071_rx_push, (unsigned long) priv);

	INIT_LIST_HEAD(&priv->read_pool);
	INIT_LIST_HEAD(&priv->read_queue);

	priv->n_read = 0;

	priv->spi = spi;
	priv->open_cnt = 0;

	/* go to arch/arm/mach-omap2/board-ldp.c to configure the GPIO line
	 * for both MRDY and SRDY 
	 */

	/* Configure the SPI */
	spi->mode = SPI_MODE_1; // need to confirm
	spi->bits_per_word = 8; // need to confirm
	spi_setup(spi);

	
	/* register IRQ for SRDY line */
	if(request_irq(spi->irq, idcc6071_irq, IRQF_TRIGGER_RISING,
		spi->dev.driver->name, priv)) {
		dev_dbg(&spi->dev, "irq %d busy?\n", spi->irq);
		err = -EBUSY;
		goto err1;
	}
	
	level = omap_get_gpio_datain(SPI_GPIO_SRDY);
	if (level)	 
	    idcc6071_irq(spi->irq, priv);

	ret = idcc6071_alloc_buffers(&priv->read_pool);
	priv->read_pool_count = QUEUE_SIZE;
	priv->throttle        = 0;

	gpriv = priv;
	
	if (ret != 0) {
	    printk("idcc6071_probe failed\n");
	}
	else
	  printk("IDCC6071_PROBE SUCCESS\n");
	
 	 
	return 0;
err1:
	if (priv)
		kfree(priv);
	return err;
}

static int __devexit idcc6071_remove(struct spi_device *spi)
{
	return 0;
}

static int idcc6071_suspend(struct spi_device *spi, pm_message_t message)
{
	/* Don't worry too much at this point */
	return 0;
}

static int idcc6071_resume(struct spi_device *spi)
{
	/* Don't worry too much at this point */
	return 0;
}

/* Driver structure we register with the USB core */
static struct spi_driver idcc6071_spi_driver = {
	.driver = {
		.name 	= "idcc6071",
		.bus 	= &spi_bus_type,
		.owner  = THIS_MODULE,
	},
	.probe =	idcc6071_probe,
	.remove = 	__devexit_p(idcc6071_remove),
	.suspend =	idcc6071_suspend,
	.resume =	idcc6071_resume,
};


static int idcc6071_free_buffers(struct list_head *head)
{
    struct idcc6071_msg *msg;

    /* Pre-allocate up to QUEUE_SIZE transfers, but if we can't
     * do quite that many this time, don't fail ... we just won't
     * be as speedy as we might otherwise be.
     */
    while (!list_empty(head)) {
        msg = list_entry(head->next, struct idcc6071_msg, queue);
	list_del(&msg->queue);
	free_spi_frame(msg->buf);
	kfree(msg);
    }

    return 0;
}

static void _idcc6071_set_termios(struct tty_struct *tty, struct ktermios *old)
{
    struct idcc6071_private_data *priv = 
        (struct idcc6071_private_data *)tty->driver_data;   
    struct ktermios *termios;
    
    printk("IDCC6071 setting terminal settings\n");

    /*
     * The default requirements for this device are:
     */
    termios = tty->termios;
    termios->c_iflag &=
      ~(IGNBRK	/* disable ignore break */
	| BRKINT	/* disable break causes interrupt */
	| PARMRK	/* disable mark parity errors */
	| ISTRIP	/* disable clear high bit of input characters */
	| INLCR		/* disable translate NL to CR */
	| IGNCR		/* disable ignore CR */
	| ICRNL		/* disable translate CR to NL */
	| IXON);	/* disable enable XON/XOFF flow control */
    
    /* disable postprocess output characters */
    termios->c_oflag &= ~OPOST;
    
    termios->c_lflag &=
      ~(ECHO		/* disable echo input characters */
	| ECHONL	/* disable echo new line */
	| ICANON	/* disable erase, kill, werase, and rprnt
			   special characters */
	| ISIG		/* disable interrupt, quit, and suspend special
			   characters */
	| IEXTEN);	/* disable non-POSIX special characters */
    
    termios->c_cflag &=
      ~(CSIZE		/* no size */
	| PARENB	/* disable parity bit */
	| CBAUD		/* clear current baud rate */
	| CBAUDEX);	/* clear current buad rate */
    
    termios->c_cflag |= CS8;	/* character size 8 bits */
    
    /* baud rate 115200 */
    tty_encode_baud_rate(priv->tty, 115200, 115200);
    
    /*
     * Force low_latency on; otherwise the pushes are scheduled;
     * this is bad as it opens up the possibility of dropping bytes
     * on the floor.  We don't want to drop bytes on the floor. :)
     */
    priv->tty->low_latency = 1;
    return;
}


/*****************************************************************************
 * Driver tty interface functions
 *****************************************************************************/
static int idcc6071_open (struct tty_struct *tty, struct file *filp)
{
	struct idcc6071_private_data *priv = gpriv;

	priv->tty = tty;
	priv->open_cnt++;

	//Temporary storing the private data only. Later on
	// when supporting multiple open, we would need to define
	// struct idcc6071_port
        tty->driver_data = priv;
	
	if (priv->open_cnt == 1) {
	    tty->low_latency = 1;
	    /* Force default termio settings */
	    _idcc6071_set_termios(tty,NULL);
	}
	
	printk("idcc6071_open with tty %p\n",tty);	
	return 0;
}

static void idcc6071_close(struct tty_struct *tty, struct file *filp)
{
        struct idcc6071_private_data *priv = (struct idcc6071_private_data *)tty->driver_data;
        if (priv && priv->open_cnt)
                priv->open_cnt--;
	//idcc6071_free_buffers(&priv->read_queue);
	//idcc6071_free_buffers(&priv->read_pool);
	priv->tty = NULL;
	tty->driver_data = NULL;
}

static int idcc6071_write(struct tty_struct *tty, 
			  const unsigned char *buf,
			  int count)
{
	struct idcc6071_private_data *priv = (struct idcc6071_private_data *)tty->driver_data;
	struct idcc6071_msg *msg;
	int max_transfer_size = MAX_DATA_SIZE;

	// TTY would handle the writing of the rest of buffer
	int trans_len = min(count, max_transfer_size);

	// 1. Copy out the data 
	// allocate a frame through alloc_spi_frame
	//printk("idcc6071_write data len = %d\n", trans_len);
	memdump(buf, trans_len, 0);


	msg = kmalloc(sizeof(struct idcc6071_msg), GFP_KERNEL);
	msg->buf = alloc_spi_frame();
	msg->actual_len = trans_len;
	msg->transfer_len = 0;
	g_appCounter++;
	msg->appCounter = g_appCounter;

	memcpy(&msg->buf[FRAME_HEADER+CHANNEL_HEADER], buf, trans_len);

	
	// 2. Put the data buffer into link list 
	list_add_tail(&msg->queue, &priv->msgq);
	

	// 3. Queue the work
	queue_work(priv->transfer_wq, &priv->transfer_work);


	// 4. Wait until it has been sent out 
	wait_for_completion(&priv->tx_completion);
	
	// 5. Free the memory now
	free_spi_frame(msg->buf);
	kfree(msg);


	// 6. Get the size of transfer
	//printk("idcc6071_write data return %d\n", msg->transfer_len);
	return trans_len;
}

static int idcc6071_chars_in_buffer(struct tty_struct *tty)
{
	int chars = 0;
	struct idcc6071_private_data *priv = (struct idcc6071_private_data *)tty->driver_data;
	struct list_head *tmp;
	struct idcc6071_msg *idcc_msg;


	list_for_each(tmp, &priv->msgq) {
		idcc_msg = list_entry(tmp, struct idcc6071_msg, queue);
		chars += idcc_msg->actual_len;
	}

	return chars;
}

static int idcc6071_write_room(struct tty_struct *tty)
{
	// Since we do kmalloc at run time, we can always put at least one buffer
	// at a time
	return (MAX_FRAME_SIZE - CHANNEL_HEADER - FRAME_HEADER);
}

static void idcc6071_throttle(struct tty_struct *tty)
{
    struct idcc6071_private_data *priv = tty->driver_data;
    unsigned long flags;

    printk("\nThrottle called\n");

    spin_lock_irqsave(&priv->lock, flags);
    priv->throttle = 1;
    spin_unlock_irqrestore(&priv->lock, flags);
}

static void idcc6071_unthrottle(struct tty_struct *tty)
{
    struct idcc6071_private_data *priv = tty->driver_data;
    unsigned long flags;
    printk("\nUnthrottle called\n");

    spin_lock_irqsave(&priv->lock, flags);
    priv->throttle = 0;
    tasklet_schedule(&priv->push);
    
    spin_unlock_irqrestore(&priv->lock, flags);
    //printk("\nUnthrottle called end\n");

    /*Queue dummy work*/
    //if (!idcc6071_data_avail(priv)) {
    queue_work(priv->transfer_wq, &priv->transfer_work);
    //}
}

static int idcc6071_ioctl(struct tty_struct *tty, struct file *file,
					unsigned int cmd, unsigned long arg)
{
	int retval;
	retval = -ENOIOCTLCMD;
	return retval;
}

static void idcc6071_set_termios(struct tty_struct *tty, struct ktermios *old)
{
      struct idcc6071_private_data *priv = 
          (struct idcc6071_private_data *)tty->driver_data;
      unsigned long flags;

      spin_lock_irqsave(&priv->lock, flags);
      if (priv->open_cnt)
	  _idcc6071_set_termios(tty, old);
      else
	  tty->termios = old;
      spin_unlock_irqrestore(&priv->lock, flags);
}


static int idcc6071_break(struct tty_struct *tty, int break_state)
{
	return 0;
}

static int idcc6071_read_proc(char *page, char **start, off_t off, int count,
							int *eof, void *data)
{
	int length = 0;
	int i;
	off_t begin = 0;

	length += sprintf(page, "idcc6071 info\n");
	for (i = 0; i < SPI_TTY_MINORS && length < PAGE_SIZE; ++i) {
		goto done;
	}
	*eof = 1;
done:
	if (off >= (length + begin))
		return 0;
	*start = page + (off-begin);
	return (count < begin+length-off) ? count : begin+length-off;
}

static int idcc6071_tiocmget(struct tty_struct *tty, struct file *file)
{
	return -EINVAL;
}

static int idcc6071_tiocmset(struct tty_struct *tty, struct file *file,
			    unsigned int set, unsigned int clear)
{
	return -EINVAL;
}

static const struct tty_operations idcc6071_ops = {
	.open =			idcc6071_open,
	.close =		idcc6071_close,
	.write =		idcc6071_write,
	.write_room =		idcc6071_write_room,
	.ioctl =		idcc6071_ioctl,
	.set_termios =		idcc6071_set_termios,
	.throttle =		idcc6071_throttle,
	.unthrottle =		idcc6071_unthrottle,
	.break_ctl =		idcc6071_break,
	.chars_in_buffer =	idcc6071_chars_in_buffer,
	.read_proc =		idcc6071_read_proc,
	.tiocmget =		idcc6071_tiocmget,
	.tiocmset =		idcc6071_tiocmset,
};

struct tty_driver *idcc6071_tty_driver;

static int __init idcc6071_init(void)
{
	int result;

	idcc6071_tty_driver = alloc_tty_driver(SPI_TTY_MINORS);
	if (!idcc6071_tty_driver)
		return -ENOMEM;

	/* Initialize our global data */
	idcc6071_tty_driver->owner = THIS_MODULE;
	idcc6071_tty_driver->driver_name = "idcc6071";
	idcc6071_tty_driver->name = 	"ttySPI";
	idcc6071_tty_driver->major = SPI_TTY_MAJOR;
	idcc6071_tty_driver->minor_start = 0;
	idcc6071_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	idcc6071_tty_driver->subtype = SERIAL_TYPE_NORMAL;
#if 0
	idcc6071_tty_driver->flags = TTY_DRIVER_REAL_RAW |
						TTY_DRIVER_DYNAMIC_DEV;
#else

	idcc6071_tty_driver->flags = TTY_DRIVER_REAL_RAW;
#endif
	idcc6071_tty_driver->init_termios = tty_std_termios;
	idcc6071_tty_driver->init_termios.c_cflag = B9600 | CS8 | CREAD
							| HUPCL | CLOCAL;
	idcc6071_tty_driver->init_termios.c_cflag |= (CLOCAL | CREAD); 

	idcc6071_tty_driver->init_termios.c_lflag &= ~(ICANON|ECHO|ECHOE|ISIG);
	idcc6071_tty_driver->init_termios.c_oflag &= ~OPOST;
	idcc6071_tty_driver->init_termios.c_iflag = IGNPAR | IGNBRK;
	idcc6071_tty_driver->init_termios.c_ispeed = 9600;
	idcc6071_tty_driver->init_termios.c_ospeed = 9600;
	idcc6071_tty_driver->init_termios.c_iflag &= ~(IXON| IXOFF);

	tty_set_operations(idcc6071_tty_driver, &idcc6071_ops);
	result = tty_register_driver(idcc6071_tty_driver);
	if (result) {
		printk("%s - tty_register_driver failed", __func__);
		goto exit_bus;
	}

	/* register the SPI driver */
	result = spi_register_driver(&idcc6071_spi_driver);
	if (result < 0) {
		printk("%s - spi_register_driver failed", __func__);
		goto exit_tty;
	}

	printk(KERN_INFO "%s\n", DRIVER_DESC);
	printk("IDCC6071_INIT success\n");

	return result;

exit_tty:
	tty_unregister_driver(idcc6071_tty_driver);

exit_bus:
	printk("%s - returning with error %d", __func__, result);
	put_tty_driver(idcc6071_tty_driver);
	return result;
}


static void __exit idcc6071_exit(void)
{

	spi_unregister_driver(&idcc6071_spi_driver);
	tty_unregister_driver(idcc6071_tty_driver);
	put_tty_driver(idcc6071_tty_driver);
}


module_init(idcc6071_init);
module_exit(idcc6071_exit);

/* Module information */
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

module_param(debug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Debug enabled or not");
