#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/wait.h>
#include <asm/uaccess.h>
#include <linux/sched.h>

#include "bsc-slave.h"
 
#define DRIVER_AUTHOR "Hendrik <hendrik.may@gmx.net>"
#define DRIVER_DESC   "i2c Slave controller"
 

#define IRQ_DESC                "i2c-Slave-irq"
#define DEVICE_NAME             "i2c-slave"
#define DRV_NAME                "i2c-slave-controller" 
#define BUFFER_SIZE             PAGE_SIZE
#define IRQ_NUMBER		43
#define MAX_DEVICES		10

#define BSC_SLAVE_BASE           (BCM2708_PERI_BASE + 0x214000)


/* BSC SLAVE register offsets */
#define BSC_DR			0x00
#define BSC_RSR 		0x04
#define BSC_SLV			0x08
#define BSC_CR  		0x0c
#define BSC_FR	        	0x10
#define BSC_IFLS		0x14
#define BSC_IMSC		0x18
#define BSC_RIS			0x1c
#define BSC_MIS			0x20
#define BSC_ICR			0c24
#define BSC_DMACR		0x28
#define BSC_TDR                 0x2c
#define BSC_GPUSTAT		0x30
#define BSC_HCTRL		0x34
#define BSC_DEBUG		0x3c



/* Bitfields in DR */
#define BSC_DR_RXFLEVEL_MASK	0xF8000000
#define BSC_DR_TXFLEVEL_MASK	0x07c00000
#define BSC_DR_RXBUSY		0x00200000
#define BSC_DR_TXFE		0x00100000
#define BSC_DR_RXFF		0x00080000
#define BSC_DR_TXFF		0x00040000
#define BSC_DR_RXFE		0x00020000
#define BSC_DR_TXBUSY		0x00010000
#define BSC_DR_DATA_MASK	0x000000FF
 
/* Bitfields in CR */
#define BSC_CR_TESTFIFO		0x00000800
#define BSC_CR_RXE              0x00000200
#define BSC_CR_TXE              0x00000100
#define BSC_CR_BRK              0x00000080
#define BSC_CR_CPOL             0x00000010
#define BSC_CR_CPHA             0x00000008
#define BSC_CR_I2C              0x00000004
#define BSC_CR_SPI	        0x00000002
#define BSC_CR_EN	        0x00000001

/* Bitfields in IMSC */
#define BSC_IMSC_TXIM           0x00000002
#define BSC_IMSC_RXIM		0x00000001

/* Bitfields in MIS */
#define BSC_MIS_TXMIS           0x00000002
#define BSC_MIS_RXMIS		0x00000001

/* Bitfields in ICR */
#define BSC_ICR_TXIC		0x00000002

/*Bitfields in IFLS*/
#define BSC_IFLS_ONE_EIGHTS	0x00000000
#define BSC_IFLS_ONE_QUART      0x00000024
#define BSC_IFLS_ONE_HALF       0x00000012

/* Bitfields in FR */
#define BSC_FR_TXFF		0x00000004
#define BSC_FR_RXFE             0x00000002
#define BSC_FR_TXBUSY           0x00000001


struct bcm2708_i2c_slave_struct {
       void  __iomem          *base;
       int                     irq;
       struct cdev             cdev;
       dev_t		       dev_number;
       unsigned long           rx_buf;
       unsigned long volatile  rx_buf_head;
       volatile unsigned long  rx_buf_tail;
       unsigned long           tx_buf;
       unsigned long volatile  tx_buf_head;
       volatile unsigned long  tx_buf_tail;
} *i2c_slave_dev;

                          //thoses variables should be in the bcm.. struct
DECLARE_WAIT_QUEUE_HEAD(i2c_slave_inq);
DECLARE_WAIT_QUEUE_HEAD(i2c_slave_outq);

static int major;
static struct class *i2c_slave_class;
static struct device *i2c_slave_device;

/***************************************************************************/
/*         Helper function to increment pointer on circular buffer         */
/***************************************************************************/
static inline void i2c_slave_incr_buffer_pointer(volatile unsigned long *index,
                                                 int delta,
                                                 unsigned long buffer){
  unsigned long new = *index + delta;
  barrier();
  *index = (new >= (buffer + BUFFER_SIZE)) ? buffer : new;
}


/****************************************************************************/
/*                     Initialize GPIO for BSC-control                      */
/****************************************************************************/
static void bcm2708_init_i2c_pinmode(int on) {

 	void __iomem *gpio;
        u32 reg;

  gpio = ioremap(GPIO_BASE, SZ_16K);

  if(on == 1){                           //set alternative function for pins
     reg = readl(gpio + 0x04);
     reg &= ~(0b111111 << 24);
     writel(reg, gpio + 0x04);
     reg |= (0b111111 << 24);            //ALT3 for Pins 18 and 19
     writel(reg, gpio + 0x04);

     reg = readl(gpio + 0x08);
     reg &= ~0b111111;
     writel(reg, gpio + 0x08);
     reg |= 0b111111;                    //ALT3 for Pins 20 ans 21
     writel(reg, gpio + 0x08);

  } else if(on == 0){                    //reset pins to be inputs
     reg = readl(gpio + 0x04);
     reg &= ~(0b111111 << 24);
     writel(reg, gpio + 0x04);

     reg = readl(gpio + 0x08);
     reg &= ~0b111111;
     writel(reg, gpio + 0x08);

  } else {
  }

  iounmap(gpio);
}



/****************************************************************************/
/* IRQ handler - fired on interrupt                                         */
/****************************************************************************/
static irqreturn_t i2c_slave_irq(int irq, void *dev_id) {

   u32 reg, stat_reg;
   struct bcm2708_i2c_slave_struct *i2c_slave = dev_id;
   int tx_value_count;


   stat_reg = readl(i2c_slave->base + BSC_MIS);      //interrupt status reg
                                                     // clear error register
   writel(0, i2c_slave->base + BSC_RSR);

   if(stat_reg & BSC_MIS_RXMIS){
                                                //while RX_FIFO not empty
     while( !((reg = readl(i2c_slave->base + BSC_FR)) & BSC_FR_RXFE) ){

       reg = readl(i2c_slave->base + BSC_DR);       //read RX_FIFO
       *( (u8 *)i2c_slave->rx_buf_head) = (0xff & reg);
       i2c_slave_incr_buffer_pointer(&i2c_slave->rx_buf_head,
                                      1,
                                      i2c_slave->rx_buf);
     }

   wake_up_interruptible(&i2c_slave_inq);

   }

   
   if(stat_reg & BSC_MIS_TXMIS){

     tx_value_count = i2c_slave->tx_buf_head - i2c_slave->tx_buf_tail;
     reg = readl(i2c_slave->base + BSC_FR);
                                             //if TX buffer empty, fill TX FIFO
                                             //with zeros to stop interrupts
     if(tx_value_count == 0)
        writel(0, i2c_slave->base + BSC_DR);
                                               //while space in TX FIFO
     while(( (!(reg & BSC_FR_TXFF)) && (tx_value_count != 0)) ){

          writel(*( (u8 *)i2c_slave->tx_buf_tail ), i2c_slave->base + BSC_DR);
          i2c_slave_incr_buffer_pointer(&i2c_slave->tx_buf_tail,
                                          1,
                                          i2c_slave->tx_buf);
          tx_value_count = i2c_slave->tx_buf_head - i2c_slave->tx_buf_tail;
          reg = readl(i2c_slave->base + BSC_FR);

     }  

   wake_up_interruptible(&i2c_slave_outq);
 
   }

   return IRQ_HANDLED;
}

/****************************************************************************/
/*                              Device open                                 */
/****************************************************************************/
static int i2c_slave_open(struct inode *inode, struct file *filp){

  struct bcm2708_i2c_slave_struct *i2c_slave;
  u32 reg = 0;

  i2c_slave = container_of(inode->i_cdev,
                           struct bcm2708_i2c_slave_struct, cdev);
                                         //save device pointer for later use
                                         //in read() and write()
  filp->private_data = i2c_slave;

  
                     //allocate 2^0 (one) page(s) of memory for RX Buffer
  i2c_slave->rx_buf = __get_free_pages(GFP_KERNEL, 0);
  if(IS_ERR((void *)i2c_slave->rx_buf)){
     printk(KERN_NOTICE "c'ant allocate buffer memory");
     return -ENOMEM;
  }
  i2c_slave->rx_buf_head = i2c_slave->rx_buf;
  i2c_slave->rx_buf_tail = i2c_slave->rx_buf;

                   //allocate TX Buffer
  i2c_slave->tx_buf = __get_free_pages(GFP_KERNEL, 0);
  if(IS_ERR((void *)i2c_slave->tx_buf)){
     printk(KERN_NOTICE "c'ant allocate buffer memory");
     return -ENOMEM;
  }
  i2c_slave->tx_buf_head = i2c_slave->tx_buf;
  i2c_slave->tx_buf_tail = i2c_slave->tx_buf;

                                         //select interrupt level for FIFOS
  writel(BSC_IFLS_ONE_EIGHTS, i2c_slave->base + BSC_IFLS);
                                         //enable interrupts for 
                                         //RX and TX FIFO level crossing
  reg = BSC_IMSC_RXIM | BSC_IMSC_TXIM;
  writel(reg, i2c_slave->base + BSC_IMSC);
                                         //clear errors
  writel(0, i2c_slave->base + BSC_RSR);

  reg = BSC_CR_BRK;                      //clear FIFOs  
  writel(reg, i2c_slave->base + BSC_CR);

  reg  = BSC_CR_TXE;                     //enable transmit mode FIFO
  reg |= (BSC_CR_EN | BSC_CR_I2C);       //enable i2c mode and device
  writel(reg, i2c_slave->base + BSC_CR);


  return 0;
}

/****************************************************************************/
/*                              Device release                              */
/****************************************************************************/
int i2c_slave_release(struct inode *inode, struct file *filp){

  struct bcm2708_i2c_slave_struct *i2c_slave;   

  i2c_slave = filp->private_data;               

  writel(0, i2c_slave->base + BSC_IMSC);    //disable interrupts
  writel(0, i2c_slave->base + BSC_CR);      //disable device

  free_page(i2c_slave->rx_buf);             //free TX and RX ringbuffers
  free_page(i2c_slave->tx_buf);

  return 0;
}


/****************************************************************************/
/*                              Device read                                 */
/****************************************************************************/
static ssize_t i2c_slave_read(struct file *filp, char __user *buf, size_t count,
                       loff_t *fpos){

  struct bcm2708_i2c_slave_struct *i2c_slave;
  DECLARE_WAITQUEUE(wait, current);
  int count0;                                 //number of readable databytes

  i2c_slave = filp->private_data;      
                                              //while RX buffer empty 
  while(i2c_slave->rx_buf_head == i2c_slave->rx_buf_tail){
    add_wait_queue(&i2c_slave_inq, &wait);
    set_current_state(TASK_INTERRUPTIBLE);
    if(i2c_slave->rx_buf_head == i2c_slave->rx_buf_tail){
       schedule();
    }
    set_current_state(TASK_RUNNING);
    remove_wait_queue(&i2c_slave_inq, &wait);
    if(signal_pending(current))
      return -ERESTARTSYS;
  }

  count0 = i2c_slave->rx_buf_head - i2c_slave->rx_buf_tail;

  if(count0 < 0){              //buffer pointer wrapped, read till wrapp point
     count0 = BUFFER_SIZE - i2c_slave->rx_buf_tail + i2c_slave->rx_buf;
  }
  if(count0 < count){         //user program want's more than possible
     count = count0;          //give him the maximum possible
  }

  if(copy_to_user(buf, (char *)i2c_slave->rx_buf_tail, count))
     return -EFAULT;

  i2c_slave_incr_buffer_pointer(&i2c_slave->rx_buf_tail,
                                 count,
                                 i2c_slave->rx_buf);
  fpos += count;

  return count;
}

/****************************************************************************/
/*                              write method                                */
/****************************************************************************/ 

static ssize_t i2c_slave_write(struct file *filp, const char *buf, size_t count,
                       loff_t *fpos){

 struct bcm2708_i2c_slave_struct *i2c_slave;
 int count0, space = 0;
 DECLARE_WAITQUEUE(wait_tx, current);

 i2c_slave = filp->private_data;

 count0 = i2c_slave->tx_buf_head - i2c_slave->tx_buf_tail;

                                      //while no space in TX buffer
 while( ((count0 == -1) || (count0 == BUFFER_SIZE-1)) ){
    add_wait_queue(&i2c_slave_outq, &wait_tx);
    set_current_state(TASK_INTERRUPTIBLE);
    count0 = i2c_slave->tx_buf_head - i2c_slave->tx_buf_tail;
    if(((count0 == -1) || (count0 == BUFFER_SIZE-1))){
       schedule();
    }
    set_current_state(TASK_RUNNING);
    remove_wait_queue(&i2c_slave_outq, &wait_tx);
    if(signal_pending(current))
      return -ERESTARTSYS;
    count0 = i2c_slave->tx_buf_head - i2c_slave->tx_buf_tail;
  }


 if(count0 > -1){
    if(i2c_slave->tx_buf_head == (i2c_slave->tx_buf + BUFFER_SIZE - 1) ){
      space = 1;
    } else { 
      space = i2c_slave->tx_buf + BUFFER_SIZE - i2c_slave->tx_buf_head - 1;
    }
 } 

 if(count0 < -1){                               //buffer pointer wrapped
    space = i2c_slave->tx_buf_tail - i2c_slave->tx_buf_head - 1;     
 }

 if(count > space){
       count = space;
 }  


 if(copy_from_user((char *)i2c_slave->tx_buf_head, buf, count)){
         return -EFAULT;
       }


 i2c_slave_incr_buffer_pointer(&i2c_slave->tx_buf_head,
                                      count,
                                      i2c_slave->tx_buf );
 fpos += count;


 return count;
}

/****************************************************************************/
/*                              ioctl method                                */
/****************************************************************************/ 

static long i2c_slave_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
        struct bcm2708_i2c_slave_struct *i2c_slave = filp->private_data;       


        switch (cmd) {
        case I2C_SLAVE:
        case I2C_SLAVE_FORCE:
               
                if ((arg > 0x7f))
                        return -EINVAL;

                writel(arg, i2c_slave->base + BSC_SLV);
                return 0;
      
        default:
                /* NOTE:  returning a fault code here could cause trouble
                 * in buggy userspace code.  Some old kernel bugs returned
                 * zero in this case, and userspace code might accidentally
                 * have depended on that bug.
                 */
                return -ENOTTY;
        }
        return 0;
}





static struct file_operations i2c_slave_fops = {
        .owner          =  THIS_MODULE,
        .open           =  i2c_slave_open,
        .release        =  i2c_slave_release,
        .read           =  i2c_slave_read,
        .write          =  i2c_slave_write,
        .unlocked_ioctl =  i2c_slave_ioctl,
};


/****************************************************************************/
/*                               Module init                                */
/****************************************************************************/

int __init bcm2708_i2c_slave_init(void){

  int ret = -ENODEV;
  dev_t dev_number;      
  struct bcm2708_i2c_slave_struct *i2c_slave;

 
  ret = alloc_chrdev_region(&dev_number, 0, MAX_DEVICES, DEVICE_NAME);
  major = MAJOR(dev_number);
  if( ret < 0){
    printk(KERN_ERR "alloc_chrdev_region failed for i2cslave\n");
    return -ENODEV;
  }

                       //Poplate sysfs entries
  i2c_slave_class = class_create(THIS_MODULE, DEVICE_NAME);
  if(!i2c_slave_class){
     printk("c'ant create class!\n");
     goto class_fail;
  }

                      //allocate memory for the per device structure
  i2c_slave = kmalloc(sizeof(struct bcm2708_i2c_slave_struct), GFP_KERNEL);
  if(!i2c_slave){
    printk("c'ant allocate memory!\n");
    ret = -ENOMEM;
    goto mem_fail;
  }
                      //Request I/O Region
  if(!request_mem_region(BSC_SLAVE_BASE, 64, DEVICE_NAME)){
     printk("i2c-slave: I/O Port 0x%x is not free!\n", BSC_SLAVE_BASE);
     ret = -EIO;
     goto mem_reg_fail;
  }

  i2c_slave->dev_number = MKDEV(major, 0);          
  cdev_init(&i2c_slave->cdev, &i2c_slave_fops);      
  ret = cdev_add(&i2c_slave->cdev, i2c_slave->dev_number,1);
  if(ret){
     goto cdev_init_fail;
  }

  i2c_slave_device = device_create(i2c_slave_class, NULL,  
                                   i2c_slave->dev_number,
                                   i2c_slave, DEVICE_NAME);

  if(IS_ERR(i2c_slave_device)){
     printk(KERN_NOTICE "C'ant create device in sysfs!\n");
     goto dev_create_fail;
  }

                      //initialize pins 18,19,20,21 for i2c-slave mode
  bcm2708_init_i2c_pinmode(1);

                      //link i2c-slave interrupt handler with irq number (43)
  ret = request_irq(IRQ_NUMBER, i2c_slave_irq, 0, DEVICE_NAME,
                    i2c_slave);
  if(ret){
     printk(KERN_NOTICE "could not request IRQ: %d!\n", ret);
     goto irq_fail;
  }

  i2c_slave->irq = IRQ_NUMBER;

  i2c_slave->base = ioremap(BSC_SLAVE_BASE, SZ_256-1);  //is size right???
  if(!i2c_slave->base){
     printk(KERN_NOTICE "could not remap memory!\n");
     goto remap_fail;
  }

  printk(KERN_NOTICE "i2c-slave at 0x%08lx (irq %d)\n",
                       (unsigned long)BSC_SLAVE_BASE, IRQ_NUMBER);

  i2c_slave_dev = i2c_slave;

  return 0;

  remap_fail:
     free_irq(IRQ_NUMBER, i2c_slave_dev);  

  irq_fail:
     device_destroy(i2c_slave_class, MKDEV(major,0));
     bcm2708_init_i2c_pinmode(0);

  dev_create_fail:
     cdev_del(&i2c_slave->cdev);

  cdev_init_fail:
     release_mem_region(BSC_SLAVE_BASE, 64);

  mem_reg_fail:
     kfree(i2c_slave);

  mem_fail:
     class_destroy(i2c_slave_class);

  class_fail:
     unregister_chrdev_region(dev_number, MAX_DEVICES);
     return ret;
 
}

/****************************************************************************/
/*                              Module exit                                 */
/****************************************************************************/
void __exit bcm2708_i2c_slave_cleanup(void){

  printk(KERN_NOTICE "i2c-slave module removed!\n");

  device_destroy(i2c_slave_class, MKDEV(major,0));
  class_destroy(i2c_slave_class);


  unregister_chrdev_region(MKDEV(major, 0), MAX_DEVICES);
  cdev_del(&i2c_slave_dev->cdev);

  release_mem_region(BSC_SLAVE_BASE, 64);

  bcm2708_init_i2c_pinmode(0);

  free_irq(IRQ_NUMBER, i2c_slave_dev);   
  iounmap(i2c_slave_dev->base); 
  kfree(i2c_slave_dev);              
 
}



module_init(bcm2708_i2c_slave_init);
module_exit(bcm2708_i2c_slave_cleanup);


/****************************************************************************/
/* Module licensing/description block.                                      */
/****************************************************************************/

MODULE_LICENSE("GPL");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_ALIAS("chdrv:" DRV_NAME);
