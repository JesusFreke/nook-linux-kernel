/*
 * linux/drivers/video/g3d/s3c_g3d.c
 *
 * Revision 1.0
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *	    S3C G3D driver
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/timer.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/errno.h> 	/* error codes */
#include <asm/div64.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/mman.h>

#include <linux/unistd.h>

#include <linux/version.h>
#include <asm/dma.h>
#include <asm/cacheflush.h>
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>

#include <mach/dma.h>
#include <mach/hardware.h>
#include <mach/map.h>
#include <plat/dma.h>
#include <plat/power-clock-domain.h>
#include <plat/pm.h>

#define DEBUG_S3C_G3D
#undef	DEBUG_S3C_G3D

#ifdef DEBUG_S3C_G3D
#define DEBUG(fmt,args...) printk(fmt, ##args)
#else
#define DEBUG(fmt,args...) do {} while(0)
#endif


typedef struct{
	unsigned int pool_buffer_addr;
	unsigned int pool_buffer_size;
	unsigned int dma_buffer_addr;
	unsigned int dma_buffer_size;
} G3D_CONFIG_STRUCT;

typedef struct{
	unsigned int offset; 	// should be word aligned
	unsigned int size; 	// byte size, should be word aligned
} DMA_BLOCK_STRUCT;

typedef struct {
	ulong src;
	ulong dst;
	int len;
} s3c_3d_dma_info;

#define FGGB_PIPESTATE		0x00
#define FGGB_CACHECTL		0x04
#define FGGB_RST		0x08
#define FGGB_VERSION		0x10
#define FGGB_INTPENDING		0x40
#define FGGB_INTMASK		0x44
#define FGGB_PIPEMASK		0x48
#define FGGB_HOSTINTERFACE	0xc000

G3D_CONFIG_STRUCT g3d_config={
	0x56000000, 	// pool buffer addr
	0x1800000, 	//pool buffer size,24Mb
	0x57800000, 	//dma buffer addr
	0x800000 	//dma buffer size
};

#define G3D_IOCTL_MAGIC		'S'
#define WAIT_FOR_FLUSH		_IO(G3D_IOCTL_MAGIC, 100)
#define GET_CONFIG 		_IO(G3D_IOCTL_MAGIC, 101)
#define START_DMA_BLOCK 	_IO(G3D_IOCTL_MAGIC, 102)

#define S3C_3D_MEM_ALLOC		_IOWR(G3D_IOCTL_MAGIC, 310, struct s3c_3d_mem_alloc)
#define S3C_3D_MEM_FREE			_IOWR(G3D_IOCTL_MAGIC, 311, struct s3c_3d_mem_alloc)
#define S3C_3D_SFR_LOCK			_IO(G3D_IOCTL_MAGIC, 312)
#define S3C_3D_SFR_UNLOCK		_IO(G3D_IOCTL_MAGIC, 313)
#define S3C_3D_MEM_ALLOC_SHARE		_IOWR(G3D_IOCTL_MAGIC, 314, struct s3c_3d_mem_alloc)
#define S3C_3D_MEM_SHARE_FREE		_IOWR(G3D_IOCTL_MAGIC, 315, struct s3c_3d_mem_alloc)
#define S3C_3D_CACHE_INVALID  _IOWR(G3D_IOCTL_MAGIC, 316, struct s3c_3d_mem_alloc)
#define S3C_3D_CACHE_CLEAN    _IOWR(G3D_IOCTL_MAGIC, 317, struct s3c_3d_mem_alloc)

#define MEM_ALLOC		1
#define MEM_ALLOC_SHARE		2

#define PFX 			"s3c_g3d"
#define G3D_MINOR  		249

static wait_queue_head_t waitq;
static struct resource *s3c_g3d_mem;
static void __iomem *s3c_g3d_base;
static int s3c_g3d_irq;
static struct clk *g3d_clock;
static struct clk *h_clk;

static DEFINE_MUTEX(mem_alloc_lock);
static DEFINE_MUTEX(mem_free_lock);
static DEFINE_MUTEX(mem_sfr_lock);

static DEFINE_MUTEX(mem_alloc_share_lock);
static DEFINE_MUTEX(mem_share_free_lock);

void *dma_3d_done;

static struct s3c2410_dma_client s3c6410_3d_dma_client = {
	.name		= "s3c6410-3d-dma",
};


struct s3c_3d_mem_alloc {
	int		size;
	unsigned int 	vir_addr;
	unsigned int 	phy_addr;
};

static unsigned int mutex_lock_processID = 0;

static int flag = 0;

static unsigned int physical_address;

int interrupt_already_recevied;

unsigned int s3c_g3d_base_physical;



///////////// for check memory leak
//*-------------------------------------------------------------------------*/
typedef struct _memalloc_desc
{
	int		size;
	unsigned int 	vir_addr;
	unsigned int 	phy_addr;
	int*    newid;	
	struct _memalloc_desc*  next;
	struct _memalloc_desc*  prev;
} Memalloc_desc;

Memalloc_desc   HeadChunk;
#define pHeadChunk (&HeadChunk)
Memalloc_desc*  LastChunk = pHeadChunk;

void grabageCollect(int *newid);

/////////////////////////////////////


irqreturn_t s3c_g3d_isr(int irq, void *dev_id)
{
	__raw_writel(0, s3c_g3d_base + FGGB_INTPENDING);

	interrupt_already_recevied = 1;
	wake_up_interruptible(&waitq);

	return IRQ_HANDLED;
}


void s3c_g3d_dma_finish(struct s3c2410_dma_chan *dma_ch, void *buf_id,
	int size, enum s3c2410_dma_buffresult result){
//	printk("3d dma transfer completed.\n");
	complete(dma_3d_done);
}

int s3c_g3d_open(struct inode *inode, struct file *file)
{
    int *newid;
    s3c_set_normal_cfg(S3C64XX_DOMAIN_G, S3C64XX_ACTIVE_MODE, S3C64XX_3D);
    if(s3c_wait_blk_pwr_ready(S3C64XX_BLK_G)){
        return -1;
    }
    newid = (int*)vmalloc(sizeof(int));
    file->private_data = newid;
	return 0;
}

int s3c_g3d_release(struct inode *inode, struct file *file)
{
    int *newid = file->private_data;
    if(mutex_lock_processID != 0 && mutex_lock_processID == file->private_data)
    {
        mutex_unlock(&mem_sfr_lock);
        printk("Abnormal close of pid # %d\n", task_pid_nr(current));        
    }
    
    grabageCollect(newid);
    vfree(newid);
    s3c_set_normal_cfg(S3C64XX_DOMAIN_G, S3C64XX_LP_MODE, S3C64XX_3D);

	return 0;
}

int s3c_g3d_read(struct file *file, char *buf, size_t count, loff_t *f_pos)
{
	return 0;
}

int s3c_g3d_write(struct file *file, const char *buf, size_t count, loff_t *f_pos)
{
	return 0;
}

static int s3c_g3d_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, unsigned long arg)
{

	u32 val;
	DMA_BLOCK_STRUCT dma_block;
	s3c_3d_dma_info dma_info;
	DECLARE_COMPLETION_ONSTACK(complete);

	u_int virt_addr;
	struct mm_struct *mm = current->mm;
	struct s3c_3d_mem_alloc param;
	
	Memalloc_desc   *memdesc;

	switch (cmd) {
	case WAIT_FOR_FLUSH:

		//if fifo has already been flushed, return;
		val = __raw_readl(s3c_g3d_base+FGGB_PIPESTATE);
		//printk("read pipestate = 0x%x\n",val);
		if((val & arg) ==0) break;

		// enable interrupt
		interrupt_already_recevied = 0;
		__raw_writel(0x0001171f,s3c_g3d_base+FGGB_PIPEMASK);
		__raw_writel(1,s3c_g3d_base+FGGB_INTMASK);

		//printk("wait for flush (arg=0x%lx)\n",arg);


		while(1) {
			wait_event_interruptible(waitq, (interrupt_already_recevied>0));
			__raw_writel(0,s3c_g3d_base+FGGB_INTMASK);
			interrupt_already_recevied = 0;
			//if(interrupt_already_recevied==0)interruptible_sleep_on(&waitq);
			val = __raw_readl(s3c_g3d_base+FGGB_PIPESTATE);
			//printk("in while read pipestate = 0x%x\n",val);
			if(val & arg){
			} else{
				break;
			}
			__raw_writel(1,s3c_g3d_base+FGGB_INTMASK);
		}
		break;

	case GET_CONFIG:
		copy_to_user((void *)arg,&g3d_config,sizeof(G3D_CONFIG_STRUCT));
		break;

	case START_DMA_BLOCK:
		copy_from_user(&dma_block,(void *)arg,sizeof(DMA_BLOCK_STRUCT));

		if (dma_block.offset%4!=0) {
			printk("G3D: dma offset is not aligned by word\n");
			return -EINVAL;
		}
		if (dma_block.size%4!=0) {
			printk("G3D: dma size is not aligned by word\n");
			return -EINVAL;
		}
		if (dma_block.offset+dma_block.size >g3d_config.dma_buffer_size) {
			printk("G3D: offset+size exceeds dam buffer\n");
			return -EINVAL;
		}

		dma_info.src = g3d_config.dma_buffer_addr+dma_block.offset;
		dma_info.len = dma_block.size;
		dma_info.dst = s3c_g3d_base_physical+FGGB_HOSTINTERFACE;

		DEBUG(" dma src=0x%x\n", dma_info.src);
		DEBUG(" dma len =%u\n", dma_info.len);
		DEBUG(" dma dst = 0x%x\n", dma_info.dst);

		dma_3d_done = &complete;

		if (s3c2410_dma_request(DMACH_3D_M2M, &s3c6410_3d_dma_client, NULL)) {
			printk(KERN_WARNING "Unable to get DMA channel(DMACH_3D_M2M).\n");
			return -EFAULT;
		}

		s3c2410_dma_set_buffdone_fn(DMACH_3D_M2M, s3c_g3d_dma_finish);
		s3c2410_dma_devconfig(DMACH_3D_M2M, S3C_DMA_MEM2MEM, 1, (u_long) dma_info.src);
		s3c2410_dma_config(DMACH_3D_M2M, 4, 4);
		s3c2410_dma_setflags(DMACH_3D_M2M, S3C2410_DMAF_AUTOSTART);

		//consistent_sync((void *) dma_info.dst, dma_info.len, DMA_FROM_DEVICE);
	//	s3c2410_dma_enqueue(DMACH_3D_M2M, NULL, (dma_addr_t) virt_to_dma(NULL, dma_info.dst), dma_info.len);
		s3c2410_dma_enqueue(DMACH_3D_M2M, NULL, (dma_addr_t) dma_info.dst, dma_info.len);

	//	printk("wait for end of dma operation\n");
		wait_for_completion(&complete);
	//	printk("dma operation is performed\n");

		s3c2410_dma_free(DMACH_3D_M2M, &s3c6410_3d_dma_client);

		break;

	case S3C_3D_MEM_ALLOC:		
		mutex_lock(&mem_alloc_lock);
		if(copy_from_user(&param, (struct s3c_3d_mem_alloc *)arg, sizeof(struct s3c_3d_mem_alloc))){
			mutex_unlock(&mem_alloc_lock);			
			return -EFAULT;
		}
		flag = MEM_ALLOC;

		param.vir_addr = do_mmap(file, 0, param.size, PROT_READ|PROT_WRITE, MAP_SHARED, 0);
		DEBUG("param.vir_addr = %08x\n", param.vir_addr);

		if(param.vir_addr == -EINVAL) {
			printk("S3C_3D_MEM_ALLOC FAILED\n");
			flag = 0;
			mutex_unlock(&mem_alloc_lock);			
			return -EFAULT;
		}
		param.phy_addr = physical_address;

       // printk("alloc %d\n", param.size);
		DEBUG("KERNEL MALLOC : param.phy_addr = 0x%X \t size = %d \t param.vir_addr = 0x%X\n", param.phy_addr, param.size, param.vir_addr);

		if(copy_to_user((struct s3c_3d_mem_alloc *)arg, &param, sizeof(struct s3c_3d_mem_alloc))){
			flag = 0;
			mutex_unlock(&mem_alloc_lock);
			return -EFAULT;		
		}

		flag = 0;
		
		//////////////////////////////////
		// for memory leak
		memdesc = (Memalloc_desc*)vmalloc(sizeof(Memalloc_desc));
		memdesc->size = param.size;
		memdesc->vir_addr = param.vir_addr;
		memdesc->phy_addr = param.phy_addr;
		memdesc->newid = (int*)file->private_data;
		memdesc->next = NULL;
		LastChunk->next = memdesc;
		memdesc->prev = LastChunk;	
		LastChunk = memdesc;	
		//////////////////////////////////
		
		mutex_unlock(&mem_alloc_lock);
		
		break;

	case S3C_3D_MEM_FREE:	
		mutex_lock(&mem_free_lock);
		if(copy_from_user(&param, (struct s3c_3d_mem_alloc *)arg, sizeof(struct s3c_3d_mem_alloc))){
			mutex_unlock(&mem_free_lock);
			return -EFAULT;
		}

		DEBUG("KERNEL FREE : param.phy_addr = 0x%X \t size = %d \t param.vir_addr = 0x%X\n", param.phy_addr, param.size, param.vir_addr);

		if (do_munmap(mm, param.vir_addr, param.size) < 0) {
			printk("do_munmap() failed !!\n");
			mutex_unlock(&mem_free_lock);
			return -EINVAL;
		}
		virt_addr = phys_to_virt(param.phy_addr);
		//printk("KERNEL : virt_addr = 0x%X\n", virt_addr);
		//printk("free %d\n", param.size);

		kfree(virt_addr);
		param.size = 0;
		DEBUG("do_munmap() succeed !!\n");

		if(copy_to_user((struct s3c_3d_mem_alloc *)arg, &param, sizeof(struct s3c_3d_mem_alloc))){
			mutex_unlock(&mem_free_lock);
			return -EFAULT;
		}
		
		//////////////////////////////////
		// for memory leak
        for(memdesc=pHeadChunk->next; memdesc != NULL ; memdesc=memdesc->next)
        {
            if(memdesc->newid == (int*)file->private_data && memdesc->vir_addr == param.vir_addr)
            {
            	if(memdesc == LastChunk)
            	{
            	    memdesc->prev->next = NULL;
            	    LastChunk = memdesc->prev;	    
            	}
            	else
            	{
            	    memdesc->prev->next = memdesc->next;
            	    memdesc->next->prev = memdesc->prev;
            	}
            	vfree(memdesc);
            	break;
            }        
        }	
		//////////////////////////////////		
		
		mutex_unlock(&mem_free_lock);
		
		break;

	case S3C_3D_SFR_LOCK:
		mutex_lock(&mem_sfr_lock);
		mutex_lock_processID = file->private_data;
		DEBUG("s3c_g3d_ioctl() : You got a muxtex lock !!\n");
		break;

	case S3C_3D_SFR_UNLOCK:
		mutex_lock_processID = 0;
		mutex_unlock(&mem_sfr_lock);
		DEBUG("s3c_g3d_ioctl() : The muxtex unlock called !!\n");
		break;

	case S3C_3D_MEM_ALLOC_SHARE:		
		mutex_lock(&mem_alloc_share_lock);
		if(copy_from_user(&param, (struct s3c_3d_mem_alloc *)arg, sizeof(struct s3c_3d_mem_alloc))){
			mutex_unlock(&mem_alloc_share_lock);
			return -EFAULT;
		}
		flag = MEM_ALLOC_SHARE;

		physical_address = param.phy_addr;
		DEBUG("param.phy_addr = %08x\n", physical_address);

		param.vir_addr = do_mmap(file, 0, param.size, PROT_READ|PROT_WRITE, MAP_SHARED, 0);
		DEBUG("param.vir_addr = %08x\n", param.vir_addr);

		if(param.vir_addr == -EINVAL) {
			printk("S3C_3D_MEM_ALLOC_SHARE FAILED\n");
			flag = 0;
			mutex_unlock(&mem_alloc_share_lock);
			return -EFAULT;
		}

		DEBUG("MALLOC_SHARE : param.phy_addr = 0x%X \t size = %d \t param.vir_addr = 0x%X\n", param.phy_addr, param.size, param.vir_addr);

		if(copy_to_user((struct s3c_3d_mem_alloc *)arg, &param, sizeof(struct s3c_3d_mem_alloc))){
			flag = 0;
			mutex_unlock(&mem_alloc_share_lock);
			return -EFAULT;		
		}

		flag = 0;
		
		mutex_unlock(&mem_alloc_share_lock);
		
		break;

	case S3C_3D_MEM_SHARE_FREE:	
		mutex_lock(&mem_share_free_lock);
		if(copy_from_user(&param, (struct s3c_3d_mem_alloc *)arg, sizeof(struct s3c_3d_mem_alloc))){
			mutex_unlock(&mem_share_free_lock);
			return -EFAULT;		
		}

		DEBUG("MEM_SHARE_FREE : param.phy_addr = 0x%X \t size = %d \t param.vir_addr = 0x%X\n", param.phy_addr, param.size, param.vir_addr);

		if (do_munmap(mm, param.vir_addr, param.size) < 0) {
			printk("do_munmap() failed - MEM_SHARE_FREE!!\n");
			mutex_unlock(&mem_share_free_lock);
			return -EINVAL;
		}

		param.vir_addr = 0;
		DEBUG("do_munmap() succeed !! - MEM_SHARE_FREE\n");

		if(copy_to_user((struct s3c_3d_mem_alloc *)arg, &param, sizeof(struct s3c_3d_mem_alloc))){
			mutex_unlock(&mem_share_free_lock);
			return -EFAULT;		
		}

		mutex_unlock(&mem_share_free_lock);
		
		break;

		case S3C_3D_CACHE_INVALID:
		if(copy_from_user(&param, (struct s3c_3d_mem_alloc *)arg, sizeof(struct s3c_3d_mem_alloc))){
			printk("ERR: Invalid Cache Error\n");
			return -EFAULT;
		}
		dmac_inv_range((unsigned int)param.vir_addr, (unsigned int)param.vir_addr+param.size);
		break;
		
		case S3C_3D_CACHE_CLEAN:		
			if(copy_from_user(&param, (struct s3c_3d_mem_alloc *)arg, sizeof(struct s3c_3d_mem_alloc))){
				printk("ERR: Invalid Cache Error\n");		
				return -EFAULT;
		}		
		dmac_clean_range((unsigned int)param.vir_addr, (unsigned int)param.vir_addr+param.size);
		break;

	default:
		DEBUG("s3c_g3d_ioctl() : default !!\n");
		return -EINVAL;
	}
	
	return 0;
}

int s3c_g3d_mmap(struct file* filp, struct vm_area_struct *vma)
{
	unsigned long pageFrameNo, size, virt_addr, phys_addr;

	size = vma->vm_end - vma->vm_start;

	switch (flag) { 
	case MEM_ALLOC :
		virt_addr = kmalloc(size, GFP_KERNEL);

		if (virt_addr == NULL) {
			printk("kmalloc() failed !\n");
			return -EINVAL;
		}
		DEBUG("MMAP_KMALLOC : virt addr = 0x%p, size = %d\n", virt_addr, size);
		phys_addr = virt_to_phys(virt_addr);
		physical_address = (unsigned int)phys_addr;

		//DEBUG("MMAP_KMALLOC : phys addr = 0x%p\n", phys_addr);
		pageFrameNo = __phys_to_pfn(phys_addr);
		//DEBUG("MMAP_KMALLOC : PFN = 0x%x\n", pageFrameNo);
		break;
		
	case MEM_ALLOC_SHARE :
		DEBUG("MMAP_KMALLOC_SHARE : phys addr = 0x%p\n", physical_address);
		
		// page frame number of the address for the physical_address to be shared.
		pageFrameNo = __phys_to_pfn(physical_address);
		//DEBUG("MMAP_KMALLOC_SHARE: PFN = 0x%x\n", pageFrameNo);
		DEBUG("MMAP_KMALLOC_SHARE : vma->end = 0x%p, vma->start = 0x%p, size = %d\n", vma->vm_end, vma->vm_start, size);
		break;
		
	default :
		// page frame number of the address for a source G2D_SFR_SIZE to be stored at.
		pageFrameNo = __phys_to_pfn(S3C64XX_PA_G3D);
		DEBUG("MMAP : vma->end = 0x%p, vma->start = 0x%p, size = %d\n", vma->vm_end, vma->vm_start, size);

		if(size > S3C64XX_SZ_G3D) {
			printk("The size of G3D_SFR_SIZE mapping is too big!\n");
			return -EINVAL;
		}
		break;
	}
	
	// sy82.yoon (2009.04.21) - use cache for improving speed
	//vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	if ((vma->vm_flags & VM_WRITE) && !(vma->vm_flags & VM_SHARED)) {
		printk("s3c_g3d_mmap() : Writable G3D_SFR_SIZE mapping must be shared !\n");
		return -EINVAL;
	}

	if (remap_pfn_range(vma, vma->vm_start, pageFrameNo, size, vma->vm_page_prot)) {
		printk("s3c_g3d_mmap() : remap_pfn_range() failed !\n");
		return -EINVAL;
	}

	return 0;
}

void grabageCollect(int* newid)
{
    struct mm_struct *mm = current->mm;
    unsigned long virt_addr;
    Memalloc_desc *temp, *prev=pHeadChunk;
    //RETAILMSG(1,(TEXT("GarbageCollect!!\n")));
    mutex_lock(&mem_free_lock);
    
    for(temp=pHeadChunk->next; temp != NULL ; temp=temp->next)
    {
        if(temp->newid == newid)
        {
            //printk("garbage size=%d\n", temp->size);   

    		if (do_munmap(mm, temp->vir_addr, temp->size) < 0) {
    			printk("do_munmap() failed !!\n");
    		}
    		virt_addr = phys_to_virt(temp->phy_addr);
    		//printk("KERNEL : virt_addr = 0x%X\n", virt_addr);
    
    		kfree(virt_addr);
                 
            prev->next = temp->next;
            if(temp->next != NULL) temp->next->prev = prev;
            vfree(temp);
        	temp = prev;
        }
        else 
        {
            prev = temp;
        }
    }
    LastChunk = prev;
  
    //RETAILMSG(1,(TEXT("GarbageCollect end!!\n")));
    mutex_unlock(&mem_free_lock);  
}

static struct file_operations s3c_g3d_fops = {
	.owner 	= THIS_MODULE,
	.ioctl 	= s3c_g3d_ioctl,
	.open 	= s3c_g3d_open,
	.release = s3c_g3d_release,
	.read 	= s3c_g3d_read,
	.write 	= s3c_g3d_write,
	.mmap	= s3c_g3d_mmap,
};


static struct miscdevice s3c_g3d_dev = {
	.minor		= G3D_MINOR,
	.name		= "s3c-g3d",
	.fops		= &s3c_g3d_fops,
};

static int s3c_g3d_remove(struct platform_device *dev)
{
	//clk_disable(g3d_clock);

	free_irq(s3c_g3d_irq, NULL);

	if (s3c_g3d_mem != NULL) {
		pr_debug("s3c_g3d: releasing s3c_post_mem\n");
		iounmap(s3c_g3d_base);
		release_resource(s3c_g3d_mem);
		kfree(s3c_g3d_mem);
	}

	misc_deregister(&s3c_g3d_dev);
	return 0;
}

int s3c_g3d_probe(struct platform_device *pdev)
{

	struct resource *res;

	int ret;
	int i;

	DEBUG("s3c_g3d probe() called\n");

	s3c_set_normal_cfg(S3C64XX_DOMAIN_G, S3C64XX_ACTIVE_MODE, S3C64XX_3D);
        if(s3c_wait_blk_pwr_ready(S3C64XX_BLK_G)) {
             return -1;
	}

	s3c_g3d_irq = platform_get_irq(pdev, 0);


	if(s3c_g3d_irq <= 0) {
		printk(KERN_ERR PFX "failed to get irq resouce\n");
		return -ENOENT;
	}

	/* get the memory region for the post processor driver */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(res == NULL) {
		printk(KERN_ERR PFX "failed to get memory region resouce\n");
		return -ENOENT;
	}

	s3c_g3d_base_physical = (unsigned int)res->start;

	s3c_g3d_mem = request_mem_region(res->start, res->end-res->start+1, pdev->name);
	if(s3c_g3d_mem == NULL) {
		printk(KERN_ERR PFX "failed to reserve memory region\n");
		return -ENOENT;
	}


	s3c_g3d_base = ioremap(s3c_g3d_mem->start, s3c_g3d_mem->end - res->start + 1);
	if(s3c_g3d_base == NULL) {
		printk(KERN_ERR PFX "failed ioremap\n");
		return -ENOENT;
	}

	g3d_clock = clk_get(&pdev->dev, "post");
	if(g3d_clock == NULL) {
		printk(KERN_ERR PFX "failed to find post clock source\n");
		return -ENOENT;
	}

	clk_enable(g3d_clock);

	h_clk = clk_get(&pdev->dev, "hclk");
	if(h_clk == NULL) {
		printk(KERN_ERR PFX "failed to find h_clk clock source\n");
		return -ENOENT;
	}

	init_waitqueue_head(&waitq);

	ret = misc_register(&s3c_g3d_dev);
	if (ret) {
		printk (KERN_ERR "cannot register miscdev on minor=%d (%d)\n",
				G3D_MINOR, ret);
		return ret;
	}

	// device reset
	__raw_writel(1,s3c_g3d_base+FGGB_RST);
	for(i=0;i<1000;i++);
	__raw_writel(0,s3c_g3d_base+FGGB_RST);
	for(i=0;i<1000;i++);

	ret = request_irq(s3c_g3d_irq, s3c_g3d_isr, IRQF_DISABLED,
			pdev->name, NULL);
	if (ret) {
		printk("request_irq(S3D) failed.\n");
		return ret;
	}

	printk("s3c_g3d version : 0x%x\n",__raw_readl(s3c_g3d_base + FGGB_VERSION));
	s3c_set_normal_cfg(S3C64XX_DOMAIN_G, S3C64XX_LP_MODE, S3C64XX_3D);

	/* check to see if everything is setup correctly */
	return 0;
}

static int s3c_g3d_suspend(struct platform_device *dev, pm_message_t state)
{
	//clk_disable(g3d_clock);
	return 0;
}
static int s3c_g3d_resume(struct platform_device *pdev)
{
	//clk_enable(g3d_clock);
	s3c_set_normal_cfg(S3C64XX_DOMAIN_G, S3C64XX_LP_MODE, S3C64XX_3D);	
	return 0;
}
static struct platform_driver s3c_g3d_driver = {
	.probe          = s3c_g3d_probe,
	.remove         = s3c_g3d_remove,
	.suspend        = s3c_g3d_suspend,
	.resume         = s3c_g3d_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "s3c-g3d",
	},
};

static char banner[] __initdata = KERN_INFO "S3C G3D Driver, (c) 2007 Samsung Electronics\n";

int __init  s3c_g3d_init(void)
{

	printk(banner);
	if(platform_driver_register(&s3c_g3d_driver)!=0)
	{
		printk("platform device register Failed \n");
		return -1;
	}

	return 0;
}

void  s3c_g3d_exit(void)
{
	platform_driver_unregister(&s3c_g3d_driver);
	printk("S3C G3D module exit\n");
}

module_init(s3c_g3d_init);
module_exit(s3c_g3d_exit);

MODULE_AUTHOR("lee@samsung.com");
MODULE_DESCRIPTION("S3C G3D Device Driver");
MODULE_LICENSE("GPL");


