#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/kfifo.h>
#include <linux/ioctl.h>
#include <asm/uaccess.h>

MODULE_LICENSE("GPL");

extern void set_spkr_frequency(unsigned int frequency);
extern void spkr_on(void);
extern void spkr_off(void);

static dev_t dev = 0; // This will hold the major number that the kernel gives us
static struct cdev c_dev; // This is the cdev structure
static struct class *cl; // This will be used later for sysfs

static int spkr_open(struct inode *i, struct file *f)
{
    // Implement the open operation here
    return 0;
}

static int spkr_release(struct inode *i, struct file *f)
{
    // Implement the release operation here
    return 0;
}

static ssize_t spkr_write(struct file *f, const char __user *buf, size_t len, loff_t *off)
{
    // Implement the write operation here
    return len;
}

static struct file_operations fops =
{
    .owner = THIS_MODULE,
    .open = spkr_open,
    .release = spkr_release,
    .write = spkr_write,
};

static int __init spkr_init(void)
{
    int ret;
    struct device *dev_ret;

    if ((ret = alloc_chrdev_region(&dev, 0, 1, "spkr")) < 0)
    {
        return ret;
    }

    cdev_init(&c_dev, &fops);

    if ((ret = cdev_add(&c_dev, dev, 1)) < 0)
    {
        return ret;
    }


    return 0;
}

static void __exit spkr_exit(void)
{
    device_destroy(cl, dev);
    class_destroy(cl);
    cdev_del(&c_dev);
    unregister_chrdev_region(dev, 1);
}

module_exit(spkr_exit);
module_init(spkr_init);