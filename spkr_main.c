/*
Integrantes del grupo:
    - Luis Bustamante Martin-Ibañez
    - Ignacio Hidalgo Power
*/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/kfifo.h>
#include <linux/ioctl.h>
#include <asm/uaccess.h>

#define FIFO_SIZE 1

MODULE_LICENSE("GPL");

// Prototipos de las funciones
extern void set_spkr_frequency(unsigned int frequency);
extern void spkr_on(void);
extern void spkr_off(void);
static int spkr_open(struct inode *i, struct file *f);
static int spkr_release(struct inode *i, struct file *f);
static ssize_t spkr_write(struct file *f, const char __user *buf, size_t count, loff_t *off);
static ssize_t spkr_write_unbuffered(struct file *f, const char __user *buf, size_t count, loff_t *off);
static void spkr_timer_callback(struct timer_list *t);

// Variables globales
static int write_count = 0; // Contador de escrituras
static dev_t dev = 0; // esto es para que el SO asigne dinamicamente el major number
static int minor = 0; // minor number

static struct cdev c_dev; // Estructura de datos para el dispositivo
static struct class *cl; // Clase del dispositivo
static struct mutex mutex; // Mutex para proteger el acceso al dispositivo

// Estrucutra de datos para las funciones de acceso al dispositivo
static struct file_operations fops =
{
    .owner = THIS_MODULE,
    .open = spkr_open,
    .release = spkr_release,
    .write = spkr_write,
};

// Estructura de informacion del dispositivo
static struct info_dispo {
    wait_queue_head_t wait_queue;
    struct timer_list timer;
    spinlock_t lock; // Spinlock for protecting against software interrupts
}info_dispo;

// Parametros de entrada
module_param(minor, int, S_IRUGO);

// Funciones de acceso al dispositivo
static int spkr_open(struct inode *i, struct file *f)
{
    mutex_lock(&mutex);

    if(f->f_mode & FMODE_WRITE && write_count != 0) {
        printk(KERN_ERR "spkr: Dispositivo ocupado");
        mutex_unlock(&mutex);
        return -EBUSY;
    }
    
    if(f->f_mode & FMODE_WRITE) {
        write_count++;
    }
    
    printk(KERN_INFO "spkr: Dispositivo abierto");

    mutex_unlock(&mutex);

    return 0;
}

static int spkr_release(struct inode *i, struct file *f)
{
    mutex_lock(&mutex);

    if(f->f_mode & FMODE_WRITE) {
        write_count--;
    }

    printk(KERN_INFO "spkr: Dispositivo liberado\n");

    mutex_unlock(&mutex);

    return 0;
}

static ssize_t spkr_write(struct file *f, const char __user *buf, size_t count, loff_t *off)
{
    ssize_t ret;
    printk(KERN_INFO "spkr: Escribiendo en el dispositivo\n");
    ret = spkr_write_unbuffered(f, buf, count, off);
    return count;
}

static int __init spkr_init(void)
{
    int ret;
    struct device *dev_ret;

    // Registro major y minor number
    if ((ret = alloc_chrdev_region(&dev, minor, 1, "spkr")) < 0)
    {
        return ret;
    }

    printk(KERN_INFO "spkr: Major number: %d, Minor number: %d\n", MAJOR(dev), MINOR(dev));

    cdev_init(&c_dev, &fops);

    // Damos de alta el dispositivo en el nucleo
    if ((ret = cdev_add(&c_dev, dev, 1)) < 0)
    {
        printk(KERN_ALERT "Error al registrar el dispositivo\n");
        return ret;
    }
    printk(KERN_INFO "spkr: Dispositivo dado de alta en el nucleo correctamente");
   
   // Damos de alta el dispositivo en el sistema de archivos
    if (IS_ERR(cl = class_create("speaker")))
    {
        cdev_del(&c_dev);
        unregister_chrdev_region(dev, 1);
        printk(KERN_ALERT "Error al registrar el dispositivo en el sistema de archivos\n");
        return PTR_ERR(cl);
    }

    // Creamos el dispositivo
    if (IS_ERR(dev_ret = device_create(cl, NULL, dev, NULL, "intspkr")))
    {
        class_destroy(cl);
        cdev_del(&c_dev);
        unregister_chrdev_region(dev, 1);
        return PTR_ERR(dev_ret);
    }
    printk(KERN_INFO "spkr: Dispositivo dado de alta en el sistema de archivos correctamente");

    // Iniciar mutex
    mutex_init(&mutex);

    init_waitqueue_head(&info_dispo.wait_queue);

    return 0;
}

static void __exit spkr_exit(void)
{
    device_destroy(cl, dev);
    class_destroy(cl);
    cdev_del(&c_dev);
    unregister_chrdev_region(dev, 1);
}

// Fin timer sin buffer interno
static void timer_callback(struct timer_list *timer) {

    spin_lock_bh(&info_dispo.lock);

    wake_up_interruptible(&info_dispo.wait_queue);

    spin_unlock_bh(&info_dispo.lock);
}

// Write function
static ssize_t spkr_write_unbuffered(struct file *f, const char __user *buf, size_t count, loff_t *off)
{
    printk(KERN_INFO "spkr: Escribiendo en el dispositivo sin buffer interno\n");
    uint16_t duracion;
    uint16_t frecuencia;

    if(count % 4 != 0) {
        printk(KERN_ERR "spkr: Error en el formato de los datos, se debe cumplir bs*count multiplo de 4\n");
        return -EINVAL;
    }

    // Cada 4 bytes leidos contienen 2 bytes con la duracion del tono y 2 bytes con la frecuencia
    // Hay que leerlos de 2 en 2 hasta que se lean todos los bytes
    // Si la frecuencia es 0, se apaga el altavoz durante la duracion especificada
    for(int i = 0; i < count; i += 4) {
        // Leer duracion
        if(copy_from_user(&duracion, buf + i, 2)) {
            printk(KERN_ERR "spkr: Error al leer la duracion\n");
            return -EFAULT;
        }
        printk(KERN_INFO "spkr: Duracion: %d\n", duracion);
        // Leer frecuencia
        if(copy_from_user(&frecuencia, buf + i + 2, 2)) {
            printk(KERN_ERR "spkr: Error al leer la frecuencia\n");
            return -EFAULT;
        }
        printk(KERN_INFO "spkr: Frecuencia: %d\n", frecuencia);

        // Si la frecuencia no es 0, se enciende el altavoz con la frecuencia especificada
        if(frecuencia) {
            set_spkr_frequency(frecuencia);
            spkr_on();
        }
        
    }

    return count;
}

module_exit(spkr_exit);
module_init(spkr_init);