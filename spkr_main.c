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
static ssize_t spkr_write_buffered(struct file *f, const char __user *buf, size_t count, loff_t *off);
int spkr_program_sound(u16 duracion, u16 frecuencia);

// Variables globales
static int write_count = 0; // Contador de escrituras
static dev_t dev = 0; // esto es para que el SO asigne dinamicamente el major number
static int speaker_on = 0; // Flag para saber si el altavoz esta encendido

static struct cdev c_dev; // Estructura de datos para el dispositivo
static struct class *cl; // Clase del dispositivo
static struct mutex mutex; // Mutex para proteger el acceso al dispositivo

// Estrucutra de datos para las funciones de acceso al dispositivo
static struct file_operations fops = {
    .owner = THIS_MODULE,
    .open = spkr_open,
    .release = spkr_release,
    .write = spkr_write,
};

// Estructura de informacion del dispositivo
static struct info_dispo {
    wait_queue_head_t wait_queue;
    struct timer_list timer;
    char buffer[4];  // Buffer para datos entrantes
    size_t buffer_index;  // Numero de bytes actualmente en el buffer
    spinlock_t lock; // Spinlock for protecting against software interrupts
    struct kfifo kfifo; // para la escritura con buffer interno
}info_dispo;

// Parametros de entrada
static int minor = 0; // minor number
module_param(minor, int, S_IRUGO);
static int buffer_length = 0; // longitud del buffer
module_param(buffer_length, int, S_IRUGO);


// Funciones de acceso al dispositivo
static int spkr_open(struct inode *i, struct file *f)
{
    mutex_lock(&mutex);

    if(f->f_mode & FMODE_WRITE && write_count != 0) 
    {
        printk(KERN_ERR "spkr: Dispositivo ocupado");
        mutex_unlock(&mutex);
        return -EBUSY;
    }
    
    if(f->f_mode & FMODE_WRITE) 
    {
        write_count++;
    }
    printk(KERN_INFO "spkr: Dispositivo abierto");
    mutex_unlock(&mutex);

    return 0;
}

static int spkr_release(struct inode *i, struct file *f)
{
    mutex_lock(&mutex);

    if(f->f_mode & FMODE_WRITE) 
    {
        write_count--;
    } 

    printk(KERN_INFO "spkr: Dispositivo liberado\n");
    mutex_unlock(&mutex);

    return 0;
}

static ssize_t spkr_write(struct file *f, const char __user *buf, size_t count, loff_t *off)
{
    ssize_t ret;
    printk(KERN_INFO "spkr: llamada a spkr_write\n");
    
    if(buffer_length == 0) ret = spkr_write_unbuffered(f, buf, count, off);
    else ret = spkr_write_buffered(f, buf, count, off);

    return ret;
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

    // Se inicia el mutex de las operaciones de acceso al dispositivo
    mutex_init(&mutex);
    
    // Se inicializa la estructura de informacion del dispositivo
    init_waitqueue_head(&info_dispo.wait_queue);
    info_dispo.buffer_index = 0;
    spin_lock_init(&info_dispo.lock);
    if(buffer_length != 0) 
    {
        if(kfifo_alloc(&info_dispo.kfifo, buffer_length, GFP_KERNEL)) 
        {
            printk(KERN_ERR "spkr: Error al inicializar la kfifo\n");
            return -ENOMEM;
        }
    }

    return 0;
}

static void __exit spkr_exit(void)
{
    printk(KERN_INFO "spkr: Desinstalando modulo\n");
    if(buffer_length != 0) kfifo_free(&info_dispo.kfifo);
    device_destroy(cl, dev);
    class_destroy(cl);
    cdev_del(&c_dev);
    unregister_chrdev_region(dev, 1);
}

// Funcion de escritura sin buffer interno
static ssize_t spkr_write_unbuffered(struct file *f, const char __user *buf, size_t count, loff_t *off)
{
    printk(KERN_INFO "spkr: Escribiendo en el dispositivo sin buffer interno\n");
    u16 *sound_as_u16;

    // Cada 4 bytes leidos contienen 2 bytes con la duracion del sonido y 2 bytes con la frecuencia
    // Si la frecuencia es 0, se apaga el altavoz durante la duracion especificada
    int i;
    for(i = 0; i < count; i ++) 
    {
        if(copy_from_user(&info_dispo.buffer[info_dispo.buffer_index++], &buf[i], 1)) 
        {
            printk(KERN_ERR "spkr: Error al leer el byte\n");
            return -EFAULT;
        }
        // Leer duracion
        if(info_dispo.buffer_index == 4)
        {
            sound_as_u16 = (u16 *)info_dispo.buffer;
            spkr_program_sound(sound_as_u16[0], sound_as_u16[1]);

            info_dispo.buffer_index = 0;
        }    
    }
    // Si el altavoz se ha quedado encendido, apagarlo
    if(speaker_on) spkr_off();

    printk(KERN_INFO "spkr: Escritura finalizada, bytes leidos=%d\n", i);
    return count;
}

// Funcion de escritura con buffer interno
static ssize_t spkr_write_buffered(struct file *f, const char __user *buf, size_t count, loff_t *off)
{
    int ret;
    unsigned int copied;

    printk(KERN_INFO "spkr: Escribiendo en el dispositivo con buffer interno\n");
    printk(KERN_INFO "spkr: Buffer length: %d\n", buffer_length);
    printk(KERN_INFO "spkr: Count: %li\n", count);

    int iteraciones = 0;
    for(int i = 0; i < count; i += buffer_length) {
        unsigned int bytes_to_copy = min(buffer_length, (int)(count - i));
        printk(KERN_INFO "spkr: Copiando %d bytes a la kfifo\n", bytes_to_copy);

        if(kfifo_from_user(&info_dispo.kfifo, buf + i, bytes_to_copy, &copied)) 
        {
            printk(KERN_ERR "spkr: Error al copiar los datos a la kfifo\n");
            return -EFAULT;
        }
        printk(KERN_INFO "spkr: Copiados %d bytes a la kfifo\n", copied);

        char sound[4]; // Buffer to hold one sound (4 bytes)
        u16 *sound_as_u16;

        // Recorremos el buffer de la kfifo y escribimos los sonidos
        while (!kfifo_is_empty(&info_dispo.kfifo)) {
            // Extract one sound from the kfifo
            ret = kfifo_out(&info_dispo.kfifo, sound, 4);
            if (ret != 4) {
                printk(KERN_ERR "spkr: Error al leer de la kfifo (kfifo_out)\n");
                break;
            }

            // Interpret the sound as an array of two u16 values
            sound_as_u16 = (u16 *)sound;

            // Write the sound (2 bytes for duration and 2 for frequency)
            spkr_program_sound(sound_as_u16[0], sound_as_u16[1]);
        }
        iteraciones++;
    }
    if(speaker_on) spkr_off();

    printk(KERN_INFO "spkr: Escritura finalizada con buffer, veces leidas=%d\n", iteraciones);

    return copied;
}

int spkr_program_sound(u16 duracion, u16 frecuencia) {
    // Si la frecuencia no es 0, se enciende el altavoz con la frecuencia especificada
    printk(KERN_INFO "spkr: Duracion: %d\n", duracion);
    printk(KERN_INFO "spkr: Frecuencia: %d\n", frecuencia);
    if(frecuencia && !speaker_on) 
    {
        speaker_on = 1;
        set_spkr_frequency(frecuencia);
        spkr_on();
    }
    else 
    {
        speaker_on = 0;
        spkr_off();
    }

    // Esperar duracion usando el timer de info_dispo y el callback
    info_dispo.timer.expires = jiffies + msecs_to_jiffies(duracion);
    timer_setup(&info_dispo.timer, spkr_timer_callback, 0);
    add_timer(&info_dispo.timer);

    if(wait_event_interruptible(info_dispo.wait_queue, timer_pending(&info_dispo.timer) == 0)) 
    {
        printk(KERN_INFO "spkr: wait condition en spkr_write_unbuffered\n");
        return -ERESTARTSYS;
    }

    return 0;
}

// Fin timer
static void spkr_timer_callback(struct timer_list *timer) 
{
    spin_lock_bh(&info_dispo.lock);
    wake_up_interruptible(&info_dispo.wait_queue);
    spin_unlock_bh(&info_dispo.lock);
}

module_exit(spkr_exit);
module_init(spkr_init);