#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/i8253.h>
#include <asm/io.h>
#include <linux/spinlock.h>

raw_spinlock_t i8253_lock;

void set_spkr_frequency(unsigned int frequency) 
{
	unsigned int result;
	unsigned long flags;

	// Calculo del result
	if (frequency == 0) 
	{
		result = 0;
	}
	else result = PIT_TICK_RATE / frequency;

	// Se bloquea el acceso a la i8253
	raw_spin_lock_irqsave(&i8253_lock, flags);

	// Se especifica el modo de operacion en el puerto 0x43
	outb(0xB0, 0x43);

	// Se especifica el resultado en el puerto 0x42
	outb(result & 0xFF, 0x42); 
   	outb(result >> 8, 0x42); 

	// Se desbloquea el acceso a la i8253
	raw_spin_unlock_irqrestore(&i8253_lock, flags);

	printk(KERN_INFO "spkr: Set frequency: %d\n", frequency);
}

void spkr_on(void) 
{
	unsigned long flags;

	// Se bloquea el acceso a la i8253
	raw_spin_lock_irqsave(&i8253_lock, flags);

	// Se realiza un OR con 0x03 para encender el altavoz
	outb(inb(0x61) | 0x03, 0x61);
 
	// Se desbloquea el acceso a la i8253
	raw_spin_unlock_irqrestore(&i8253_lock, flags);

	printk(KERN_INFO "spkr: ON\n");
}

void spkr_off(void) 
{
	unsigned long flags;

	// Se bloquea el acceso a la i8253
	raw_spin_lock_irqsave(&i8253_lock, flags);

	// Se realiza un AND con 0xFE para apagar el altavoz
	outb(inb(0x61) & 0xFE, 0x61);

	// Se desbloquea el acceso a la i8253
	raw_spin_unlock_irqrestore(&i8253_lock, flags);

	printk(KERN_INFO "spkr: OFF\n");
}