#ifndef IAR_TO_GCC_H
#define IAR_TO_GCC_H

#define __no_init __attribute__ ((section (".noinit")))
#define __packed __attribute__((__aligned__(8))) /*__attribute__ ((packed)) */
#define __weak __attribute__ ((weak))
#define __ro_placement __attribute__((section(".IP_Code")))

#endif /* IAR_TO_GCC_H */
