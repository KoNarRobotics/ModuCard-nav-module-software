

# RAM allocatio

The stm cube generates all ram table however inputs all data to the default  one ,DTCMRAM whitch is too small sine its only have 128KB for FREERTOS HEAP.

so what We will have to do to define attributes to tell 
the compiler to put the data on the specific ram regions

* In linker file We add inside  SECTIONS {} definitions for each ram region
```ld
.dtcmram (NOLOAD) : 
  {
      *(.dtcmram_section) 
  } >DTCMRAM

  .ramaxis (NOLOAD) : 
  {
      *(.ramaxis_section) 
  } >RAM

  .ram_d2 (NOLOAD) : 
  {
      *(.ram_d2_section)
  } >RAM_D2

  .ram_d3 (NOLOAD) : 
  {
      *(.ram_d3_section)
  } >RAM_D3
```
* Now we will use attributes like `__attribute__((section(".ramaxis_section")))` to tell the compiler to put the data on the specific ram region, for example:
```c
__attribute__((section(".ramaxis_section"))) uint8_t some_data[1024];
```
* We will use the default DTCMRAM for the locak and global variables.
so we don't have to do anything for them.
* However we will have to tell the compiler to put the freertos heap on the specific AXI RAM  region, by modifying 'Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c'
```c
/* in line 64 we modify this */
static uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];
/* to this */
 __attribute__((section(".ramaxis_section"))) static uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];
```
