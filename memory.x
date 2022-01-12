/* Linker script for the STM32F103C8T6 */
/* Total memory 64K */
/* Program memory 60K */
/* Data memory 4K */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 60K
  RAM : ORIGIN = 0x20000000, LENGTH = 20K
}