/* SubWCRev version file for Bootloader InkSystem board.
 После запуска SubWCRev будет создан файл version.h, который используется в проекте.
 Данный файл (version.tmpl, version.h) никому не давать, т.к. здесь
 содержится индивидуальный 96-и битный ключ для дешифрования.
 Source code state: Not modified
 Date:              2016/02/26 15:24:21
 Range:             92
 Mixed:             Not mixed
 URL:               file:///D:/Projects/Ink_System/SVN/Soft/Bootloader_STM32/trunk
*/

#define BOOT_FW_COMPATIBILITY 0x0200
#define FIRMWARE_MAJOR      ((uint32_t)2)
#define FIRMWARE_MINOR      ((uint32_t)0)
#define FIRMWARE_BUILD      ((uint32_t)0)
#define FIRMWARE_REVISION   ((uint32_t)0)
#define HARDWARE_MAJOR  ((uint32_t)1)
#define HARDWARE_MINOR  ((uint32_t)0)

#define KEY0 ((uint32_t)0x0D392708)
#define KEY1 ((uint32_t)0x7EC9A493)
#define KEY2 ((uint32_t)0x82F99581)


#pragma location="flash_SerialNumber"
__root const uint32_t  SerialNumber = 0xFFFFFFFF;
#pragma location="flash_HardVersion"
__root const uint32_t  bootHardware = (HARDWARE_MAJOR<<16)|(HARDWARE_MINOR); 
#pragma location="flash_SoftVersion"
__root const uint32_t  bootFirmware = (FIRMWARE_MAJOR<<8)|(FIRMWARE_MINOR);

#ifdef	NDEBUG
#if 0
#warning "Uncomment next line"
#error Source is modified. SVN commit projects and repeat build projects
#endif
#endif

// End of file