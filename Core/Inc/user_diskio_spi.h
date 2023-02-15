/**
 ******************************************************************************
  * @file    user_diskio_spi.h
  * @brief   This file contains the common defines and functions prototypes for
  *          the user_diskio_spi driver implementation
  ******************************************************************************
  * Portions copyright (C) 2014, ChaN, all rights reserved.
  * Portions copyright (C) 2017, kiwih, all rights reserved.
  *
  * This software is a free software and there is NO WARRANTY.
  * No restriction on use. You can use, modify and redistribute it for
  * personal, non-profit or commercial products UNDER YOUR RESPONSIBILITY.
  * Redistributions of source code must retain the above copyright notice.
  *
  ******************************************************************************
  */

#ifndef _USER_DISKIO_SPI_H
#define _USER_DISKIO_SPI_H

#include "integer.h" //from FatFs middleware library
#include "diskio.h" //from FatFs middleware library
#include "ff_gen_drv.h" //from FatFs middleware library


#define SD_SPI_HANDLE hspi1
//we define these as inline because we don't want them to be actual function calls (they get "called" from the cubemx autogenerated user_diskio file)
//we define them as extern because they are defined in a separate .c file to user_diskio.c (which #includes this .h file)

extern DSTATUS USER_SPI_initialize (BYTE pdrv);
extern DSTATUS USER_SPI_status (BYTE pdrv);
extern DRESULT USER_SPI_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  extern DRESULT USER_SPI_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  extern DRESULT USER_SPI_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

#endif
