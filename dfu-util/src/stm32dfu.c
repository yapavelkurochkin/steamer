/* This is supposed to be a "real" DFU implementation, just as specified in the
 * USB DFU 1.0 Spec.  Not overloaded like the Atmel one...
 *
 * (C) 2007-2008 by Harald Welte <laforge@gnumonks.org>
 * (C) 2010 Modified by Nikolay Zamotaev <fhunter@metrotek.spb.ru>
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <usb.h>

#include "config.h"
#include "dfu.h"
#include "main.h"
#include "usb_dfu.h"
#include "param.h"

/* ugly hack for Win32 */
#ifndef O_BINARY
#define O_BINARY 0
#endif

extern short transaction;

int in_array( unsigned char state, unsigned char *states,
              int states_count )
{
  int i;
  for( i = 0; i < states_count; i++ ) {
    if( state == states[i] ) {
      return i;
    }
  }
  return -1;
};

int stm32dfu_wait_for_state( struct usb_dev_handle *usb_handle,
                             int interface, unsigned int delay,
                             unsigned char *states, int states_count,
                             char *error_message )
{
  int ret;
  struct dfu_status dst;
  do {
    ret = dfu_get_status( usb_handle, interface, &dst );
    if( ret < 0 ) {
      fprintf( stderr, "%s\n", error_message );
      return -1;
    }
    //printf("dst.bState= %s dst.bStatus= %s\n",dfu_state_to_string(dst.bState),dfu_status_to_string(dst.bStatus));
    usleep( delay );
  } while( in_array( dst.bState, states, states_count ) < 0 );
  return ( dst.bState << 8 ) | ( dst.bStatus );
}

/* FIXME: should return cleanly, without requiring for second reading of status */
int stm32dfu_set_address( struct usb_dev_handle *usb_handle,
                          int interface, unsigned int address )
{
  int rc;
  char buffer[5];
  struct dfu_status status;
  buffer[0] = 0x21;             //Set address pointer command
  buffer[1] = address & 0xffUL;
  buffer[2] = ( address & 0x00ff00UL ) >> 8;
  buffer[3] = ( address & 0xff0000UL ) >> 16;
  buffer[4] = ( address & 0xff000000UL ) >> 24;
  transaction = 0;              //wValue=0;
  rc = dfu_download( usb_handle, interface, 5, buffer );
  if( rc < 0 ) {
    fprintf( stderr, "Error setting address\n" );
    return rc;
  }
  if( ( rc = dfu_get_status( usb_handle, interface, &status ) ) < 0 ) {
    fprintf( stderr, "Error getting status\n" );
    return rc;
  };
//  printf("dst.bState= %s dst.bStatus= %s\n",dfu_state_to_string(status.bState),dfu_status_to_string(status.bStatus));
  if( status.bState != STATE_DFU_DOWNLOAD_BUSY ) {
    fprintf( stderr, "Error performing set address\n" );
    return -1;
  };
  if( stm32dfu_wait_for_state
      ( usb_handle, interface, 5000, ( unsigned char[2] ) {
        DFU_STATE_dfuDNLOAD_IDLE, 0,}, 1,
        "Error during set_address" ) < 0 ) {
	  return -1;
  }
  /*
   * returns 0 on succesfull set
   * or error (<0)
   */
  return rc;
}

/* FIXME: should return cleanly, without requiring for second reading of status */
int stm32dfu_erase_all( struct usb_dev_handle *usb_handle,
                        int interface )
{
  int rc;
  char buffer[1];
  struct dfu_status status;
  transaction = 0;              //wValue=0;
  buffer[0] = 0x41;
  rc = dfu_download( usb_handle, interface, 1, buffer );
  if( rc < 0 ) {
    fprintf( stderr, "Error performing bulk erase\n" );
    return rc;
  };
  if( ( rc = dfu_get_status( usb_handle, interface, &status ) ) < 0 ) {
    fprintf( stderr, "Error getting status\n" );
    return rc;
  };
//  printf("dst.bState= %s dst.bStatus= %s\n",dfu_state_to_string(status.bState),dfu_status_to_string(status.bStatus));
  if( status.bState != STATE_DFU_DOWNLOAD_BUSY ) {
    fprintf( stderr, "Error performing bulk erase\n" );
    return -1;
  };
  if( stm32dfu_wait_for_state
      ( usb_handle, interface, 5000, ( unsigned char[2] ) {
        DFU_STATE_dfuDNLOAD_IDLE, 0,}, 1,
        "Error during download get_status" ) < 0 ) 
	  return -1;

  return rc;
}

#define PROGRESS_BAR_WIDTH 50

int stm32dfu_do_dnload( struct usb_dev_handle *usb_handle,
                        int interface, const parameters param)
/*const int xfer_size,
                        const char *fname, const unsigned int address,const int do_execute )*/
{
  int ret, fd, bytes_sent = 0;
  unsigned int bytes_per_hash, hashes = 0;
  char *buf = malloc( param.transfer_size );
  struct stat st;
  struct dfu_status dst;

  if( !buf )
    return -ENOMEM;

  fd = open( param.filename, O_RDONLY | O_BINARY );
  if( fd < 0 ) {
    perror( param.filename );
    ret = fd;
    free( buf );
    return ret;
  }

  ret = fstat( fd, &st );
  if( ret < 0 ) {
    perror( param.filename );
    close( fd );
    free( buf );
    return ret;
  }

  if( st.st_size <= 0 /* + DFU_HDR */  ) {
    fprintf( stderr, "File seems a bit too small...\n" );
    ret = -EINVAL;
    close( fd );
    free( buf );
    return ret;
  }

  bytes_per_hash = st.st_size / PROGRESS_BAR_WIDTH;
  if( bytes_per_hash == 0 )
    bytes_per_hash = 1;
  printf( "bytes_per_hash=%u\n", bytes_per_hash );
  printf( "Starting download: [" );
  fflush( stdout );
  if(param.do_execute & DO_ERASE){
    if( stm32dfu_erase_all( usb_handle, interface ) < 0 ) {
      ret = -1;
      close( fd );
      free( buf );
      return ret;
    };
  };
  if( stm32dfu_set_address( usb_handle, interface, param.offset ) < 0 ) {
    ret = -1;
    close( fd );
    free( buf );
    return ret;
  };
  transaction = 2;
  while( bytes_sent < st.st_size ) {
    int hashes_todo;

    ret = read( fd, buf, param.transfer_size );
    if( ret < 0 ) {
      perror( param.filename );
      close( fd );
      free( buf );
      return ret;
    }
    ret =
      dfu_download( usb_handle, interface, ret, ret ? buf : NULL );
    if( ret < 0 ) {
      fprintf( stderr, "Error during download\n" );
      close( fd );
      free( buf );
      return ret;
    }
    bytes_sent += ret;

    ret =
      stm32dfu_wait_for_state( usb_handle, interface, 5000,
                               ( unsigned char[2] ) {
                               DFU_STATE_dfuDNLOAD_IDLE, 0,}, 1,
                               "Error during set_address" );
    if( ret < 0 ) {
      close( fd );
      free( buf );
      return ret;
    };
    if( ( ret & 0xffUL ) != DFU_STATUS_OK ) {
      printf( " failed!\n" );
      printf( "state(%u) = %s, status(%u) = %s\n",
              ( ret & 0xff00U ) >> 8,
              dfu_state_to_string( ( ret & 0xff00U ) >> 8 ),
              ret & 0xffU, dfu_status_to_string( ret & 0xffU ) );
      ret = -1;
      close( fd );
      free( buf );
      return ret;
    }

    hashes_todo = ( bytes_sent / bytes_per_hash ) - hashes;
    hashes += hashes_todo;
    while( hashes_todo-- )
      putchar( '#' );
    fflush( stdout );
  }

  /* send one zero sized download request to signalize end */
  //этот код сбрасывает процессор после заливки данных - это нужно сделать опциональным
  if(param.do_execute & DO_EXECUTE){
      ret = dfu_download(usb_handle, interface, 0, NULL);
      if (ret >= 0)
              ret = bytes_sent;
  };
  printf( "] finished!\n" );
  fflush( stdout );

  do {
    /* Transition to MANIFEST_SYNC state */
    ret = dfu_get_status( usb_handle, interface, &dst );
    if( ret < 0 ) {
      fprintf( stderr, "unable to read DFU status\n" );
      close( fd );
      free( buf );
      return ret;
    }
   
//    printf( "state(%u) = %s, status(%u) = %s\n", dst.bState,
//        dfu_state_to_string( dst.bState ), dst.bStatus,
//        dfu_status_to_string( dst.bStatus ) );

    /* FIXME: deal correctly with ManifestationTolerant=0 / WillDetach bits */
    switch ( dst.bState ) {
      case DFU_STATE_dfuMANIFEST_SYNC:
      case DFU_STATE_dfuMANIFEST:
        /* some devices (e.g. TAS1020b) need some time before we
         * can obtain the status */
        sleep( 1 );
        continue;
        break;
      case DFU_STATE_dfuIDLE:
        break;
      case DFU_STATE_dfuMANIFEST_WAIT_RST:
        /* In this case device cannot communicate via USB after reprogramming,
           so it must be reset */
        /* In a matter of fact, host can't receive this status from device, 
           because device can't transmit anything via USB in this case :) */
        break;
    }
  } while( dst.bState == DFU_STATE_dfuIDLE );

  printf( "Done!\n" );
  close( fd );
  free( buf );

  return ret;
}

/*void stm32dfu_init(  )
{
  dfu_debug( debug );
  dfu_init( 5000 );
}*/
