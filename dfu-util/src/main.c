/*
 * dfu-util
 *
 * (C) 2007-2008 by OpenMoko, Inc., (C) 2010-2011 STC Metrotek
 * Written by Harald Welte <laforge@openmoko.org>,
 * Modified by Nikolay Zamotaev <fhunter@metrotek.spb.ru>
 *
 * Based on existing code of dfu-programmer-0.4
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <unistd.h>
#include "main.h"

//flag indicating that dfu device is found
static unsigned char dfu_device_found = 0;

/* define a portable function for reading a 16bit little-endian word */
unsigned short get_int16_le( const void *p )
{
  const unsigned char *cp = p;

  return ( cp[0] ) | ( ( ( unsigned short ) cp[1] ) << 8 );
}

int debug;

static int _get_first_cb( struct dfu_if *dif, void *v )
{
  struct dfu_if *v_dif = v;

  memcpy( v_dif, dif,
          sizeof( *v_dif ) - sizeof( struct usb_dev_handle * ) );

  /* return a value that makes find_dfu_if return immediately */
  return 1;
}

/* Find a DFU interface (and altsetting) in a given device */
static int find_dfu_if( struct usb_device *dev,
                        int ( *handler ) ( struct dfu_if *, void * ),
                        void *v )
{
  struct usb_config_descriptor *cfg;
  struct usb_interface_descriptor *intf;
  struct usb_interface *uif;
  struct dfu_if _dif, *dfu_if = &_dif;
  int cfg_idx, intf_idx, alt_idx;
  int rc;

  memset( dfu_if, 0, sizeof( *dfu_if ) );

  for( cfg_idx = 0; cfg_idx < dev->descriptor.bNumConfigurations;
       cfg_idx++ ) {
    cfg = &dev->config[cfg_idx];
    /* in some cases, noticably FreeBSD if uid != 0,
     * the configuration descriptors are empty */
    if( !cfg )
      return 0;
    for( intf_idx = 0; intf_idx < cfg->bNumInterfaces; intf_idx++ ) {
      uif = &cfg->interface[intf_idx];
      if( !uif )
        return 0;
      for( alt_idx = 0; alt_idx < uif->num_altsetting; alt_idx++ ) {
        intf = &uif->altsetting[alt_idx];
        if( !intf )
          return 0;
        if( intf->bInterfaceClass == 0xfe &&
            intf->bInterfaceSubClass == 1 ) {
          dfu_if->dev = dev;
          dfu_if->vendor = dev->descriptor.idVendor;
          dfu_if->product = dev->descriptor.idProduct;
          dfu_if->configuration = cfg_idx;
          dfu_if->interface = intf->bInterfaceNumber;
          dfu_if->altsetting = intf->bAlternateSetting;
          if( (dfu_if->vendor == 0x483) && (dfu_if->product == 0xdf11) ) {
            //This is a case of device corresponding to STM32 DFU Bootloader
            //which resides on STM32F105 MCU.
            //In this case it's guaranteed that this device is already in DFU 
            //mode so there is no need to check an interface protocol or other 
            //things.
            dfu_if->flags |= DFU_IFF_DFU;
          } else if( intf->bInterfaceProtocol == 2 )
            dfu_if->flags |= DFU_IFF_DFU;
          else
            dfu_if->flags &= ~DFU_IFF_DFU;
          if( !handler )
            return 1;
          rc = handler( dfu_if, v );
          dfu_device_found = 1;
          if( rc != 0 )
            return rc;
        }
      }
    }
  }

  return 0;
}

static int get_first_dfu_if( struct dfu_if *dif )
{
  return find_dfu_if( dif->dev, &_get_first_cb, ( void * ) dif );
}

static int print_dfu_if( struct dfu_if *dfu_if, void *v )
{
  struct usb_device *dev = dfu_if->dev;
  int if_name_str_idx;
  char name[MAX_STR_LEN + 1] = "UNDEFINED";

  if_name_str_idx = dev->config[dfu_if->configuration]
    .interface[dfu_if->interface]
    .altsetting[dfu_if->altsetting].iInterface;
  if( if_name_str_idx ) {
    if( !dfu_if->dev_handle )
      dfu_if->dev_handle = usb_open( dfu_if->dev );
    if( dfu_if->dev_handle )
      usb_get_string_simple( dfu_if->dev_handle,
                             if_name_str_idx, name, MAX_STR_LEN );
  }

  printf( "Found %s: [0x%04x:0x%04x] devnum=%u, cfg=%u, intf=%u, "
          "alt=%u, name=\"%s\"\n",
          dfu_if->flags & DFU_IFF_DFU ? "DFU" : "Runtime",
          dev->descriptor.idVendor, dev->descriptor.idProduct,
          dev->devnum, dfu_if->configuration, dfu_if->interface,
          dfu_if->altsetting, name );

  return 0;
}

static int alt_by_name( struct dfu_if *dfu_if, void *v )
{
  struct usb_device *dev = dfu_if->dev;
  int if_name_str_idx;
  char name[MAX_STR_LEN + 1] = "UNDEFINED";

  if_name_str_idx =
    dev->config[dfu_if->configuration].interface[dfu_if->interface].
    altsetting[dfu_if->altsetting].iInterface;
  if( !if_name_str_idx )
    return 0;
  if( !dfu_if->dev_handle )
    dfu_if->dev_handle = usb_open( dfu_if->dev );
  if( !dfu_if->dev_handle )
    return 0;
  if( usb_get_string_simple
      ( dfu_if->dev_handle, if_name_str_idx, name, MAX_STR_LEN ) < 0 )
    return 0;                   /* should we return an error here ? */
  if( strcmp( name, v ) )
    return 0;
  /*
   * Return altsetting+1 so that we can use return value 0 to indicate
   * "not found".
   */
  return dfu_if->altsetting + 1;
}

static int _count_cb( struct dfu_if *dif, void *v )
{
  int *count = v;

  ( *count )++;

  return 0;
}

/* Count DFU interfaces within a single device */
static int count_dfu_interfaces( struct usb_device *dev )
{
  int num_found = 0;

  find_dfu_if( dev, &_count_cb, ( void * ) &num_found );

  return num_found;
}

/* Iterate over all matching DFU capable devices within system */
static int iterate_dfu_devices( struct dfu_if *dif,
                                int ( *action ) ( struct usb_device *
                                                  dev, void *user ),
                                void *user )
{
  struct usb_bus *usb_bus;
  struct usb_device *dev;

  /* Walk the tree and find our device. */
  for( usb_bus = usb_get_busses(  ); NULL != usb_bus;
       usb_bus = usb_bus->next ) {
    for( dev = usb_bus->devices; NULL != dev; dev = dev->next ) {
      int retval;

      if( dif && ( dif->flags &
                   ( DFU_IFF_VENDOR | DFU_IFF_PRODUCT ) ) &&
          ( dev->descriptor.idVendor != dif->vendor ||
            dev->descriptor.idProduct != dif->product ) )
        continue;
      if( dif && ( dif->flags & DFU_IFF_DEVNUM ) &&
          ( atoi( usb_bus->dirname ) != dif->bus ||
            dev->devnum != dif->devnum ) )
        continue;
      if( !count_dfu_interfaces( dev ) )
        continue;

      retval = action( dev, user );
      if( retval )
        return retval;
    }
  }
  return 0;
}

static int found_dfu_device( struct usb_device *dev, void *user )
{
  struct dfu_if *dif = user;

  dif->dev = dev;
  return 1;
}

/* Find the first DFU-capable device, save it in dfu_if->dev */
static int get_first_dfu_device( struct dfu_if *dif )
{
  return iterate_dfu_devices( dif, found_dfu_device, dif );
}

static int count_one_dfu_device( struct usb_device *dev, void *user )
{
  int *num = user;

  ( *num )++;
  return 0;
}

/* Count DFU capable devices within system */
static int count_dfu_devices( struct dfu_if *dif )
{
  int num_found = 0;

  iterate_dfu_devices( dif, count_one_dfu_device, &num_found );
  return num_found;
}

static int list_dfu_interfaces( void )
{
  struct usb_bus *usb_bus;
  struct usb_device *dev;

  /* Walk the tree and find our device. */
  for( usb_bus = usb_get_busses(  ); NULL != usb_bus;
      usb_bus = usb_bus->next ) {
    for( dev = usb_bus->devices; NULL != dev; dev = dev->next ) {
      find_dfu_if( dev, &print_dfu_if, NULL );
    }
  }
  return 0;
}

static int parse_vendprod( struct usb_vendprod *vp, const char *str )
{
  unsigned long vend, prod;
  const char *colon;

  colon = strchr( str, ':' );
  if( !colon || strlen( colon ) < 2 )
    return -EINVAL;

  vend = strtoul( str, NULL, 16 );
  prod = strtoul( colon + 1, NULL, 16 );

  if( vend > 0xffff || prod > 0xffff )
    return -EINVAL;

  vp->vendor = vend;
  vp->product = prod;

  return 0;
}

static void help( void )
{
  printf( "Usage: dfu-util [options] ...\n"
          "  -h --help\t\t\tPrint this help message\n"
          "  -V --version\t\t\tPrint the version number\n"
          "  -l --list\t\t\tList the currently attached DFU capable USB devices\n"
          "  -d --device vendor:product\tSpecify Vendor/Product ID of DFU device\n"
          "  -D --download file\t\tWrite firmware from <file> into device\n"
          "  -o --offset offset\t\tSet starting address for transfer\n" 
	  "  -e                \t\tErase memory before writing\n"
	  "  -X --execute\t\t\tStart program from offset\n"
	);
}

static void print_version( void )
{
  puts( "(C) 2007-2008 by OpenMoko, Inc., (C) 2010-2011 STC Metrotek" );
  printf( "dfu-util version %s\n", VERSION "+svn" DFU_UTIL_VERSION );
}

void open_usb_device( struct dfu_if *dif )
{
  printf( "Opening USB Device 0x%04x:0x%04x...\n", dif->vendor,
          dif->product );
  dif->dev_handle = usb_open( dif->dev );
  if( !dif->dev_handle ) {
    fprintf( stderr, "Cannot open device: %s\n", usb_strerror(  ) );
    exit( 1 );
  }
}

void locate_dfu_device(struct dfu_if *dif,char * error_string)
{
  int num_devs;
  num_devs = count_dfu_devices( dif );
  if( num_devs == 0 ) {
    fprintf( stderr, "%s\n",error_string );
    exit( 1 );
  }else if( num_devs > 1 ) {
    /* We cannot safely support more than one DFU capable device
     * with same vendor/product ID, since during DFU we need to do
     * a USB bus reset, after which the target device will get a
     * new address */
    fprintf( stderr, "More than one DFU capable USB device found, "
             "you might try `--list' and then disconnect all but one "
             "device\n" );
    exit( 3 );
  }
  if( !get_first_dfu_device( dif ) )
    exit( 3 );
}

static void set_altinterface( struct dfu_if *dif )
{
  printf( "Setting Alternate Setting ...\n" );
  if( usb_set_altinterface( dif->dev_handle, dif->altsetting ) < 0 ) {
    fprintf( stderr, "Cannot set alternate interface: %s\n",
        usb_strerror(  ) );
    exit( 1 );
  }
}

static void claim_interface(struct dfu_if *dif)
{
  printf( "Claiming USB DFU Runtime Interface...\n" );
  if( usb_claim_interface( dif->dev_handle, dif->interface ) < 0 ) {
    fprintf( stderr, "Cannot claim interface: " );
    fprintf( stderr, usb_strerror( ) );
    fprintf( stderr, "\n" );
    exit( 1 );
  }
}

void check_runtime_interface( struct dfu_if *_rt_dif, struct dfu_if *dif,
    struct dfu_status *status )
{
  if( !_rt_dif->flags & DFU_IFF_DFU ) {

    while(1) { 

      /* In the 'first round' during runtime mode, there can only be one
       * DFU Interface descriptor according to the DFU Spec. */

      /* FIXME: check if the selected device really has only one */

      claim_interface(_rt_dif);

      printf( "Determining device status: " );
      if( dfu_get_status
          ( _rt_dif->dev_handle, _rt_dif->interface, status ) < 0 ) {
        fprintf( stderr, "error get_status: %s\n", usb_strerror(  ) );
        exit( 1 );
      }
      printf( "state = %s, status = %d\n",
          dfu_state_to_string( status->bState ), status->bStatus );

      switch ( status->bState ) {
        case DFU_STATE_appIDLE:
        case DFU_STATE_appDETACH:
          printf( "Device really in Runtime Mode, send DFU "
              "detach request...\n" );
          if( dfu_detach( _rt_dif->dev_handle,
                _rt_dif->interface, 1000 ) < 0 ) {
            fprintf( stderr, "error detaching: %s\n",
                usb_strerror(  ) );
            exit( 1 );
            break;
          }
          printf( "Resetting USB...\n" );
          int ret = usb_reset( _rt_dif->dev_handle );
          if( ret < 0 && ret != -ENODEV )
            fprintf( stderr,
                "error resetting after detach: %s\n",
                usb_strerror(  ) );
          sleep( 2 );
          break;
        case DFU_STATE_dfuERROR:
          printf( "dfuERROR, clearing status\n" );
          if( dfu_clear_status( _rt_dif->dev_handle,
                _rt_dif->interface ) < 0 ) {
            fprintf( stderr, "error clear_status: %s\n",
                usb_strerror(  ) );
            exit( 1 );
            break;
          }
          break;
        default:
          fprintf( stderr, "WARNING: Runtime device already "
              "in DFU state ?!?\n" );
          break;
      }

      if( in_array( status->bState, ( unsigned char[3] ) {
            DFU_STATE_appIDLE, DFU_STATE_appDETACH,
            DFU_STATE_dfuERROR}, 3 ) >= 0 ) {

        /* now we need to re-scan the bus and locate our device */
        int ret = 0;
        if( ( ret = usb_find_devices(  ) ) < 2 )
          printf( "not at least 2 device changes found ?!?\n" );

        locate_dfu_device(dif,"Lost device after RESET?");

        open_usb_device(dif);
      } else break;

      sleep(1);
    }
  }
}


void check_num_of_dfu_interfaces( struct dfu_if *dif )
{
  int num_ifs = count_dfu_interfaces( dif->dev );
  if( num_ifs < 0 ) {
    fprintf( stderr, "No DFU Interface after RESET?!?\n" );
    exit( 1 );
  } else if( num_ifs == 1 ) {
    if( !get_first_dfu_if( dif ) ) {
      fprintf( stderr, "Can't find the single available "
          "DFU IF\n" );
      exit( 1 );
    }
  } else if( (num_ifs > 1)
      && (!dif->flags) & ( DFU_IFF_IFACE | DFU_IFF_ALT ) ) {
    fprintf( stderr,
        "We have %u DFU Interfaces/Altsettings, "
        "you have to specify one via --intf / --alt options\n",
        num_ifs );
    exit( 1 );
  }
}


void find_alt_by_name( struct dfu_if *dif, char *alt_name )
{
  int n;

  n = find_dfu_if( dif->dev, &alt_by_name, alt_name );
  if( !n ) {
    fprintf( stderr, "No such Alternate Setting: \"%s\"\n",
        alt_name );
    exit( 1 );
  }
  if( n < 0 ) {
    fprintf( stderr, "Error %d in name lookup\n", n );
    exit( 1 );
  }
  dif->altsetting = n - 1;
}


void wait_for_dfu_idle_state( struct dfu_if *dif, struct dfu_status *status )
{
  do {
    printf( "Determining device status: " );
    if( dfu_get_status( dif->dev_handle, dif->interface, status ) <
        0 ) {
      fprintf( stderr, "error get_status: %s\n", usb_strerror(  ) );
      exit( 1 );
    }
    printf( "state = %s, status = %d\n",
        dfu_state_to_string( status->bState ), status->bStatus );

    switch ( status->bState ) {
      case DFU_STATE_appIDLE:
      case DFU_STATE_appDETACH:
        fprintf( stderr, "Device still in Runtime Mode!\n" );
        exit( 1 );
        break;
      case DFU_STATE_dfuERROR:
        printf( "dfuERROR, clearing status\n" );
        if( dfu_clear_status( dif->dev_handle, dif->interface ) < 0 ) {
          fprintf( stderr, "error clear_status: %s\n",
              usb_strerror(  ) );
          exit( 1 );
        }
        continue;
      case DFU_STATE_dfuDNLOAD_IDLE:
      case DFU_STATE_dfuUPLOAD_IDLE:
        printf( "aborting previous incomplete transfer\n" );
        if( dfu_abort( dif->dev_handle, dif->interface ) < 0 ) {
          fprintf( stderr, "can't send DFU_ABORT: %s\n",
              usb_strerror(  ) );
          exit( 1 );
        }
        continue;
      case DFU_STATE_dfuIDLE:
        printf( "dfuIDLE, continuing\n" );
        break;
    }
  }  while( status->bState != DFU_STATE_dfuIDLE );
}


void check_dfu_status_ok( struct dfu_if *dif, struct dfu_status *status )
{
  if( DFU_STATUS_OK != status->bStatus ) {
    printf( "WARNING: DFU Status: '%s'\n",
        dfu_status_to_string( status->bStatus ) );
    /* Clear our status & try again. */
    dfu_clear_status( dif->dev_handle, dif->interface );
    dfu_get_status( dif->dev_handle, dif->interface, status );

    if( DFU_STATUS_OK != status->bStatus ) {
      fprintf( stderr, "Error: %d\n", status->bStatus );
      exit( 1 );
    }
  }
}


/** \brief Parse command line arguments passed while running a program.
 *  \param argc number of command line arguments
 *  \param argv array containing pointers to command line arguments
 *  \param param - structure for holding parameters
 *  \param vend an instance of structure for vendor's and product' codes
 *  \param dif device interface structure
 */ 
void parse_command_line_opts( int argc, char **argv, parameters *param,
    struct usb_vendprod *vend, struct dfu_if *dif )
{
  param->do_execute=NO_FLAGS;
  while( 1 ) {
    int c, option_index = 0;
    c = getopt_long( argc, argv, "hVvld:c:o:i:a:t:U:D:REeXr", opts,
        &option_index );
    if( c == -1 )
      break;

    switch ( c ) {
      case 'h':
        help( );
        exit( 0 );
        break;
      case 'V':
        print_version( );
        exit( 0 );
        break;
      case 'l':
        puts("Available DFU Interfaces: ");
        list_dfu_interfaces(  );
        if( !dfu_device_found ) {
          puts("No available DFU Interfaces found.");
        }
        exit( 0 );
        break;
      case 'd':
        /* Parse device */
        if( parse_vendprod( vend, optarg ) < 0 ) {
          fprintf( stderr, "unable to parse `%s'\n", optarg );
          exit( 2 );
        }
        dif->vendor = vend->vendor;
        dif->product = vend->product;
        dif->flags |= ( DFU_IFF_VENDOR | DFU_IFF_PRODUCT );
        break;
      case 'c':
        /* Configuration */
        dif->configuration = atoi( optarg );
        dif->flags |= DFU_IFF_CONFIG;
        break;
      case 'D':
        param->mode = MODE_DOWNLOAD;
        param->filename = optarg;
        break;
      case 'X':
        param->do_execute |= DO_EXECUTE;
        break;
      case 'o':
        /* get offset */
        param->offset = strtol( optarg,NULL, 0 );
        break;
      case 'e':
	//erase before writing
	param->do_execute |= DO_ERASE;
	break;
      case 't':
        param->transfer_size = atoi( optarg );
        break;
      default:
        help(  );
        exit( 2 );
    }
  }
}


/** \brief Entry point to application.
 *  \param argc number of command line arguments
 *  \param argv array containing pointers to command line arguments
 *  \return error code
 */ 
int main( int argc, char **argv )
{
  struct usb_vendprod vendprod;
  struct dfu_if _rt_dif, _dif, *dif = &_dif;
  parameters param;
  param.transfer_size = 2048;
  param.offset = 0x08000000;
  param.mode = MODE_NONE;
  struct dfu_status status;
  param.filename = NULL;
  param.alt_name = NULL;        /* query alt name if non-NULL */
  param.do_execute = NO_FLAGS;

  memset( dif, 0, sizeof( *dif ) );

  usb_init(  );
  usb_find_busses(  );
  usb_find_devices(  );

  parse_command_line_opts( argc, argv, &param,
      &vendprod, dif );

  if( param.mode == MODE_NONE ) {
    fprintf( stderr, "You need to specify -D \n" );
    help( );
    exit( 2 );
  }

  if( !param.filename && ( param.mode == MODE_DOWNLOAD ) ) {
    fprintf( stderr, "You need to specify a filename to -D \n" );
    help( );
    exit( 2 );
  }

  puts("Please wait while searching for available DFU devices...");

  //wait for the moment when dfu device is found
  while(!dfu_device_found) {
    usb_find_devices(  );
    list_dfu_interfaces(  );
    sleep(1);
  }

  dfu_init( 5000 );

  locate_dfu_device(dif,"No DFU capable USB device found");

  /* We have exactly one device. It's usb_device is now in dif->dev */
  open_usb_device( dif );

  /* try to find first DFU interface of device */
  memcpy( &_rt_dif, dif, sizeof( _rt_dif ) );
  if( !get_first_dfu_if( &_rt_dif ) )
    exit( 1 );

  check_runtime_interface( &_rt_dif, dif, &status );

  if( param.alt_name ) {
    find_alt_by_name( dif, param.alt_name );
  }

  print_dfu_if( dif, NULL );

  check_num_of_dfu_interfaces( dif );

  claim_interface(dif);

  set_altinterface(dif);

  wait_for_dfu_idle_state( dif, &status );

  printf( "Transfer Size = 0x%04x\n", param.transfer_size );
  printf( "Offset = 0x%08x\n", param.offset);

  check_dfu_status_ok( dif, &status );

  switch ( param.mode ) {
    case MODE_DOWNLOAD:
      if( stm32dfu_do_dnload( dif->dev_handle, dif->interface, param) < 0 )
//		      .transfer_size, param.filename, param.offset,param.do_execute ) < 0 )
        exit( 1 );
      break;
    default:
      fprintf( stderr, "Unsupported mode: %u\n", param.mode );
      exit( 1 );
  }

  exit( 0 );
}
