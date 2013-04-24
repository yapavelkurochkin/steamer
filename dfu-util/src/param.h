#ifndef _PARAM_H_
#define _PARAM_H_

#include "main.h"

enum mode {
	MODE_NONE,
	MODE_DOWNLOAD
};

/*
 * \struct parameters 
 *  \param filename a name of file to send to device
 *  \param do_execute a flag indicating whether execute a firmware passed to 
 *         a device or if there is need for memory erase
 *  \param offset initial address of memory to start a writing from        
 *  \param transfer_size a size of USB transfer for this firmware transmitting
 *         operation
 */
typedef struct parameters {
  unsigned int transfer_size;
  unsigned int offset;
  char *filename;
  char *alt_name;   /* query alt name if non-NULL */
  int do_execute;
  enum mode mode;
} parameters;


#endif
