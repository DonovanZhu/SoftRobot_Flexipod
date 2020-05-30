#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <strings.h>
#include <limits.h>
#include <dirent.h>
#include <sys/ioctl.h>
#include <sys/param.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/serial.h>
#include <linux/input.h>
#include "host.h"

// Flags
#define HOST_STANDALONE                             // main is added

#define HOST_MAX_DEVICES    5                       // Max number of teensys
//#define HOST_MODEMDEVICE    "/dev/serial/by-id/usb-Teensyduino_USB_Serial_6582050-if00"
#define HOST_MODEMDEVICE    "/dev/ttyACM0"
#define HOST_DEV_SERIALNB     6582050              	// Serial number of the teensy
#define HOST_DEV_SERIALLG     10                    // Max length of a serial number
#define HOST_SERIAL_DEV_DIR   "/dev/serial/by-id/"

#define HOST_BAUDRATE       B115200                 // Serial baudrate
#define HOST_READ_TIMEOUT   5                       // Tenth of second
#define HOST_STEP_REF       200                     // Velocity reference step size (10 rpm motor)


// Globals
int                 Host_fd[HOST_MAX_DEVICES] = 
                          { HOST_ERROR_FD, 
                            HOST_ERROR_FD,
                            HOST_ERROR_FD,
                            HOST_ERROR_FD,
                            HOST_ERROR_FD };        // Serial port file descriptor
                            
char                Host_devname[HOST_MAX_DEVICES][PATH_MAX] =
                          { "",
                            "",
                            "",
                            "",
                            "" };                   // Serial port devname used to get fd with open
                            
struct termios      Host_oldtio[HOST_MAX_DEVICES];  // Backup of initial tty configuration

Teensycomm_struct_t Teensy_comm[HOST_MAX_DEVICES];
RPicomm_struct_t   RPi_comm[HOST_MAX_DEVICES];

//
//  Get the device name from the device serial number
//
char *Host_name_from_serial( uint32_t serial_nb ) {
  DIR           *d;
  struct dirent *dir;
  char          serial_nb_char[HOST_DEV_SERIALLG];
  static char   portname[PATH_MAX];
  
  // Convert serial number into string
  snprintf( serial_nb_char, HOST_DEV_SERIALLG, "%d", serial_nb );
  
  // Open directory where serial devices can be found
  d = opendir( HOST_SERIAL_DEV_DIR );
  
  // Look for a device name contining teensy serial number
  if ( d ) {
  
    // Scan each file in the directory
    while ( ( dir = readdir( d ) ) != NULL ) {
      if ( strstr( dir->d_name, serial_nb_char ) )  {
      
        // A match is a device name containing the serial number
        snprintf( portname, PATH_MAX, "%s%s", HOST_SERIAL_DEV_DIR, dir->d_name );
        return portname;
      }
    }
    closedir( d );
  }
  
  return NULL;
}

//
//  Get the file descriptor index which device name contains
//  specified serial number. 
//  Returns -1 if no matching fd is found.
//
int Host_get_fd( uint32_t serial_nb ) {
  int   i;
  char  serial_nb_char[HOST_DEV_SERIALLG];
  
  // Convert serial number into string
  snprintf( serial_nb_char, HOST_DEV_SERIALLG, "%d", serial_nb );
    
  for ( i = 0; i < HOST_MAX_DEVICES; i++ )
    if ( Host_fd[i] != HOST_ERROR_FD )
        if ( strstr( Host_devname[i], serial_nb_char ) )
          return i;

  return HOST_ERROR_FD;
}

//
//  Initialize serial port
//
int Host_init_port( uint32_t serial_nb )  {
  struct  termios newtio;
  int     check_fd;
  int     i, fd_idx;
  char    *portname;
  
  // Check if device plugged in
  portname = Host_name_from_serial( serial_nb );
  if ( !portname )
    return HOST_ERROR_DEV;

  // Open device
  check_fd = open( portname, O_RDWR | O_NOCTTY | O_NONBLOCK );
  
  if ( check_fd < 0 )  {
    perror( portname );
    return HOST_ERROR_DEV;
  }
  
  // Look for an empty slot to store the fd
  for ( fd_idx = 0; fd_idx < HOST_MAX_DEVICES; fd_idx++ )
    if ( Host_fd[fd_idx] == HOST_ERROR_FD )
      break;
      
  // Close fd and throw an error if all slots are used
  if ( fd_idx == HOST_MAX_DEVICES ) {
    close( check_fd );
    return HOST_ERROR_MAX_DEV;
  }
  
  // Store the fd and the corresponding devname
  Host_fd[fd_idx] = check_fd;
  strncpy( Host_devname[fd_idx], portname, PATH_MAX );
  
  // Initialize corresponding data structure
  for ( i = 0; i < MAX_ESC; i++ )  {
    RPi_comm[fd_idx].magic = COMM_MAGIC;
    RPi_comm[fd_idx].RPM[i] =  0;
  }

  /* Save current port settings */
  tcgetattr( check_fd, &Host_oldtio[fd_idx] );

  /* Define new settings */
  bzero( &newtio, sizeof(newtio) );
  cfmakeraw( &newtio );

  newtio.c_cflag =      HOST_BAUDRATE | CS8 | CLOCAL | CREAD;
  newtio.c_iflag =      IGNPAR;
  newtio.c_oflag =      0;
  newtio.c_lflag =      0;
  newtio.c_cc[VTIME] =  0;
  newtio.c_cc[VMIN] =   0;


  /* Apply the settings */
  tcflush( check_fd, TCIFLUSH );
  tcsetattr( check_fd, TCSANOW, &newtio );

  return 0;
}

//
//  Release serial port
//
void Host_release_port( uint32_t serial_nb )  {
  int   fd_idx;
    
  // Get fd index from serial number
  fd_idx = Host_get_fd( serial_nb );
  
  if ( fd_idx != HOST_ERROR_FD ) {
    // Restore initial settings if needed
    tcsetattr( Host_fd[fd_idx], TCSANOW, &Host_oldtio[fd_idx] );
    close( Host_fd[fd_idx] );
    
    // Clear fd and corresponding devname
    Host_fd[fd_idx] = HOST_ERROR_FD;
    strncpy( Host_devname[fd_idx], "", PATH_MAX );
  }
}

//
// Manage communication with the teensy connected to portname
//
int Host_comm_update( uint32_t            serial_nb,
                      int16_t             *RPM,
                      Teensycomm_struct_t **comm ) {
                      
  int                 i, ret, res = 0, fd_idx;
  uint8_t             *pt_in;
  struct timespec     start, cur;
  unsigned long long  elapsed_us;
  
  // Get fd index
  fd_idx = Host_get_fd( serial_nb );
  
  // Check if fd index is valid
  if ( fd_idx == HOST_ERROR_FD )
    return HOST_ERROR_FD;
  
  // Update output data structue
  for ( i = 0; i < MAX_ESC; i++ )  {
    RPi_comm[fd_idx].RPM[i] = 100;
  }
   
  // Send output structure
  res = write( Host_fd[fd_idx], &RPi_comm[fd_idx], sizeof( RPi_comm[fd_idx] ) );
  if ( res < 0 )  {
    perror( "write RPi_comm" );
    return HOST_ERROR_WRITE_SER;
  }
  
  // Flush output buffer
  fsync( Host_fd[fd_idx] );

  // Wait for response

  // Get current time
  clock_gettime( CLOCK_MONOTONIC, &start );

  // Reset byte counter and magic number
  res = 0;
  Teensy_comm[fd_idx].magic = 0;
  pt_in = (uint8_t*)(&Teensy_comm[fd_idx]);

  do  {
    ret = read( Host_fd[fd_idx], &pt_in[res], 1 );

    // Data received
    if ( ret > 0 )  {
      res += ret;
    }

    // Read error
    if ( ret < 0 )
      break;

    // Compute time elapsed
    clock_gettime( CLOCK_MONOTONIC, &cur );
    elapsed_us =  ( cur.tv_sec * 1e6 + cur.tv_nsec / 1e3 ) -
                  ( start.tv_sec * 1e6 + start.tv_nsec / 1e3 );

    // Timeout
    if ( elapsed_us / 100000 > HOST_READ_TIMEOUT )
      break;

  } while ( res < sizeof( Teensy_comm[fd_idx] ) );

  // Check response size
  if ( res != sizeof( Teensy_comm[fd_idx] ) )  {
    fprintf( stderr, "Packet with bad size received.\n" );

    // Flush input buffer
    while ( ( ret = read( Host_fd[fd_idx], pt_in, 1 ) ) )
      if ( ret <= 0 )
        break;
        
    return HOST_ERROR_BAD_PK_SZ;
  }

  // Check magic number
  if ( Teensy_comm[fd_idx].magic !=  COMM_MAGIC )  {
    fprintf( stderr, "Invalid magic number.\n" );
    return HOST_ERROR_MAGIC;
  }
  
  // Return pointer to Teensy_comm structure
  *comm = &Teensy_comm[fd_idx];
  
  // Print rountrip duration
  #ifdef HOST_STANDALONE
  fprintf( stderr, "Delay: %llu us\n", elapsed_us );
  #endif

  return 0;
}

#ifdef HOST_STANDALONE
//
//  main
//
int main( int argc, char *argv[] )  {

  int                 i, k, ret;
  int16_t             RPM[MAX_ESC];
  Teensycomm_struct_t *comm;
  
  // Initialize tunable PID data
  for ( i = 0; i < MAX_ESC; i++ )  {
    RPM[i] = HOST_STEP_REF;
  }
  
  // Initialize serial port
  if ( Host_init_port( HOST_DEV_SERIALNB ) )  {
    fprintf( stderr, "Error initializing serial port.\n" );
    exit( -1 );
  }

  // Testing roundtrip serial link duration
  i = 0;
  while (1) {
  
    // Serial exchange with teensy
    if ( ( ret = Host_comm_update(  HOST_DEV_SERIALNB,
                                    RPM,
                                    &comm ) ) )  {
      fprintf( stderr, "Error %d in Host_comm_update.\n", ret );
      break;
    }
    
    // Display telemetry
    for ( k = 0; k < NB_ESC; k++ )
      fprintf(  stderr,
                "#:%d.%d\tdeg:%d\trpm:%d\tA:%d\n",
                i,
                k,
                comm->deg[k],
                comm->rpm[k],
                comm->amp[k]);
    i++;          
  }

  // Restoring serial port initial configuration
  Host_release_port( HOST_DEV_SERIALNB );

  return 0;
}
#endif
