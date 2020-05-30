// Includes
#include <Arduino.h>
#include "TeensyCAN.h"

// Globals
float     Command_speed[NB_ESC] = {};

Teensycomm_struct_t Teensy_comm = {
                                  COMM_MAGIC,
                                  {},
                                  {},
                                  {}
                                  };
RPicomm_struct_t   RPi_comm =   {
                                  COMM_MAGIC,
                                  {}
                                  };


// Manage communication with the host
int Teensy_comm_update( void ) {
  static int          i;
  static uint8_t      *ptin   = (uint8_t*)(&RPi_comm),
                      *ptout  = (uint8_t*)(&Teensy_comm);
  static int          ret;
  static int          in_cnt = 0;
  
  ret = 0;
  
  // Read all incoming bytes available until incoming structure is complete
  while(  ( Serial.available( ) > 0 ) && 
          ( in_cnt < (int)sizeof( RPi_comm ) ) )
    ptin[in_cnt++] = Serial.read( );
  
  // Check if a complete incoming packet is available
  if ( in_cnt == (int)sizeof( RPi_comm ) ) {
  
    // Clear incoming bytes counter
    in_cnt = 0;
    
    
    // Check the validity of the magic number
    if ( RPi_comm.magic != COMM_MAGIC ) {
    
      // Flush input buffer
      while ( Serial.available( ) )
        Serial.read( );
    
      ret = ERROR_MAGIC;
    }
    else {
      for ( i = 0; i < NB_ESC; i++ )
        Command_speed[i] = RPi_comm.RPM[i];

      for ( i = 0; i < NB_ESC; i++ ) {
        Teensy_comm.deg[i] = 1;
        Teensy_comm.rpm[i] = Command_speed[i];
        Teensy_comm.amp[i] = 3;
      }
      
      // Send data structure to host
      Serial.write( ptout, sizeof( Teensy_comm ) );
      
      // Force immediate transmission
      Serial.send_now( );
    }
  }

  return ret;
}

void setup() {

  Serial.begin( USB_UART_SPEED );

}

void loop( ) {

  Teensy_comm_update(  );
}
