#ifndef _I2C_H
#define _I2C_H

#include <math.h>

#include "i2cbus.h"
#include "pin.h"
#include "delay.h"

namespace first_project {
    class I2C : public first_project::I2cbus {
        protected:
            int bufin;
            unsigned int iport;
            unsigned char buf[128];
            unsigned int dummy1, dummy2;
            Pin SDA, SCL;
            // I2C_TypeDef *port;
        public:

            // Methods to start the port.
            // void start( unsigned char sda, unsigned char scl, unsigned int v, unsigned char dir = 0 );
            void start( unsigned int v = 100000 );
            bool testid( unsigned char id );

            // Method to receive data in slave mode.
            int rtxi2c( unsigned char * dat );
            void dmatx( bool d );
            void dmarx( bool d );

            // Method to transmit an string data.
            void txi2c( unsigned char adr, const unsigned char * dat, int n );

            // Method to transmit a repeated data.
            void txi2c( unsigned char adr, unsigned char dat, int n );

            // Method to transmit a data.
            void txi2c( unsigned char adr, unsigned char dat );

            // Method to receive a data.
            unsigned char rtxi2c( unsigned char adr );

            // Method to receive an string data.
            void rtxi2c( unsigned char adr, unsigned char * dat, int n );

            // Method to assign an initial event Rx.
            // void setfuncrx( funint2 fun );

            // Method to assign an event Rx.
            // void setfunini( funint1 fun );

            // Method to assign an event Fin Rx.
            // void setfunfin( funint1 fun );
    };
}; // namespace first_project

#endif