#ifndef _I2CBUS_H
#define _I2CBUS_H

namespace first_project {
    class I2cbus{
        public:
            I2cbus();
            virtual void start( unsigned char sda, unsigned char scl, unsigned int vel );
            virtual void start( unsigned int vel );
            virtual void start( void );
            // virtual bool testid( unsigned char id );
            virtual void txi2c( unsigned char adr, const unsigned char *dat, int n );
            virtual void txi2c( unsigned char adr, unsigned char dat, int n );
            virtual void txi2c( unsigned char adr, unsigned char dat );
            // virtual unsigned char rxi2c( unsigned char adr );
            // virtual void rxi2c ( unsigned char adr, unsigned char * dat, int n );
    };
} // namespace first_project

#endif