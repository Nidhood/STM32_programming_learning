#include "i2cbus.h"

first_project::I2cbus::I2cbus() {}

void first_project::I2cbus::start( unsigned int vel ){}

void first_project::I2cbus::start( void ){}

void first_project::I2cbus::start( unsigned char sda, unsigned char sc1, unsigned int vel ){}

// bool first_project::I2cbus::testid( unsigned char id ){}

void first_project::I2cbus::txi2c( unsigned char adr, unsigned char dat, int n ){};

void first_project::I2cbus::txi2c( unsigned char adr, const unsigned char * dat, int n ){};

void first_project::I2cbus::txi2c( unsigned char adr, unsigned char dat ){};

// unsigned char first_project::I2cbus::rxi2c( unsigned char adr ){};

// void first_project::I2cbus::rxi2c( unsigned char adr, unsigned char * dat, int n ){};

