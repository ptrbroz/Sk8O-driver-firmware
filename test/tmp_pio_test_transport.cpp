#include <mbed.h>
#include <output_export.h>

#if MBED_MAJOR_VERSION == 6
UnbufferedSerial pc(USBTX, USBRX);
#else
RawSerial pc(USBTX, USBRX);
#endif

#ifdef __GNUC__
void output_start(unsigned int baudrate __attribute__((unused)))
#else
void output_start(unsigned int baudrate)
#endif
{
    pc.baud(115200);
}

void output_char(int c)
{
    #if MBED_MAJOR_VERSION == 6
pc.write(&c, 1);
#else
pc.putc(c);
#endif
}

void output_flush(void)
{
    
}

void output_complete(void)
{
   
}