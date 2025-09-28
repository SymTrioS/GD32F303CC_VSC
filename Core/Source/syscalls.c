/*!
    \file    syscalls.c
    \version 2025-09-28, V2.2.0, firmware for GD32F303 Prime-S73P board
    \author  SymTrioS
*/
/*
    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 

NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include <stdio.h>
#include <gd32f30x.h>
#include <gd32f30x_gpio.h>
#include "prime-s73p.h"

#define GCC
//#define uLIB
//-------------------------------for GNU-------------------------------//
#ifdef GCC
int _write (int fd, char *pBuffer, int size)
{ 
	for (int i = 0; i < size; i++)
	{
          usart_data_transmit(COM2, (uint8_t)pBuffer[i]);
          while(RESET == usart_flag_get(COM2, USART_FLAG_TBE));
	}
	return size;
}

int _read(int fd, char *pBuffer, int size) {
    for (int i = 0; i < size; i++) {
        while (usart_flag_get(COM2, USART_FLAG_RBNE) == 1) {
        }

        pBuffer[i] = usart_data_receive(COM2);
    }
    return size;
}
#endif

//-----------------------------for MicroLIB-----------------------------//
#ifdef uLIB
int fputc(int ch, FILE *f)
{
  // Place your implementation of fputc here
  // e.g. write a character to the USART1 and Loop until the end of transmission
	usart_data_transmit(COM2, (uint8_t)ch);
	while(RESET == usart_flag_get(COM2, USART_FLAG_TBE));
	return ch;
}
#endif
/*
//---------------------------------------------------------------------------//
extern int _end;

void * _sbrk ( int incr )
{
    static unsigned char *heap = NULL;
    unsigned char *prev_heap;

    if (heap == NULL) {
        heap = (unsigned char *)&_end;
    }
    prev_heap = heap;

    heap += incr;

    return (void *) prev_heap;
}

int _open(char *path, int flags, ...)
{
    return -1;
}

int _close(int file)
{
    return -1;
}

int _lseek(int file, int ptr, int dir)
{
    return 0;
}

int _fstat(int file, struct stat *st)
{
    return 0;
}

int _isatty(int file)
{
    return 1;
}
//---------------------------------------------------------------------------//
*/
