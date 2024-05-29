#include "rpl/private-header.hpp"

#define INPUT_SIZE 64 * 512
#define OUTPUT_SIZE 0

// Changing the buffer and increasing frame
void updateBuffer(int* bufferptr, uint* frameptr)
{
    for(;;)
    {
        for (int i = 0; i < INPUT_SIZE; i++)
        {
            bufferptr[i] = rand() % 256; // Generate a random number between 0 and 255
        }

        (*frameptr)++;
    }
}

int main()
{   

    return 0;
}
