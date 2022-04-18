#include <stdio.h>

int main()
{
    int a = 10;
    unsigned b = -123;
    char c[0x100];

    for(int i=0; i<0x100; i++)
    {
        if(i % 4)
            a += b;
        else
            b -= a;

        c[a%0x100] = a*b + (int)b/a;
    }
}