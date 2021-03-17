#!/bin/bash

echo "
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdbool.h>
#include <sys/stat.h>
#include <stdio.h>
#include <math.h>

__attribute__((import_module(\"host\")))
unsigned char* fopen_and_read(const char *, int *);
__attribute__((import_module(\"host\")))
int write_file(const char *, const char *, int);

__attribute__((used))
void exported_print(const char *format, int i) {
    printf(format, i);
}
" > jumbo.c

for i in `seq 1 3000`; do

echo "
__attribute__((used))
int dummy${i}(char *filename, char *output_file, int start, int end, int needle) {
	unsigned char *data = fopen_and_read(filename, &end);
	while (start <= end){
		int mid = start + (end - start)/2;
		if (data[mid] == needle)
			return mid;
		if (data[mid] < needle)
			start = mid + 1;
		else
			end = mid - 1;
	}
	unsigned int prime = 2;
    unsigned int count = 1;
    unsigned int number = 3;
    while(count < needle)
    {
        bool isprime = true;
        for(unsigned int i = 3; i <= sqrt((long double)number); i+=2)
        {
            if(number % i == 0)
            {
                isprime = false;
                break;
            }
        }
        if(isprime)
        {
            prime = number;
            count++;
        }
        number += 2;
    }
	write_file(output_file, data, end);
	printf(\"Test print %p %s %u %u\\n\", data, output_file, count, prime);
	free(data);
	return 1;
}

" >> jumbo.c
done

