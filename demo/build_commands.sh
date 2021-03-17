emcc main.c -o main.wasm -Wl,--export-table,--growable-table,--export=strlen,--export=open,--export=write,--export=malloc,--export=free,--export=memcpy,--export=realloc,--export=munmap,--export=close,--export=lseek,--export=mmap,--export=perror,--export=printf -s ALLOW_MEMORY_GROWTH=1  -Wno-pointer-sign

emcc jumbo.c -o jumbo.wasm -s SIDE_MODULE=1 -Wno-pointer-sign
