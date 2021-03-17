// __attribute__((import_module("MYMODULE")))
// __attribute__((import_name("FUNCNAME")))
// __attribute__((export_name("FUNCNAME")))
// __attribute__((used))

#include <stdlib.h> // malloc
#include <stdio.h> // printf
#include <stddef.h> // size_t

// host defined functions
__attribute__((import_module("host")))
void* dlopen(const char *);
__attribute__((import_module("host")))
void* dlsym(void*, const char *);
__attribute__((import_module("host")))
char* dlerror();
__attribute__((import_module("host")))
void* dlclose(void*);

// alloc function export is required by our runtime to be able to allocate memory
__attribute__((used))
void* alloc(size_t size) {
    void* p = malloc(size);
    return p;
}

void check_error(void* p) {
    if (!p) {
        printf("ERR: %p", dlerror());
    }
}

__attribute__((used))
int main() {
    // Despite loading and unloading modules constantly, the memory consumption of the application should not increase
    int i = 0;
    while (1) {
        void* handle1;
        void* handle3;
        handle1 = dlopen("jumbo");
        check_error(handle1);
        {
            void (*handle2)(const char*, int);
            handle2 = dlsym(handle1, "exported_print");
            check_error(handle2);
            handle2("loop: %i\n", ++i); // print using a function from the loaded module
        }
        handle3 = dlclose(handle1);
        check_error(handle3);
    }
    return 0;
}
