Requirements:
- cmake 3.11.4 or higher
- emscripten 2.0.0

Build:
- create a new `build` folder inside the source root folder
- go into the folder and run `cmake ..`
- run `make`
- The output is the `dynamic-linking` file inside the build folder

Running the demo:
- Build the host application with the above instructions
- Build the Wasm modules by running `build_commands.sh` in the demo folder
- Copy `dynamic-linking` to the demo folder and run the demo with `./dynamic-linking main.wasm`
- The demo will load and unload a module in a while loop. Inspect the memory use of the application to see that memory does not increase and that unloading and loading works as memory keeps changing.

Wasm Environment:
- The runtime provides WASI, host::dlopen, host::dlsym, host::dlclose.
