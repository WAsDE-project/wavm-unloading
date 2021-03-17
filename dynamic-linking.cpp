#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include "WAVM/IR/Module.h"
#include "WAVM/IR/Types.h"
#include "WAVM/IR/Value.h"
#include "WAVM/Platform/File.h"
#include "WAVM/Runtime/Intrinsics.h"
#include "WAVM/Runtime/Linker.h"
#include "WAVM/Runtime/Runtime.h"
#include "WAVM/VFS/SandboxFS.h"
#include "WAVM/WASI/WASI.h"
#include "WAVM/WASM/WASM.h"

#include "WAVM/RuntimeABI/RuntimeABI.h"

#include <iostream>
#include <unordered_map>
#include <set>

using namespace WAVM;
using namespace WAVM::IR;
using namespace WAVM::Runtime;
using namespace WAVM::WASI;

struct InstanceData
{
	InstanceData(std::string const& name, GCPointer<Instance> const& instance, std::set<size_t>&& dependency_indexes, I32 memory_base = -1, I32 table_base = -1) :
	name(name), instance(instance), dependency_indexes(dependency_indexes), memory_base(memory_base), table_base(table_base)
	{
	}
	std::string name;
	GCPointer<Instance> instance;

	I32 memory_base;
	I32 table_base;
	U32 load_count = 1;
	std::unordered_map<std::string, I32> function_name_to_table_index;
	std::set<size_t> dependency_indexes;
};

struct Globals
{
	const FunctionType MALLOC_SIGNATURE = {{ValueType::i32}, {ValueType::i32}};
	const FunctionType FREE_SIGNATURE = {{}, {ValueType::i32}};
	GCPointer<Runtime::Function> MALLOC_PTR;
	GCPointer<Runtime::Function> FREE_PTR;
	GCPointer<Runtime::Memory> MEMORY_PTR;
	GCPointer<Runtime::Table> TABLE_PTR;
	GCPointer<Runtime::Instance> host_module_instance;
	std::unordered_multimap<Uptr, Uptr> FREE_TABLE_SIZE_TO_BASE; // TODO: we should have a memory management style allocation of table space
	std::unordered_map<std::string, std::pair<size_t, std::map<std::string, GCPointer<Runtime::Object>>>> ALL_EXPORTS;
	std::vector<InstanceData> INSTANCES;
};
static std::unique_ptr<Globals> GLOBALS;

static bool readFile(const char* path, std::vector<U8>& outBytes)
{
	FILE* file = fopen(path, "rb");
	if(!file)
	{
		fprintf(stderr, "Failed to open '%s' for reading: %s\n", path, strerror(errno));
		return false;
	}

	if(fseek(file, 0, SEEK_END))
	{
		fprintf(stderr, "Failed to seek to end of '%s': %s\n", path, strerror(errno));
		return false;
	}

	const long numBytes = ftell(file);
	if(numBytes < 0)
	{
		fprintf(stderr, "Failed to query position in '%s': %s\n", path, strerror(errno));
		return false;
	}
	const unsigned long numBytesUnsigned = (unsigned long)numBytes;

	if(fseek(file, 0, SEEK_SET))
	{
		fprintf(stderr, "Failed to seek to beginning of '%s': %s\n", path, strerror(errno));
		return false;
	}

	outBytes.resize(numBytesUnsigned);

	size_t numBytesRead = fread(outBytes.data(), 1, numBytesUnsigned, file);
	if(numBytesRead != numBytesUnsigned)
	{
		fprintf(stderr,
				"Failed to read %lu bytes from '%s': %s\n",
				numBytesUnsigned,
				path,
				strerror(errno));
		return false;
	}

	if(fclose(file))
	{
		fprintf(stderr, "Failed to close '%s': %s\n", path, strerror(errno));
		return false;
	}

	return true;
}

#include <exception>

constexpr uint32_t ERROR_BUFFER_LENGTH = 256;

struct MyException : public std::exception
{
	MyException(const char* format, ...)
	{
		va_list argptr;
		va_start(argptr, format);
		vsnprintf(buff, ERROR_BUFFER_LENGTH, format, argptr);
		va_end(argptr);
	}
	const char * what () const throw ()
    {
    	return buff;
    }

	char buff[ERROR_BUFFER_LENGTH];
};

InstanceData& GetInstanceData(I32 handle) {
	try {
		return GLOBALS->INSTANCES.at(handle-1);
	} catch (std::out_of_range& ex) {
		throw MyException("module %i handle is out of bounds", handle);
	}
}

void ExtractExports(IR::Module moduleIR, Instance* instance, std::string const & moduleName, size_t index) {
	auto it = GLOBALS->ALL_EXPORTS.find(moduleName);
	if (it != GLOBALS->ALL_EXPORTS.end()) {
		throw MyException("Export namespace %s already exists", moduleName.c_str());
		return;
	}
	auto& index_map = GLOBALS->ALL_EXPORTS[moduleName];
	index_map.first = index;
	auto& exportMap = index_map.second;
	for (auto& exp : moduleIR.exports) {
		exportMap[exp.name] = getInstanceExport(instance, exp.name);
	}
}

char* get_c_string_from_wasm_memory(Memory* memory, I32 string_ptr) {
	I32 string_itr = string_ptr;
	while (memoryRef<char>(memory, string_itr) != '\0') {
		++string_itr;
	}
	return memoryArrayPtr<char>(memory, string_ptr, string_itr-string_ptr+1);
}

U32 call_malloc(Context* context, U32 numBytes) {
	UntaggedValue args[1] = {numBytes};
	UntaggedValue results[1];
	invokeFunction(context, GLOBALS->MALLOC_PTR, GLOBALS->MALLOC_SIGNATURE, args, results);
	return results[0].u32;
}

Uptr AllocateTableElements(Uptr required_table_space) {
	if (required_table_space < 1) {
		return 0;
	}

	auto it = GLOBALS->FREE_TABLE_SIZE_TO_BASE.find(required_table_space);
	if (it != GLOBALS->FREE_TABLE_SIZE_TO_BASE.end()) {
		auto free_table_base = it->second;
		GLOBALS->FREE_TABLE_SIZE_TO_BASE.erase(it);
		return free_table_base;
	}

	Uptr new_table_base = 0;
	auto result = growTable(GLOBALS->TABLE_PTR, required_table_space, &new_table_base);
	if (result != GrowResult::success)
		throw MyException("Could not grow table by %u, error code: %i", required_table_space, static_cast<int>(result));

	// Temporary fix for https://github.com/WAVM/WAVM/issues/298
	for (auto i = new_table_base; i < new_table_base+required_table_space; ++i) {
		setTableElement(GLOBALS->TABLE_PTR, i, nullptr);
	}

	return new_table_base;
}

struct CustomResolver : Runtime::Resolver
{
	CustomResolver(Instance* host_module_instance, Runtime::Resolver& extendedResolver, Compartment* compartment, Context* context, I32* memory_base = nullptr, I32* table_base = nullptr, std::set<size_t>* dependencies = nullptr) :
	host_module_instance(host_module_instance), extendedResolver(extendedResolver), compartment(compartment), context(context), memory_base(memory_base), table_base(table_base), dependencies(dependencies) {}

	bool resolve(const std::string& moduleName,
					const std::string& exportName,
					IR::ExternType type,
					Runtime::Object*& outObject) override
	{
		// Importing host functions
		if (moduleName == "host" || exportName == "emscripten_notify_memory_growth") {
			outObject = getInstanceExport(host_module_instance, exportName);
			if (outObject) {
				if (!isA(outObject, type)) {
					// types did not match
					throw MyException("Import for %s::%s did not match the export type", moduleName.c_str(), exportName.c_str());
					return false;
				}
				return true;
			}
		}
		if (GLOBALS) { // Globals dont exist yet on instantiation of initial module.
			// Dynamic linking import
			if (moduleName == "env") {
				if (exportName == "__memory_base") {
					U32 required_memory = 1024; // TODO: Read the real value from dylink section
					auto new_memory_base = call_malloc(context, required_memory);
					if (memory_base)
						*memory_base = new_memory_base;
					Global* memory_base_global = Runtime::createGlobal(compartment, asGlobalType(type), std::string(exportName));
					Runtime::initializeGlobal(memory_base_global, new_memory_base);
					outObject = asObject(memory_base_global);
					if (!isA(outObject, type)) {
						// types did not match
						throw MyException("Import for %s::%s did not match the export type", moduleName.c_str(), exportName.c_str());
						return false;
					}
					return true;
				}
				if (exportName == "__table_base") {
					U32 required_table_space = 10; // TODO: Read the real value from dylink section
					Uptr new_table_base = AllocateTableElements(required_table_space);
					if (table_base)
						*table_base = new_table_base;

					GCPointer<Global> table_base_global = Runtime::createGlobal(compartment, asGlobalType(type), std::string(exportName));
					Runtime::initializeGlobal(table_base_global, static_cast<I32>(new_table_base)); // assuming 0 indexing
					outObject = asObject(table_base_global);
					if (!isA(outObject, type)) {
						// types did not match
						throw MyException("Import for %s::%s did not match the export type", moduleName.c_str(), exportName.c_str());
						return false;
					}
					return true;
				}
			}
			// Importing export of another module
			auto it = GLOBALS->ALL_EXPORTS.find(moduleName);
			if (it != GLOBALS->ALL_EXPORTS.end()) {
				auto it2 = it->second.second.find(exportName);
				if (it2 != it->second.second.end()) {
					outObject = it2->second;
					if (!isA(outObject, type)) {
						// types did not match
						throw MyException("Import for %s::%s did not match the export type", moduleName.c_str(), exportName.c_str());
						return false;
					}
					// increase load counter and insert into list of dependencies (basically dlopen and later index used for dlclose)
					if (dependencies && it->second.first >= 1) {
						GetInstanceData(it->second.first).load_count++;
						dependencies->insert(it->second.first);
					}
					return true;
				}
				return false;
			}
		}
		// WASI import
		return extendedResolver.resolve(moduleName, exportName, type, outObject);
	}
	
	Instance* host_module_instance;
	Runtime::Resolver& extendedResolver;
	Compartment* compartment;
	Context* context;
	I32* memory_base;
	I32* table_base;
	std::set<size_t>* dependencies;
};

int runtime_dynamic_link_module(const char* module_path, Compartment* compartment, Context* context, WASI::Process* process)
{
	ModuleRef module;
	{
		std::vector<U8> wasmBytes;
		std::string realpath = module_path;
		realpath += ".wasm";
		if(!readFile(realpath.c_str(), wasmBytes)) {
			throw MyException("Could not open file %s", realpath.c_str());
		}

		WASM::LoadError loadError;
		// ModuleRef module;
		if(!loadBinaryModule(wasmBytes.data(), wasmBytes.size(), module, FeatureLevel::mature, &loadError))
		{
			throw MyException("Couldn't load '%s': %s\n", module_path, loadError.message.c_str());
		}
	}

	// Link the WASM module with the WASI exports.
	auto& moduleIR = getModuleIR(module);
	I32 memory_base = -1;
	I32 table_base = -1;
	std::set<size_t> dependencies;
	CustomResolver resolver { GLOBALS->host_module_instance, getProcessResolver(*process), compartment, context, &memory_base, &table_base, &dependencies };
	LinkResult linkResult = linkModule(moduleIR, resolver);
	if(!linkResult.success)
	{
		char buff[ERROR_BUFFER_LENGTH];
		int written = snprintf(buff, ERROR_BUFFER_LENGTH, "Failed to link '%s':\n", module_path);
		for(const auto& missingImport : linkResult.missingImports)
		{
			if (written < ERROR_BUFFER_LENGTH-1)
			written += snprintf(buff+written, ERROR_BUFFER_LENGTH-written,
					"Failed to resolve import: type=%s module=%s export=%s\n",
					asString(missingImport.type).c_str(),
					missingImport.moduleName.c_str(),
					missingImport.exportName.c_str());
		}
		throw MyException(buff);
	}

	// Instantiate the linked module.
	GCPointer<Instance> instance = instantiateModule(compartment, module, std::move(linkResult.resolvedImports), std::string(module_path));
	Function* startFunction = getTypedInstanceExport(instance, "__post_instantiate", FunctionType());
	try {
		invokeFunction(context, startFunction, FunctionType());
	} catch (Runtime::Exception& ex) {
		throw MyException("Couldnt run the post instantiate function of %s", module_path);
	}

	// use preexisting vector slot
	for (size_t i = 0; i < GLOBALS->INSTANCES.size(); ++i) {
		auto& inst_data = GLOBALS->INSTANCES[i];
		if (inst_data.load_count < 1) {
			auto index = i+1;
			inst_data.name = module_path;
			inst_data.instance = instance;
			inst_data.memory_base = memory_base;
			inst_data.table_base = table_base;
			inst_data.load_count++;
			inst_data.dependency_indexes = std::move(dependencies);
			ExtractExports(moduleIR, instance, module_path, index);
			return index;
		}
	}

	// use new vector slot
	ExtractExports(moduleIR, instance, module_path, GLOBALS->INSTANCES.size()+1);
	GLOBALS->INSTANCES.emplace_back(module_path, instance, std::move(dependencies), memory_base, table_base);
	return GLOBALS->INSTANCES.size();
}

WAVM_DEFINE_INTRINSIC_MODULE(host);
WAVM_DEFINE_INTRINSIC_FUNCTION(host, "print_instance_info", void, print_instance_info) {
	// UNIMPLEMENTED
}
WAVM_DEFINE_INTRINSIC_FUNCTION(host, "fopen_and_read", U32, fopen_and_read, I32 filename, I32 length) {
	// UNIMPLEMENTED
}
WAVM_DEFINE_INTRINSIC_FUNCTION(host, "write_file", U32, write_file, I32 filename, I32 buffer, I32 length) {
	// UNIMPLEMENTED
}
WAVM_DEFINE_INTRINSIC_FUNCTION(host, "emscripten_notify_memory_growth", void, wasm_emscripten_notify_memory_growth, I32 index) {
	// UNIMPLEMENTED
}
WAVM_DEFINE_INTRINSIC_FUNCTION(host, "dlerror", I32, wasm_dlerror) {
	// UNIMPLEMENTED
	return 0;
}
WAVM_DEFINE_INTRINSIC_FUNCTION(host, "dlsym", I32, wasm_dlsym, I32 handle, I32 name_ptr) {
	// Context* context = getContextFromRuntimeData(contextRuntimeData);
	// Compartment* compartment = getCompartmentFromContextRuntimeData(contextRuntimeData);
	// Process* process = getProcessFromContextRuntimeData(contextRuntimeData);
	// Memory* memory = getProcessMemory(*process);
	Memory* memory = GLOBALS->MEMORY_PTR;

	try {
		auto& instancedata = GetInstanceData(handle);
		const char* name = get_c_string_from_wasm_memory(memory, name_ptr);

		// module was unloaded
		if (instancedata.load_count < 1) {
			throw MyException("dlsym: Handle %i was already unloaded, trying to link %s", handle, name);
		}

		// already loaded, return index
		auto it = instancedata.function_name_to_table_index.find(name);
		if (it != instancedata.function_name_to_table_index.end()) {
			return it->second;
		}

		// get the export
		auto* iExport = getInstanceExport(instancedata.instance, name);
		if (!iExport) {
			throw MyException("No export found %s::%s", instancedata.name.c_str(), name);
		}

		// add to table
		Uptr free_table_index = 0;
		while (free_table_index == 0) // Make sure return value cannot be 0 on success
			free_table_index = AllocateTableElements(1);

		setTableElement(GLOBALS->TABLE_PTR, free_table_index, iExport);
		
		instancedata.function_name_to_table_index[name] = free_table_index;
		return free_table_index;
	} catch (MyException& ex) {
		std::cout << ex.what() << std::endl;
	}
	return 0;
}
WAVM_DEFINE_INTRINSIC_FUNCTION(host, "dlopen", I32, wasm_dlopen, I32 name_ptr) {

	Context* context = getContextFromRuntimeData(contextRuntimeData);
	Compartment* compartment = getCompartmentFromContextRuntimeData(contextRuntimeData);
	Process* process = getProcessFromContextRuntimeData(contextRuntimeData);
	Memory* memory2 = getProcessMemory(*process);
	Memory* memory = GLOBALS->MEMORY_PTR;

	try {
		const char* name = get_c_string_from_wasm_memory(memory, name_ptr);
		for (size_t i = 0; i < GLOBALS->INSTANCES.size(); ++i) {
			auto& v = GLOBALS->INSTANCES[i];
			if (v.name == name && v.load_count > 0) {
				v.load_count++;
				return i+1;
			}
		}
		auto res = runtime_dynamic_link_module(name, compartment, context, process);
		// collectCompartmentGarbage(getCompartmentFromContextRuntimeData(contextRuntimeData));
		return res;
	} catch (MyException& ex) {
		std::cout << ex.what() << std::endl;
	}
	return 0;
}
I32 UnloadModuleInstance(Context* context, Compartment* compartment, I32 handle, bool collect_garbage)
{
	auto& instancedata = GetInstanceData(handle);

	if (instancedata.load_count > 1) {
		instancedata.load_count--;
		return 1; // others still referring
	}
	if (instancedata.load_count == 1) {
		// free dlsym indexes
		for (auto& it : instancedata.function_name_to_table_index) {
			setTableElement(GLOBALS->TABLE_PTR, it.second, nullptr);
			GLOBALS->FREE_TABLE_SIZE_TO_BASE.insert({1, it.second});
		}
		instancedata.function_name_to_table_index.clear();

		if (instancedata.memory_base != -1)
		{
			// free memory space
			UntaggedValue args[1] = {instancedata.memory_base};
			invokeFunction(context, GLOBALS->FREE_PTR, GLOBALS->FREE_SIGNATURE, args);
			instancedata.memory_base = -1;
		}
		if (instancedata.table_base != -1)
		{
			// free table space
			Uptr required_table_space = 10; // TODO: hardcoded value for table space, should get from dylink section
			for (Uptr i = 0; i < required_table_space; ++i) 
				setTableElement(GLOBALS->TABLE_PTR, instancedata.table_base+i, nullptr);
			GLOBALS->FREE_TABLE_SIZE_TO_BASE.insert({required_table_space, instancedata.table_base});
			instancedata.table_base = -1;
		}

		// remove exposed exports
		GLOBALS->ALL_EXPORTS.erase(instancedata.name);

		// free the instance
		instancedata.name.clear();
		instancedata.load_count = 0; // set count to 0
		instancedata.instance = nullptr; // remove instance (GC root)

		// free any import dependencies id should
		for (auto dep_handle : instancedata.dependency_indexes) {
			UnloadModuleInstance(context, compartment, dep_handle, false);
		}
		instancedata.dependency_indexes.clear();

		if (collect_garbage)
			collectCompartmentGarbage(compartment); // run GC, there shouldnt be any more references to the instance
		return 1; // unload successful
	}
	if (instancedata.load_count < 1) {
		throw MyException("dlclose: module with given handle %i is already unloaded", handle);
	}
	return 0;
}
WAVM_DEFINE_INTRINSIC_FUNCTION(host, "dlclose", I32, wasm_dlclose, I32 handle) {
	Process* process = getProcessFromContextRuntimeData(contextRuntimeData);
	Context* context = getContextFromRuntimeData(contextRuntimeData);
	Memory* memory = getProcessMemory(*process);
	Compartment* compartment = getCompartmentFromContextRuntimeData(contextRuntimeData);

	try {
		return UnloadModuleInstance(context, compartment, handle, true);
	} catch (MyException& ex) {
		std::cout << ex.what() << std::endl;
	}
	return 0;
}

int main(int argc, char** argv)
{
	if(argc < 2)
	{
		fprintf(stderr, "Usage: %s <path to WASM binary> [WASI arguments]\n", argv[0]);
		return EXIT_FAILURE;
	}

	// Load the WASM file.
	std::vector<U8> wasmBytes;
	if(!readFile(argv[1], wasmBytes)) { return EXIT_FAILURE; }

	WASM::LoadError loadError;
	ModuleRef module;
	if(!loadBinaryModule(
		wasmBytes.data(), wasmBytes.size(), module, FeatureLevel::mature, &loadError))
	{
		fprintf(stderr, "Couldn't load '%s': %s\n", argv[1], loadError.message.c_str());
		return EXIT_FAILURE;
	}

	// Create a WAVM compartment and context.
	GCPointer<Compartment> compartment = createCompartment();
	{
		GCPointer<Context> context = createContext(compartment);

		// Create host module instance
		GCPointer<Instance> host_module_instance = Intrinsics::instantiateModule(compartment,
			{
				WAVM_INTRINSIC_MODULE_REF(host)
				// could list more modules here, they will be bundled to one
			},
			"host"
		);

		// Create the WASI process.
		std::vector<std::string> wasiArgs;
		for(int argIndex = 1; argIndex < argc; ++argIndex) { wasiArgs.push_back(argv[argIndex]); }

		std::shared_ptr<VFS::FileSystem> sandboxFS
			= VFS::makeSandboxFS(&Platform::getHostFS(), Platform::getCurrentWorkingDirectory());

		std::shared_ptr<WASI::Process> process
			= createProcess(compartment,
							std::move(wasiArgs),
							{},
							sandboxFS.get(),
							Platform::getStdFD(Platform::StdDevice::in),
							Platform::getStdFD(Platform::StdDevice::out),
							Platform::getStdFD(Platform::StdDevice::err));

		// Link the WASM module with the WASI exports.
		CustomResolver resolver { host_module_instance, getProcessResolver(*process), compartment, context };
		auto& moduleIR = getModuleIR(module);
		LinkResult linkResult = linkModule(moduleIR, resolver);
		if(!linkResult.success)
		{
			fprintf(stderr, "Failed to link '%s':\n", argv[1]);
			for(const auto& missingImport : linkResult.missingImports)
			{
				fprintf(stderr,
						"Failed to resolve import: type=%s module=%s export=%s\n",
						asString(missingImport.type).c_str(),
						missingImport.moduleName.c_str(),
						missingImport.exportName.c_str());
			}
			return EXIT_FAILURE;
		}

		// TODO: Support importing of table or memory from host

		// Instantiate the main module.
		GCPointer<Instance> instance = instantiateModule(compartment, module, std::move(linkResult.resolvedImports), std::string(argv[1]));

		GLOBALS = std::make_unique<Globals>();
		GLOBALS->host_module_instance = host_module_instance;
		GLOBALS->MALLOC_PTR = getTypedInstanceExport(instance, "malloc", GLOBALS->MALLOC_SIGNATURE);
		GLOBALS->FREE_PTR = getTypedInstanceExport(instance, "free", GLOBALS->FREE_SIGNATURE);
		GLOBALS->MEMORY_PTR = asMemory(getInstanceExport(instance, "memory"));

		auto table = getInstanceExport(instance, "table");
		if (!table)
			table = getInstanceExport(instance, "__indirect_function_table");
		GLOBALS->TABLE_PTR = asTable(table);

		// Link WASI with the memory exported by the WASM module.
		setProcessMemory(*process, GLOBALS->MEMORY_PTR);

		ExtractExports(moduleIR, instance, argv[1], 0);

		// make sure main module's exports area also found under "env" namespace as linked modules use env to import WASI functions from main module
		for (auto& p : GLOBALS->ALL_EXPORTS[argv[1]].second) {
			GLOBALS->ALL_EXPORTS["env"].second[p.first] = p.second;
		}
		GLOBALS->ALL_EXPORTS["env"].second["table"] = table; // make sure env::table point to the table regardless of real export name

		// Call the WASM module's "_start" function, using WASI::catchExit to handle non-local returns
		// via the WASI exit API.
		FunctionType main_signature;
		GCPointer<Function> startFunction = getTypedInstanceExport(instance, "_start", main_signature);
		// FunctionType main_signature({ValueType::i32}, {});
		// Function* startFunction = getTypedInstanceExport(instance, "__original_main", main_signature);
		const I32 exitCode = WASI::catchExit([&]() -> I32 {
			invokeFunction(context, startFunction, main_signature);
			return 0;
		});

		// Clean up the WAVM runtime objects.
		GLOBALS.reset(nullptr);
		// process.reset();
		// instance = nullptr;
		// host_module_instance = nullptr;
	}
	WAVM_ERROR_UNLESS(tryCollectCompartment(std::move(compartment)));

	// return exitCode;
	return 0;
}
