# Migrating ClangJIT to Clang 17

This is an attempt at upgrading the original, Clang 9 based [ClangJIT project](https://github.com/hfinkel/llvm-project-cxxjit) to Clang 17.

## Current Status

### Things that work:
- Clang builds
- If JIT mode is enabled manually (see issues):
	- Dynamic template arguments are parsed correctly
	- AOT IR is emitted seemingly correctly, including `__clang_jit` calls and embedded AST
    - Simple JITed test code runs

### Issues:
- The`-fjit` option is currently not always passed correctly to the LangOpt parsing. Current workaround is to always operate in JIT mode (see `CompilerInvocation.cpp:4040`).
~~- Linking of required Clang libs for JIT runtime fails due to loads of `undefined reference` errors~~ (solved by adding `lclangSupport`) 
- When finalizing the JIT calls for IR module generation (`CodeGenAction.cpp:378`), local symbols are collected using an AST visitor. Calling this visitor currently results in a crash.  

### TODOs:
JIT runtime:
- The JIT runtime originally used ORC v1, which has since been replaced by ORC v2. ClangJIT requires a specific symbol resolution scheme that I am not sure how to implement using the v2 API. For now, the standard symbol lookup is used, which may not work properly (see `JIT.cpp`).
- Static ctor/dtor runnners are not implemented

CUDA support:
~~- For JIT device kernels, calls to `__clang_jit` require the number of available devices. This was previously identified using the parameter's pointer type and does not work anymore due to opaque pointers. For now, a placeholder `0` is inserted. (`BackendUtil.cpp:780`:)~~

