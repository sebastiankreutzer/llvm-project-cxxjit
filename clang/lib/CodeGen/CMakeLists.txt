set(LLVM_LINK_COMPONENTS
  Analysis
  BitReader
  BitWriter
  Core
  CodeGen
  Coroutines
  Coverage
  IPO
  IRReader
  AggressiveInstCombine
  InstCombine
  Instrumentation
  LTO
  Linker
  MC
  ObjCARCOpts
  Object
  Option
  OrcJIT
  OrcError
  Passes
  ProfileData
  RuntimeDyld
  Remarks
  ScalarOpts
  Support
  Target
  TransformUtils
  Vectorize
  ExecutionEngine
  Interpreter
  ${LLVM_TARGETS_TO_BUILD}
  )
# Note that above we depend on ${LLVM_TARGETS_TO_BUILD} instead of just native
# because we might need NVPTX, etc.

# In a standard Clang+LLVM build, we need to generate intrinsics before
# building codegen. In a standalone build, LLVM is already built and we don't
# need this dependency. Furthermore, LLVM doesn't export it so we can't have
# this dependency.
set(codegen_deps intrinsics_gen)
if (CLANG_BUILT_STANDALONE)
  set(codegen_deps)
endif()

if (MSVC)
  set_source_files_properties(CodeGenModule.cpp PROPERTIES COMPILE_FLAGS /bigobj)
endif()

add_clang_library(clangCodeGen
  BackendUtil.cpp
  CGAtomic.cpp
  CGBlocks.cpp
  CGBuiltin.cpp
  CGCUDANV.cpp
  CGCUDARuntime.cpp
  CGCXX.cpp
  CGCXXABI.cpp
  CGCall.cpp
  CGClass.cpp
  CGCleanup.cpp
  CGCoroutine.cpp
  CGDebugInfo.cpp
  CGDecl.cpp
  CGDeclCXX.cpp
  CGException.cpp
  CGExpr.cpp
  CGExprAgg.cpp
  CGExprCXX.cpp
  CGExprComplex.cpp
  CGExprConstant.cpp
  CGExprScalar.cpp
  CGGPUBuiltin.cpp
  CGLoopInfo.cpp
  CGNonTrivialStruct.cpp
  CGObjC.cpp
  CGObjCGNU.cpp
  CGObjCMac.cpp
  CGObjCRuntime.cpp
  CGOpenCLRuntime.cpp
  CGOpenMPRuntime.cpp
  CGOpenMPRuntimeNVPTX.cpp
  CGRecordLayoutBuilder.cpp
  CGStmt.cpp
  CGStmtOpenMP.cpp
  CGVTT.cpp
  CGVTables.cpp
  CodeGenABITypes.cpp
  CodeGenAction.cpp
  CodeGenFunction.cpp
  CodeGenModule.cpp
  CodeGenPGO.cpp
  CodeGenTBAA.cpp
  CodeGenTypes.cpp
  ConstantInitBuilder.cpp
  CoverageMappingGen.cpp
  ItaniumCXXABI.cpp
  MacroPPCallbacks.cpp
  MicrosoftCXXABI.cpp
  ModuleBuilder.cpp
  ObjectFilePCHContainerOperations.cpp
  PatternInit.cpp
  SanitizerMetadata.cpp
  SwiftCallingConv.cpp
  TargetInfo.cpp
  VarBypassDetector.cpp
  JIT.cpp

  #Tuner/StaticOptimizer.cpp
  Tuner/Tuner.cpp
  #Tuner/LoopKnobCreator.cpp
  #Tuner/KnobSet.cpp
  #Tuner/LoopKnob.cpp
  Tuner/TimingHelper.cpp
  Tuner/SimplexTuner.cpp
  Tuner/ModifiedSimplexTuner.cpp
  Tuner/TunerDriver.cpp
  Tuner/LoopTransformTreeCreator.cpp
  Tuner/LoopTransformTreeApplicator.cpp
  Tuner/LoopTransformTree.cpp
  Tuner/Transformations.cpp
  Tuner/TransformTreeOptimizer.cpp
  Tuner/Search.cpp
  Driver.cpp

  DEPENDS
  ${codegen_deps}

  LINK_LIBS
  clangAnalysis
  clangAST
  clangASTMatchers
  clangBasic
  clangDriver
  clangFrontend
  clangLex
  clangParse
  clangSema
  clangSerialization
  )

if ("polly" IN_LIST LLVM_ENABLE_PROJECTS)
  option(CLANG_JIT_POLLY "Enable polly for JIT tuning." OFF)
  if (CLANG_JIT_POLLY)

    set(Polly_DIR "${LLVM_BINARY_DIR}/lib/cmake/polly")
    message(STATUS "Using PollyConfig.cmake in ${Polly_DIR}")
    find_package(Polly REQUIRED CONFIG)

    include_directories(${Polly_INCLUDE_DIRS})
    add_definitions(${Polly_DEFINITIONS})
    add_definitions("-DPOLLY_TUNER")

    target_link_libraries(clangCodeGen PUBLIC Polly)
  endif()
endif()