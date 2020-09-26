//
// Created by sebastian on 07.11.19.
//

#ifndef CLANG_JIT_COMPILER_H
#define CLANG_JIT_COMPILER_H

#include "Tuner/Tuner.h"
#include "Tuner/Debug.h"
#include "Tuner/TimingHelper.h"
#include "clang/CodeGen/CodeGenAction.h"
#include "CodeGenModule.h"
#include "CoverageMappingGen.h"
#include "CGCXXABI.h"
#include "MacroPPCallbacks.h"
#include "clang/AST/ASTConsumer.h"
#include "clang/AST/ASTContext.h"
#include "clang/AST/DeclCXX.h"
#include "clang/AST/DeclGroup.h"
#include "clang/AST/RecursiveASTVisitor.h"
#include "clang/Basic/Diagnostic.h"
#include "clang/Basic/DiagnosticIDs.h"
#include "clang/Basic/DiagnosticOptions.h"
#include "clang/Basic/FileManager.h"
#include "clang/Basic/FileSystemOptions.h"
#include "clang/Basic/LLVM.h"
#include "clang/Basic/SourceManager.h"
#include "clang/Basic/TargetInfo.h"
#include "clang/Basic/TargetOptions.h"
#include "clang/CodeGen/BackendUtil.h"
#include "clang/CodeGen/ModuleBuilder.h"
#include "clang/Driver/Compilation.h"
#include "clang/Driver/Driver.h"
#include "clang/Driver/Job.h"
#include "clang/Driver/Options.h"
#include "clang/Driver/Tool.h"
#include "clang/Driver/ToolChain.h"
#include "clang/Frontend/ASTUnit.h"
#include "clang/Frontend/CompilerInstance.h"
#include "clang/Frontend/CompilerInvocation.h"
#include "clang/Frontend/FrontendDiagnostic.h"
#include "clang/Frontend/FrontendOptions.h"
#include "clang/Frontend/TextDiagnosticPrinter.h"
#include "clang/Lex/HeaderSearch.h"
#include "clang/Lex/HeaderSearchOptions.h"
#include "clang/Lex/Preprocessor.h"
#include "clang/Lex/PreprocessorOptions.h"
#include "clang/Parse/Parser.h"
#include "clang/Sema/Sema.h"
#include "clang/Sema/Template.h"
#include "clang/Sema/TemplateDeduction.h"
#include "clang/Serialization/ASTReader.h"
#include "clang/Serialization/InMemoryModuleCache.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/IntrusiveRefCntPtr.h"
#include "llvm/ADT/SmallString.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/Twine.h"
#include "llvm/Bitcode/BitcodeReader.h"
#include "llvm/CodeGen/MachineOptimizationRemarkEmitter.h"
#include "llvm/ExecutionEngine/ExecutionEngine.h"
#include "llvm/ExecutionEngine/JITSymbol.h"
#include "llvm/ExecutionEngine/Orc/CompileUtils.h"
#include "llvm/ExecutionEngine/Orc/ExecutionUtils.h"
#include "llvm/ExecutionEngine/Orc/IRCompileLayer.h"
#include "llvm/ExecutionEngine/Orc/LambdaResolver.h"
#include "llvm/ExecutionEngine/Orc/RTDyldObjectLinkingLayer.h"
#include "llvm/ExecutionEngine/RTDyldMemoryManager.h"
#include "llvm/ExecutionEngine/SectionMemoryManager.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/IR/DebugInfo.h"
#include "llvm/IR/DiagnosticInfo.h"
#include "llvm/IR/DiagnosticPrinter.h"
#include "llvm/IR/GlobalValue.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/IR/Mangler.h"
#include "llvm/IR/Module.h"
#include "llvm/IRReader/IRReader.h"
#include "llvm/Linker/Linker.h"
#include "llvm/Option/ArgList.h"
#include "llvm/Option/OptTable.h"
#include "llvm/Option/Option.h"
#include "llvm/Pass.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/DynamicLibrary.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/Host.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/MemoryBuffer.h"
#include "llvm/Support/Path.h"
#include "llvm/Support/VirtualFileSystem.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/SourceMgr.h"
#include "llvm/Support/TargetSelect.h"
#include "llvm/Support/Timer.h"
#include "llvm/Support/ToolOutputFile.h"
#include "llvm/Support/YAMLTraits.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Transforms/Utils/Cloning.h"
#include "llvm/Transforms/IPO/Internalize.h"
#include "llvm/Support/FormatVariadic.h"
#include "llvm/Support/FormatAdapters.h"

#include <cassert>
#include <cstdlib> // ::getenv
#include <cstring>
#include <memory>
#include <string>
#include <sstream>
#include <system_error>
#include <utility>
#include <vector>
#include <iomanip>
#include <unordered_map>



namespace clang {
namespace jit {

using namespace llvm;

inline void fatal() {
  report_fatal_error("Clang JIT failed!");
}


// This is a variant of ORC's LegacyLookupFnResolver with a cutomized
// getResponsibilitySet behavior allowing us to claim responsibility for weak
// symbols in the loaded modules that we don't otherwise have.
// Note: We generally convert all IR level symbols to have strong linkage, but
// that won't cover everything (and especially doesn't cover the DW.ref.
// symbols created by the low-level EH logic on some platforms).
template <typename LegacyLookupFn>
class ClangLookupFnResolver final : public llvm::orc::SymbolResolver {
public:
  using ErrorReporter = std::function<void(Error)>;

  ClangLookupFnResolver(llvm::orc::ExecutionSession &ES,
                        LegacyLookupFn LegacyLookup,
                        ErrorReporter ReportError)
      : ES(ES), LegacyLookup(std::move(LegacyLookup)),
        ReportError(std::move(ReportError)) {}

  llvm::orc::SymbolNameSet
  getResponsibilitySet(const llvm::orc::SymbolNameSet &Symbols) final {
    llvm::orc::SymbolNameSet Result;

    for (auto &S : Symbols) {
      if (JITSymbol Sym = LegacyLookup(*S)) {
        // If the symbol exists elsewhere, and we have only a weak version,
        // then we're not responsible.
        continue;
      } else if (auto Err = Sym.takeError()) {
        ReportError(std::move(Err));
        return llvm::orc::SymbolNameSet();
      } else {
        Result.insert(S);
      }
    }

    return Result;
  }

  llvm::orc::SymbolNameSet
  lookup(std::shared_ptr<llvm::orc::AsynchronousSymbolQuery> Query,
         llvm::orc::SymbolNameSet Symbols) final {
    return llvm::orc::lookupWithLegacyFn(ES, *Query, Symbols, LegacyLookup);
  }

private:
  llvm::orc::ExecutionSession &ES;
  LegacyLookupFn LegacyLookup;
  ErrorReporter ReportError;
};

template <typename LegacyLookupFn>
std::shared_ptr<ClangLookupFnResolver<LegacyLookupFn>>
createClangLookupResolver(llvm::orc::ExecutionSession &ES,
                          LegacyLookupFn LegacyLookup,
                          std::function<void(Error)> ErrorReporter) {
  return std::make_shared<ClangLookupFnResolver<LegacyLookupFn>>(
      ES, std::move(LegacyLookup), std::move(ErrorReporter));
}



class ClangJIT {
public:
  using ObjLayerT = llvm::orc::LegacyRTDyldObjectLinkingLayer;
  using CompileLayerT = llvm::orc::LegacyIRCompileLayer<ObjLayerT, llvm::orc::SimpleCompiler>;

  ClangJIT(DenseMap<StringRef, const void *> &LocalSymAddrs);

  ~ClangJIT() {
    // Run any destructors registered with __cxa_atexit.
    CXXRuntimeOverrides.runDestructors();

    // Run any IR destructors.
    for (auto &DtorRunner : IRStaticDestructorRunners)
      cantFail(DtorRunner.runViaLayer(CompileLayer));
  }

  llvm::TargetMachine &getTargetMachine() { return *TM; }

  llvm::orc::VModuleKey addModule(std::unique_ptr<llvm::Module> M);

  void removeModule(llvm::orc::VModuleKey K);

  llvm::JITSymbol findSymbol(const std::string Name) {
    return findMangledSymbol(mangle(Name));
  }

private:
  std::string mangle(const std::string &Name) {
    std::string MangledName;
    {
      llvm::raw_string_ostream MangledNameStream(MangledName);
      llvm::Mangler::getNameWithPrefix(MangledNameStream, Name, DL);
    }
    return MangledName;
  }

  llvm::JITSymbol findMangledSymbol(const std::string &Name) {
    for (auto H : make_range(ModuleKeys.rbegin(), ModuleKeys.rend()))
      if (auto Sym = CompileLayer.findSymbolIn(H, Name,
          /*ExportedSymbolsOnly*/ false))
        return Sym;

    if (auto Sym = CXXRuntimeOverrides.searchOverrides(Name))
      return Sym;

    auto LSAI = LocalSymAddrs.find(Name);
    if (LSAI != LocalSymAddrs.end())
      return llvm::JITSymbol(llvm::pointerToJITTargetAddress(LSAI->second),
                             llvm::JITSymbolFlags::Exported);

    // If we can't find the symbol in the JIT, try looking in the host process.
    if (auto SymAddr = RTDyldMemoryManager::getSymbolAddressInProcess(Name))
      return llvm::JITSymbol(SymAddr, llvm::JITSymbolFlags::Exported);

#ifdef _WIN32
    // For Windows retry without "_" at beginning, as RTDyldMemoryManager uses
    // GetProcAddress and standard libraries like msvcrt.dll use names
    // with and without "_" (for example "_itoa" but "sin").
    if (Name.length() > 2 && Name[0] == '_')
      if (auto SymAddr =
              RTDyldMemoryManager::getSymbolAddressInProcess(Name.substr(1)))
        return llvm::JITSymbol(SymAddr, llvm::JITSymbolFlags::Exported);
#endif

    return nullptr;
  }

  DenseMap<StringRef, const void *> &LocalSymAddrs;
  llvm::orc::ExecutionSession ES;
  std::shared_ptr<llvm::orc::SymbolResolver> Resolver;
  std::unique_ptr<llvm::TargetMachine> TM;
  const llvm::DataLayout DL;
  ObjLayerT ObjectLayer;
  CompileLayerT CompileLayer;
  std::vector<llvm::orc::VModuleKey> ModuleKeys;

  llvm::orc::LegacyLocalCXXRuntimeOverrides CXXRuntimeOverrides;
  std::vector<llvm::orc::LegacyCtorDtorRunner<CompileLayerT>>
      IRStaticDestructorRunners;
};



struct DevFileData {
  const char *Filename;
  const void *Data;
  size_t DataSize;
};

struct DevData {
  const char *Triple;
  const char *Arch;
  const char *ASTBuffer;
  size_t ASTBufferSize;
  const void *CmdArgs;
  size_t CmdArgsLen;
  DevFileData *FileData;
  size_t FileDataCnt;
};

class BackendConsumer : public ASTConsumer {
  DiagnosticsEngine &Diags;
  BackendAction Action;
  const HeaderSearchOptions &HeaderSearchOpts;
  const CodeGenOptions &CodeGenOpts;
  const clang::TargetOptions &TargetOpts;
  const LangOptions &LangOpts;
  std::unique_ptr<raw_pwrite_stream> AsmOutStream;
  ASTContext *Context;
  std::string InFile;
  const PreprocessorOptions &PPOpts;
  LLVMContext &C;
  std::vector<std::unique_ptr<llvm::Module>> &DevLinkMods;
  CoverageSourceInfo *CoverageInfo;

  std::unique_ptr<CodeGenerator> Gen;

  void replaceGenerator() {
    Gen.reset(CreateLLVMCodeGen(Diags, InFile, HeaderSearchOpts, PPOpts,
                                CodeGenOpts, C, CoverageInfo));
  }

public:
  BackendConsumer(BackendAction Action, DiagnosticsEngine &Diags,
                  const HeaderSearchOptions &HeaderSearchOpts,
                  const PreprocessorOptions &PPOpts,
                  const CodeGenOptions &CodeGenOpts,
                  const clang::TargetOptions &TargetOpts,
                  const LangOptions &LangOpts, bool TimePasses,
                  const std::string &InFile,
                  std::unique_ptr<raw_pwrite_stream> OS, LLVMContext &C,
                  std::vector<std::unique_ptr<llvm::Module>> &DevLinkMods,
                  CoverageSourceInfo *CoverageInfo = nullptr)
      : Diags(Diags), Action(Action), HeaderSearchOpts(HeaderSearchOpts),
        CodeGenOpts(CodeGenOpts), TargetOpts(TargetOpts), LangOpts(LangOpts),
        AsmOutStream(std::move(OS)), Context(nullptr), InFile(InFile),
        PPOpts(PPOpts), C(C), DevLinkMods(DevLinkMods),
        CoverageInfo(CoverageInfo) { }

  llvm::Module *getModule() const { return Gen->GetModule(); }
  std::unique_ptr<llvm::Module> takeModule() {
    return std::unique_ptr<llvm::Module>(Gen->ReleaseModule());
  }

  CodeGenerator *getCodeGenerator() { return Gen.get(); }

  void HandleCXXStaticMemberVarInstantiation(VarDecl *VD) override {
    Gen->HandleCXXStaticMemberVarInstantiation(VD);
  }

  void Initialize(ASTContext &Ctx) override {
    replaceGenerator();
    Context = &Ctx;
    Gen->Initialize(Ctx);
  }

  bool HandleTopLevelDecl(DeclGroupRef D) override {
    Gen->HandleTopLevelDecl(D);
    return true;
  }

  void HandleInlineFunctionDefinition(FunctionDecl *D) override {
    Gen->HandleInlineFunctionDefinition(D);
  }

  void HandleInterestingDecl(DeclGroupRef D) override {
    HandleTopLevelDecl(D);
  }

  void HandleTranslationUnit(ASTContext &C) override;

  void HandleTagDeclDefinition(TagDecl *D) override {
    Gen->HandleTagDeclDefinition(D);
  }

  void HandleTagDeclRequiredDefinition(const TagDecl *D) override {
    Gen->HandleTagDeclRequiredDefinition(D);
  }

  void CompleteTentativeDefinition(VarDecl *D) override {
    Gen->CompleteTentativeDefinition(D);
  }

  void AssignInheritanceModel(CXXRecordDecl *RD) override {
    Gen->AssignInheritanceModel(RD);
  }

  void HandleVTable(CXXRecordDecl *RD) override {
    Gen->HandleVTable(RD);
  }

  void EmitOptimized() {
    EmitBackendOutput(Diags, HeaderSearchOpts, CodeGenOpts, TargetOpts,
                      LangOpts, Context->getTargetInfo().getDataLayout(),
                      getModule(), Action,
                      std::make_unique<llvm::buffer_ostream>(*AsmOutStream));
  }
};


struct CompilerData {
  std::unique_ptr<CompilerInvocation>     Invocation;
  IntrusiveRefCntPtr<DiagnosticOptions>   DiagOpts;
  std::unique_ptr<TextDiagnosticPrinter>  DiagnosticPrinter;
  llvm::IntrusiveRefCntPtr<llvm::vfs::InMemoryFileSystem> InMemoryFileSystem;
  IntrusiveRefCntPtr<DiagnosticsEngine>   Diagnostics;
  IntrusiveRefCntPtr<FileManager>         FileMgr;
  IntrusiveRefCntPtr<SourceManager>       SourceMgr;
  IntrusiveRefCntPtr<InMemoryModuleCache>   ModuleCache;
  std::unique_ptr<HeaderSearch>           HeaderInfo;
  std::unique_ptr<PCHContainerReader>     PCHContainerRdr;
  IntrusiveRefCntPtr<TargetInfo>          Target;
  std::shared_ptr<Preprocessor>           PP;
  IntrusiveRefCntPtr<ASTContext>          Ctx;
  std::shared_ptr<clang::TargetOptions>   TargetOpts;
  std::shared_ptr<HeaderSearchOptions>    HSOpts;
  std::shared_ptr<PreprocessorOptions>    PPOpts;
  IntrusiveRefCntPtr<ASTReader>           Reader;
  std::unique_ptr<BackendConsumer>        Consumer;
  std::unique_ptr<Sema>                   S;
  TrivialModuleLoader                     ModuleLoader;
  std::unique_ptr<llvm::Module>           RunningMod;

  llvm::DenseMap<StringRef, const void *>       LocalSymAddrs;
  llvm::DenseMap<StringRef, ValueDecl *>        NewLocalSymDecls;
  std::unique_ptr<ClangJIT>               CJ;

  llvm::DenseMap<unsigned, FunctionDecl *>      FuncMap;

  // A map of each instantiation to the containing function. These might not be
  // unique, but should be unique for any place where it matters
  // (instantiations with from-string types).
  llvm::DenseMap<unsigned, FunctionDecl *>      CSFuncMap;

  std::unique_ptr<CompilerData>           DevCD;
  SmallString<1>                          DevAsm;
  std::vector<std::unique_ptr<llvm::Module>> DevLinkMods;


  CompilerData(const void *CmdArgs, unsigned CmdArgsLen,
               const void *ASTBuffer, size_t ASTBufferSize,
               const void *IRBuffer, size_t IRBufferSize,
               const void **LocalPtrs, unsigned LocalPtrsCnt,
               const void **LocalDbgPtrs, unsigned LocalDbgPtrsCnt,
               const DevData *DeviceData, unsigned DevCnt,
               int ForDev = -1);


  void restoreFuncDeclContext(FunctionDecl *FunD);

//  std::string instantiateTemplate(const void *NTTPValues, const char **TypeStrings,
//                                  unsigned Idx);

  void emitAllNeeded(bool CheckExisting = true);

  std::unique_ptr<llvm::Module> createModule(StringRef Name);

  using ModulePair = std::pair<std::unique_ptr<Module>, std::unique_ptr<Module>>;

  // Creates IR modules for host and device.
  ModulePair createModules(StringRef Name);

  void embedPTXFatBin(llvm::Module& HostMod, llvm::Module& DevMod);

  void linkInAvailableDefs(llvm::Module& Mod, bool InvokeDefaultOpt = false);

  void makeDefsAvailable(std::unique_ptr<llvm::Module> NewMod);


//  Function* instrumentFunction(Function* F/*, StatsTracker& Tracker, JITContext::VersionID ID*/) {
//    auto* Gen = Consumer->getCodeGenerator();
//
//    auto *FnTy =
//        llvm::FunctionType::get(Gen->CGM().VoidTy, {Gen->CGM().VoidPtrTy, Gen->CGM().Int32Ty, Gen->CGM().Int32Ty, Gen->CGM().DoubleTy, Gen->CGM().DoubleTy, Gen->CGM().Int64Ty}/*TypeParams*/,
//            /*isVarArg*/ false);
//
//    auto CBFn = Gen->CGM().CreateRuntimeFunction(FnTy, "__clang_jit_update_stats");
//
//    return PerfMonitor->createInstrumentedWrapper(F/*, dyn_cast<Function>(CBFn.getCallee()), Tracker, ID*/);
//  }

//  TunedCodeVersion recompileFunction(const void *NTTPValues, const char **TypeStrings,
//                                     unsigned Idx, JITContext& JITCtx, JITContext::VersionID ID);


//  TunedCodeVersion resolveFunction(const void *NTTPValues, const char **TypeStrings,
//                                   unsigned Idx, JITContext& JITCtx, JITContext::VersionID ID);

private:

  void prepareForLinking(llvm::Module* DstMod, const llvm::Module* SrcMod);

//  TunedCodeVersion finalizeModule(std::unique_ptr<llvm::Module> Mod, JITContext& JITCtx, JITContext::VersionID ID);
};

struct InstInfo {
  InstInfo(const char *InstKey, const void *NTTPValues,
           unsigned NTTPValuesSize, const char **TypeStrings,
           unsigned TypeStringsCnt)
      : Key(InstKey),
        NTArgs(StringRef((const char *) NTTPValues, NTTPValuesSize)) {
    for (unsigned i = 0, e = TypeStringsCnt; i != e; ++i)
      TArgs.push_back(StringRef(TypeStrings[i]));
  }

  InstInfo(const StringRef &R) : Key(R) { }

  // The instantiation key (these are always constants, so we don't need to
  // allocate storage for them).
  StringRef Key;

  // The buffer of non-type arguments (this is packed).
  SmallString<16> NTArgs;

  // Vector of string type names.
  SmallVector<SmallString<32>, 1> TArgs;
};

struct InstData {
  void* FPtr;
  bool UseFastLookup;
};

struct ThisInstInfo {
  ThisInstInfo(const char *InstKey, const void *NTTPValues,
               unsigned NTTPValuesSize, const char **TypeStrings,
               unsigned TypeStringsCnt)
      : InstKey(InstKey), NTTPValues(NTTPValues), NTTPValuesSize(NTTPValuesSize),
        TypeStrings(TypeStrings), TypeStringsCnt(TypeStringsCnt) { }

  const char *InstKey;

  const void *NTTPValues;
  unsigned NTTPValuesSize;

  const char **TypeStrings;
  unsigned TypeStringsCnt;
};


struct InstMapInfo {
  static inline InstInfo getEmptyKey() {
    return InstInfo(DenseMapInfo<StringRef>::getEmptyKey());
  }

  static inline InstInfo getTombstoneKey() {
    return InstInfo(DenseMapInfo<StringRef>::getTombstoneKey());
  }

  static unsigned getHashValue(const InstInfo &II) {
    using llvm::hash_code;
    using llvm::hash_combine;
    using llvm::hash_combine_range;

    hash_code h = hash_combine_range(II.Key.begin(), II.Key.end());
    h = hash_combine(h, hash_combine_range(II.NTArgs.begin(),
                                           II.NTArgs.end()));
    for (auto &TA : II.TArgs)
      h = hash_combine(h, hash_combine_range(TA.begin(), TA.end()));

    return (unsigned) h;
  }

  static unsigned getHashValue(const ThisInstInfo &TII) {
    using llvm::hash_code;
    using llvm::hash_combine;
    using llvm::hash_combine_range;

    hash_code h =
        hash_combine_range(TII.InstKey, TII.InstKey + std::strlen(TII.InstKey));
    h = hash_combine(h, hash_combine_range((const char *) TII.NTTPValues,
                                           ((const char *) TII.NTTPValues) +
                                           TII.NTTPValuesSize));
    for (unsigned int i = 0, e = TII.TypeStringsCnt; i != e; ++i)
      h = hash_combine(h,
                       hash_combine_range(TII.TypeStrings[i],
                                          TII.TypeStrings[i] +
                                          std::strlen(TII.TypeStrings[i])));

    return (unsigned) h;
  }

  static bool isEqual(const InstInfo &LHS, const InstInfo &RHS) {
    return LHS.Key    == RHS.Key &&
           LHS.NTArgs == RHS.NTArgs &&
           LHS.TArgs  == RHS.TArgs;
  }

  static bool isEqual(const ThisInstInfo &LHS, const InstInfo &RHS) {
    return isEqual(RHS, LHS);
  }

  static bool isEqual(const InstInfo &II, const ThisInstInfo &TII) {
    if (II.Key != StringRef(TII.InstKey))
      return false;
    if (II.NTArgs != StringRef((const char *) TII.NTTPValues,
                               TII.NTTPValuesSize))
      return false;
    if (II.TArgs.size() != TII.TypeStringsCnt)
      return false;
    for (unsigned int i = 0, e = TII.TypeStringsCnt; i != e; ++i)
      if (II.TArgs[i] != StringRef(TII.TypeStrings[i]))
        return false;

    return true;
  }
};

void updateActiveInstantiation(const InstInfo&, InstData);

}
}

#endif //CLANG_JIT_COMPILER_H
