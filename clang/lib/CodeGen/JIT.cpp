//===--- CodeGenAction.cpp - LLVM Code Generation Frontend Action ---------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "JIT.h"
#include "CGCXXABI.h"
#include "CodeGenModule.h"
#include "CoverageMappingGen.h"
#include "MacroPPCallbacks.h"
#include "Tuner/Debug.h"
#include "Tuner/TunerDriver.h"
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
#include "clang/CodeGen/CodeGenAction.h"
#include "clang/CodeGen/ModuleBuilder.h"
#include "clang/CodeGen/Tuning.h"
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
#include "llvm/Support/FormatAdapters.h"
#include "llvm/Support/FormatVariadic.h"
#include "llvm/Support/Host.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/MemoryBuffer.h"
#include "llvm/Support/Mutex.h"
#include "llvm/Support/Path.h"
#include "llvm/Support/SourceMgr.h"
#include "llvm/Support/TargetSelect.h"
#include "llvm/Support/Timer.h"
#include "llvm/Support/ToolOutputFile.h"
#include "llvm/Support/VirtualFileSystem.h"
#include "llvm/Support/YAMLTraits.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Transforms/IPO/Internalize.h"
#include "llvm/Transforms/Utils/Cloning.h"

#include "Driver.h"
#include <cassert>
#include <cstdlib> // ::getenv
#include <cstring>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <system_error>
#include <unordered_map>
#include <utility>
#include <vector>
#include <mutex>

using namespace llvm;
using namespace clang;
using namespace clang::jit;

#define DEBUG_TYPE "clang-jit"

namespace {
// FIXME: This is copied from lib/Frontend/ASTUnit.cpp

/// Gathers information from ASTReader that will be used to initialize
/// a Preprocessor.
class ASTInfoCollector : public ASTReaderListener {
  Preprocessor &PP;
  ASTContext *Context;
  HeaderSearchOptions &HSOpts;
  PreprocessorOptions &PPOpts;
  LangOptions &LangOpt;
  std::shared_ptr<clang::TargetOptions> &TargetOpts;
  IntrusiveRefCntPtr<TargetInfo> &Target;
  unsigned &Counter;
  bool InitializedLanguage = false;

public:
  ASTInfoCollector(Preprocessor &PP, ASTContext *Context,
                   HeaderSearchOptions &HSOpts, PreprocessorOptions &PPOpts,
                   LangOptions &LangOpt,
                   std::shared_ptr<clang::TargetOptions> &TargetOpts,
                   IntrusiveRefCntPtr<TargetInfo> &Target, unsigned &Counter)
      : PP(PP), Context(Context), HSOpts(HSOpts), PPOpts(PPOpts),
        LangOpt(LangOpt), TargetOpts(TargetOpts), Target(Target),
        Counter(Counter) {}

  bool ReadLanguageOptions(const LangOptions &LangOpts, bool Complain,
                           bool AllowCompatibleDifferences) override {
    if (InitializedLanguage)
      return false;

    LangOpt = LangOpts;
    InitializedLanguage = true;

    updated();
    return false;
  }

  bool ReadHeaderSearchOptions(const HeaderSearchOptions &HSOpts,
                               StringRef SpecificModuleCachePath,
                               bool Complain) override {
    this->HSOpts = HSOpts;
    return false;
  }

  bool ReadPreprocessorOptions(const PreprocessorOptions &PPOpts, bool Complain,
                               std::string &SuggestedPredefines) override {
    this->PPOpts = PPOpts;
    return false;
  }

  bool ReadTargetOptions(const clang::TargetOptions &TargetOpts, bool Complain,
                         bool AllowCompatibleDifferences) override {
    // If we've already initialized the target, don't do it again.
    if (Target)
      return false;

    this->TargetOpts = std::make_shared<clang::TargetOptions>(TargetOpts);
    Target =
        TargetInfo::CreateTargetInfo(PP.getDiagnostics(), this->TargetOpts);

    updated();
    return false;
  }

  void ReadCounter(const serialization::ModuleFile &M,
                   unsigned Value) override {
    Counter = Value;
  }

private:
  void updated() {
    if (!Target || !InitializedLanguage)
      return;

    // Inform the target of the language options.
    //
    // FIXME: We shouldn't need to do this, the target should be immutable once
    // created. This complexity should be lifted elsewhere.
    Target->adjust(LangOpt);

    // Initialize the preprocessor.
    PP.Initialize(*Target);

    if (!Context)
      return;

    // Initialize the ASTContext
    Context->InitBuiltinTypes(*Target);

    // Adjust printing policy based on language options.
    Context->setPrintingPolicy(PrintingPolicy(LangOpt));

    // We didn't have access to the comment options when the ASTContext was
    // constructed, so register them now.
    Context->getCommentCommandTraits().registerCommentOptions(
        LangOpt.CommentOpts);
  }
};

class JFIMapDeclVisitor : public RecursiveASTVisitor<JFIMapDeclVisitor> {
  DenseMap<unsigned, FunctionDecl *> &Map;

public:
  explicit JFIMapDeclVisitor(DenseMap<unsigned, FunctionDecl *> &M) : Map(M) {}

  bool shouldVisitTemplateInstantiations() const { return true; }

  bool VisitFunctionDecl(const FunctionDecl *D) {
    if (auto *A = D->getAttr<JITFuncInstantiationAttr>())
      Map[A->getId()] = const_cast<FunctionDecl *>(D);
    return true;
  }
};

class JFICSMapDeclVisitor : public RecursiveASTVisitor<JFICSMapDeclVisitor> {
  DenseMap<unsigned, FunctionDecl *> &Map;
  SmallVector<FunctionDecl *, 1> CurrentFD;

public:
  explicit JFICSMapDeclVisitor(DenseMap<unsigned, FunctionDecl *> &M)
      : Map(M) {}

  bool TraverseFunctionDecl(FunctionDecl *FD) {
    CurrentFD.push_back(FD);
    bool Continue =
        RecursiveASTVisitor<JFICSMapDeclVisitor>::TraverseFunctionDecl(FD);
    CurrentFD.pop_back();

    return Continue;
  }

  bool VisitDeclRefExpr(DeclRefExpr *E) {
    auto *FD = dyn_cast<FunctionDecl>(E->getDecl());
    if (!FD)
      return true;

    auto *A = FD->getAttr<JITFuncInstantiationAttr>();
    if (!A)
      return true;

    Map[A->getId()] = CurrentFD.back();

    return true;
  }
};

unsigned LastUnique = 0;
std::unique_ptr<llvm::LLVMContext> LCtx;

bool InitializedDevTarget = false;

struct TUData {
  std::unique_ptr<CompilerData> CD;
  std::unique_ptr<Driver> CompilerDriver;
};

llvm::sys::SmartMutex<false> Mutex;
bool InitializedTarget = false;
llvm::DenseMap<const void *, TUData> TUCompilerData;

llvm::sys::SmartMutex<false> IMutex;
llvm::DenseMap<InstInfo, InstData, InstMapInfo> Instantiations;

enum class DriverType {
  FAST, TUNER
};

inline DriverType loadDriverTypeEnv() {
  auto DriverStr = std::getenv("CJ_DRIVER");
  if (DriverStr && std::strcmp(DriverStr, "tuner") == 0) {
    return DriverType::TUNER;
  } else {
    return DriverType::FAST;
  }
}

inline LogLevel loadDebugLvlEnv() {
  auto DebugStr = std::getenv("CJ_DEBUG");
  unsigned Lvl = LOG_ERROR;
  if (DebugStr) {
    int StrInt = std::atoi(DebugStr);
    if (StrInt >= 0 && StrInt <= LOG_DEBUG) {
      Lvl = StrInt;
      outs() << "[JIT] Debug level is " << Lvl << "\n";
    } else {
      outs () << "[JIT] Invalid debug level: " << DebugStr << "\n";
    }
  }
  return static_cast<LogLevel>(Lvl);
}

} // anonymous namespace

namespace clang {
namespace jit {

std::once_flag LogLvlLoaded;
unsigned LogLvl = LOG_ERROR;

void BackendConsumer::HandleTranslationUnit(ASTContext &C) {
  Gen->HandleTranslationUnit(C);

  // Silently ignore if we weren't initialized for some reason.
  if (!getModule())
    return;

  for (auto &BM : DevLinkMods) {
    std::unique_ptr<llvm::Module> M = llvm::CloneModule(*BM);
    M->setDataLayout(getModule()->getDataLayoutStr());
    M->setTargetTriple(getModule()->getTargetTriple());

    for (Function &F : *M)
      Gen->CGM().AddDefaultFnAttrs(F);

    bool Err = Linker::linkModules(
        *getModule(), std::move(M), llvm::Linker::Flags::LinkOnlyNeeded,
        [](llvm::Module &M, const llvm::StringSet<> &GVS) {
          internalizeModule(M, [&GVS](const llvm::GlobalValue &GV) {
            return !GV.hasName() || (GVS.count(GV.getName()) == 0);
          });
        });

    if (Err)
      fatal();
  }
}

ClangJIT::ClangJIT(DenseMap<StringRef, const void *> &LocalSymAddrs)
    : LocalSymAddrs(LocalSymAddrs),
      Resolver(createClangLookupResolver(
          ES, [this](const std::string &Name) { return findSymbol(Name); },
          [](Error Err) { cantFail(std::move(Err), "lookupFlags failed"); })),
      TM(EngineBuilder().selectTarget()), DL(TM->createDataLayout()),
      ObjectLayer(ES,
                  [this](llvm::orc::VModuleKey) {
                    return ObjLayerT::Resources{
                        std::make_shared<SectionMemoryManager>(), Resolver};
                  }),
      CompileLayer(ObjectLayer, llvm::orc::SimpleCompiler(*TM)),
      CXXRuntimeOverrides([this](const std::string &S) { return mangle(S); }) {
  llvm::sys::DynamicLibrary::LoadLibraryPermanently(nullptr);
}

llvm::orc::VModuleKey ClangJIT::addModule(std::unique_ptr<llvm::Module> M) {
  // Record the static constructors and destructors. We have to do this before
  // we hand over ownership of the module to the JIT.
  std::vector<std::string> CtorNames, DtorNames;
  for (auto Ctor : llvm::orc::getConstructors(*M))
    if (Ctor.Func && !Ctor.Func->hasAvailableExternallyLinkage())
      CtorNames.push_back(mangle(Ctor.Func->getName()));
  for (auto Dtor : llvm::orc::getDestructors(*M))
    if (Dtor.Func && !Dtor.Func->hasAvailableExternallyLinkage())
      DtorNames.push_back(mangle(Dtor.Func->getName()));

  auto K = ES.allocateVModule();
  cantFail(CompileLayer.addModule(K, std::move(M)));
  ModuleKeys.push_back(K);

  // Run the static constructors, and save the static destructor runner for
  // execution when the JIT is torn down.
  llvm::orc::LegacyCtorDtorRunner<CompileLayerT> CtorRunner(
      std::move(CtorNames), K);
  if (auto Err = CtorRunner.runViaLayer(CompileLayer)) {
    llvm::errs() << "Error while running static constructors: " << Err << "\n";
    errs().flush();
    fatal();
  }

  IRStaticDestructorRunners.emplace_back(std::move(DtorNames), K);

  return K;
}

void ClangJIT::removeModule(llvm::orc::VModuleKey K) {
  ModuleKeys.erase(find(ModuleKeys, K));
  cantFail(CompileLayer.removeModule(K));
}

CompilerData::CompilerData(const void *CmdArgs, unsigned CmdArgsLen,
                           const void *ASTBuffer, size_t ASTBufferSize,
                           const void *IRBuffer, size_t IRBufferSize,
                           const void **LocalPtrs, unsigned LocalPtrsCnt,
                           const void **LocalDbgPtrs, unsigned LocalDbgPtrsCnt,
                           const clang::jit::DevData *DeviceData,
                           unsigned DevCnt, int ForDev) {
  bool IsForDev = (ForDev != -1);

  StringRef CombinedArgv((const char *)CmdArgs, CmdArgsLen);
  SmallVector<StringRef, 32> Argv;
  CombinedArgv.split(Argv, '\0', /*MaxSplit*/ -1, false);

  llvm::opt::ArgStringList CC1Args;
  for (auto &ArgStr : Argv)
    CC1Args.push_back(ArgStr.begin());

  unsigned MissingArgIndex, MissingArgCount;

  llvm::opt::InputArgList ParsedArgs =
                              driver::getDriverOptTable().ParseArgs(CC1Args, MissingArgIndex, MissingArgCount);

  DiagOpts = new DiagnosticOptions();
  ParseDiagnosticArgs(*DiagOpts, ParsedArgs);
  DiagnosticPrinter.reset(new TextDiagnosticPrinter(llvm::errs(), &*DiagOpts));
  Diagnostics = new DiagnosticsEngine(
      IntrusiveRefCntPtr<DiagnosticIDs>(new DiagnosticIDs()), &*DiagOpts,
      DiagnosticPrinter.get(), false);

  // Note that LangOpts, TargetOpts can also be read from the AST, but
  // CodeGenOpts need to come from the stored command line.

  Invocation.reset(new CompilerInvocation);
  CompilerInvocation::CreateFromArgs(
      *Invocation, CC1Args, *Diagnostics);
  Invocation->getFrontendOpts().DisableFree = false;
  Invocation->getCodeGenOpts().DisableFree = false;

  InMemoryFileSystem = new llvm::vfs::InMemoryFileSystem;
  FileMgr = new FileManager(FileSystemOptions(), InMemoryFileSystem);

  const char *Filename = "__clang_jit.pcm";
  StringRef ASTBufferSR((const char *)ASTBuffer, ASTBufferSize);
  InMemoryFileSystem->addFile(
      Filename, 0, llvm::MemoryBuffer::getMemBufferCopy(ASTBufferSR));

  PCHContainerRdr.reset(new RawPCHContainerReader);
  SourceMgr = new SourceManager(*Diagnostics, *FileMgr,
                                /*UserFilesAreVolatile*/ false);
  ModuleCache = new InMemoryModuleCache;
  HSOpts = std::make_shared<HeaderSearchOptions>();
  HSOpts->ModuleFormat = PCHContainerRdr->getFormat();
  HeaderInfo.reset(new HeaderSearch(HSOpts,
                                    *SourceMgr,
                                    *Diagnostics,
                                    *Invocation->getLangOpts(),
      /*Target=*/nullptr));
  PPOpts = std::make_shared<PreprocessorOptions>();

  unsigned Counter;

  PP = std::make_shared<Preprocessor>(
      PPOpts, *Diagnostics, *Invocation->getLangOpts(),
      *SourceMgr, *HeaderInfo, ModuleLoader,
      /*IILookup=*/nullptr,
      /*OwnsHeaderSearch=*/false);

  // For parsing type names in strings later, we'll need to have Preprocessor
  // keep the Lexer around even after it hits the end of the each file (used
  // for each type name).
  PP->enableIncrementalProcessing();

  Ctx = new ASTContext(*Invocation->getLangOpts(), *SourceMgr,
                       PP->getIdentifierTable(), PP->getSelectorTable(),
                       PP->getBuiltinInfo());

  Reader = new ASTReader(*PP, *ModuleCache, Ctx.get(), *PCHContainerRdr, {},
      /*isysroot=*/"",
      /*DisableValidation=*/ false,
      /*AllowPCHWithCompilerErrors*/ false);

  Reader->setListener(std::make_unique<ASTInfoCollector>(
      *PP, Ctx.get(), *HSOpts, *PPOpts, *Invocation->getLangOpts(), TargetOpts,
      Target, Counter));

  Ctx->setExternalSource(Reader);

  switch (Reader->ReadAST(Filename, serialization::MK_MainFile,
                          SourceLocation(), ASTReader::ARR_None)) {
  case ASTReader::Success:
    break;

  case ASTReader::Failure:
  case ASTReader::Missing:
  case ASTReader::OutOfDate:
  case ASTReader::VersionMismatch:
  case ASTReader::ConfigurationMismatch:
  case ASTReader::HadErrors:
    Diagnostics->Report(diag::err_fe_unable_to_load_pch);
    fatal();
    return;
  }

  PP->setCounterValue(Counter);

  // Now that we've read the language options from the AST file, change the JIT
  // mode.
  Invocation->getLangOpts()->setCPlusPlusJIT(LangOptions::JITMode::JM_IsJIT);

  // Keep externally available functions, etc.
  Invocation->getCodeGenOpts().PrepareForLTO = true;

  BackendAction BA = Backend_EmitNothing;
  std::unique_ptr<raw_pwrite_stream> OS(new llvm::raw_null_ostream);

  if (ForDev) {
    BA = Backend_EmitAssembly;
    OS.reset(new raw_svector_ostream(DevAsm));
  }

  Consumer.reset(new BackendConsumer(
      BA, *Diagnostics, Invocation->getHeaderSearchOpts(),
      Invocation->getPreprocessorOpts(), Invocation->getCodeGenOpts(),
      Invocation->getTargetOpts(), *Invocation->getLangOpts(), false, Filename,
      std::move(OS), *LCtx, DevLinkMods));

  // Create a semantic analysis object and tell the AST reader about it.
  S.reset(new Sema(*PP, *Ctx, *Consumer));
  S->Initialize();
  Reader->InitializeSema(*S);

  // Tell the diagnostic client that we have started a source file.
  Diagnostics->getClient()->BeginSourceFile(PP->getLangOpts(), PP.get());

  JFIMapDeclVisitor(FuncMap).TraverseAST(*Ctx);
  JFICSMapDeclVisitor(CSFuncMap).TraverseAST(*Ctx);

  if (IRBufferSize) {
    llvm::SMDiagnostic Err;
    StringRef IRBufferSR((const char *)IRBuffer, IRBufferSize);
    RunningMod =
        parseIR(*llvm::MemoryBuffer::getMemBufferCopy(IRBufferSR), Err, *LCtx);

    for (auto &F : RunningMod->functions())
      if (!F.isDeclaration())
        F.setLinkage(llvm::GlobalValue::AvailableExternallyLinkage);

    for (auto &GV : RunningMod->global_values())
      if (!GV.isDeclaration()) {
        if (GV.hasAppendingLinkage())
          cast<GlobalVariable>(GV).setInitializer(nullptr);
        else if (isa<GlobalAlias>(GV))
          // Aliases cannot have externally-available linkage, so give them
          // private linkage.
          GV.setLinkage(llvm::GlobalValue::PrivateLinkage);
        else
          GV.setLinkage(llvm::GlobalValue::AvailableExternallyLinkage);
      }
  }

  Consumer->Initialize(*Ctx);

  for (unsigned Idx = 0; Idx < 2 * LocalPtrsCnt; Idx += 2) {
    const char *Name = (const char *)LocalPtrs[Idx];
    const void *Ptr = LocalPtrs[Idx + 1];
    LocalSymAddrs[Name] = Ptr;
  }

  for (unsigned Idx = 0; Idx < 2 * LocalDbgPtrsCnt; Idx += 2) {
    const char *Name = (const char *)LocalDbgPtrs[Idx];
    const void *Ptr = LocalDbgPtrs[Idx + 1];
    LocalSymAddrs[Name] = Ptr;
  }

  if (!IsForDev)
    CJ = std::make_unique<ClangJIT>(LocalSymAddrs);

  if (IsForDev)
    for (unsigned i = 0; i < DeviceData[ForDev].FileDataCnt; ++i) {
      StringRef FileBufferSR((const char *)DeviceData[ForDev].FileData[i].Data,
                             DeviceData[ForDev].FileData[i].DataSize);

      llvm::SMDiagnostic Err;
      DevLinkMods.push_back(parseIR(
          *llvm::MemoryBuffer::getMemBufferCopy(FileBufferSR), Err, *LCtx));
    }

  if (!IsForDev && Invocation->getLangOpts()->CUDA) {
    typedef int (*cudaGetDevicePtr)(int *);
    auto cudaGetDevice =
        (cudaGetDevicePtr)RTDyldMemoryManager::getSymbolAddressInProcess(
            "cudaGetDevice");
    if (!cudaGetDevice) {
      llvm::errs() << "Could not find CUDA API functions; "
                      "did you forget to link with -lcudart?\n";
      fatal();
    }

    typedef int (*cudaGetDeviceCountPtr)(int *);
    auto cudaGetDeviceCount =
        (cudaGetDeviceCountPtr)RTDyldMemoryManager::getSymbolAddressInProcess(
            "cudaGetDeviceCount");

    int SysDevCnt;
    if (cudaGetDeviceCount(&SysDevCnt)) {
      llvm::errs() << "Failed to get CUDA device count!\n";
      fatal();
    }

    typedef int (*cudaDeviceGetAttributePtr)(int *, int, int);
    auto cudaDeviceGetAttribute = (cudaDeviceGetAttributePtr)
        RTDyldMemoryManager::getSymbolAddressInProcess(
            "cudaDeviceGetAttribute");

    if (SysDevCnt) {
      int CDev;
      if (cudaGetDevice(&CDev))
        fatal();

      int CLMajor, CLMinor;
      if (cudaDeviceGetAttribute(
              &CLMajor, /*cudaDevAttrComputeCapabilityMajor*/ 75, CDev))
        fatal();
      if (cudaDeviceGetAttribute(
              &CLMinor, /*cudaDevAttrComputeCapabilityMinor*/ 76, CDev))
        fatal();

      SmallString<6> EffArch;
      raw_svector_ostream(EffArch) << "sm_" << CLMajor << CLMinor;

      SmallVector<StringRef, 2> DevArchs;
      for (unsigned i = 0; i < DevCnt; ++i) {
        if (!Triple(DeviceData[i].Triple).isNVPTX())
          continue;
        if (!StringRef(DeviceData[i].Arch).startswith("sm_"))
          continue;
        DevArchs.push_back(DeviceData[i].Arch);
      }

      std::sort(DevArchs.begin(), DevArchs.end());
      auto ArchI = std::upper_bound(DevArchs.begin(), DevArchs.end(), EffArch);
      if (ArchI == DevArchs.begin()) {
        llvm::errs() << "No JIT device configuration supports " << EffArch
                     << "\n";
        fatal();
      }

      auto BestDevArch = *--ArchI;
      int BestDevIdx = 0;
      for (; BestDevIdx < (int)DevCnt; ++BestDevIdx) {
        if (!Triple(DeviceData[BestDevIdx].Triple).isNVPTX())
          continue;
        if (DeviceData[BestDevIdx].Arch == BestDevArch)
          break;
      }

      assert(BestDevIdx != (int)DevCnt &&
             "Didn't find the chosen device data?");

      if (!InitializedDevTarget) {
        // In theory, we only need to initialize the NVPTX target here,
        // however, there doesn't seem to be any good way to know if the
        // NVPTX target is enabled.
        //
        // LLVMInitializeNVPTXTargetInfo();
        // LLVMInitializeNVPTXTarget();
        // LLVMInitializeNVPTXTargetMC();
        // LLVMInitializeNVPTXAsmPrinter();

        llvm::InitializeAllTargets();
        llvm::InitializeAllTargetMCs();
        llvm::InitializeAllAsmPrinters();

        InitializedDevTarget = true;
      }

      DevCD.reset(new CompilerData(
          DeviceData[BestDevIdx].CmdArgs, DeviceData[BestDevIdx].CmdArgsLen,
          DeviceData[BestDevIdx].ASTBuffer,
          DeviceData[BestDevIdx].ASTBufferSize, nullptr, 0, nullptr, 0, nullptr,
          0, DeviceData, DevCnt, BestDevIdx));
    }
  }
}

void CompilerData::restoreFuncDeclContext(FunctionDecl *FunD) {
  // NOTE: This mirrors the corresponding code in
  // Parser::ParseLateTemplatedFuncDef (which is used to late parse a C++
  // function template in Microsoft mode).

  struct ContainingDC {
    ContainingDC(DeclContext *DC, bool ShouldPush) : Pair(DC, ShouldPush) {}
    llvm::PointerIntPair<DeclContext *, 1, bool> Pair;
    DeclContext *getDC() { return Pair.getPointer(); }
    bool shouldPushDC() { return Pair.getInt(); }
  };

  SmallVector<ContainingDC, 4> DeclContextsToReenter;
  DeclContext *DD = FunD;
  DeclContext *NextContaining = S->getContainingDC(DD);
  while (DD && !DD->isTranslationUnit()) {
    bool ShouldPush = DD == NextContaining;
    DeclContextsToReenter.push_back({DD, ShouldPush});
    if (ShouldPush)
      NextContaining = S->getContainingDC(DD);
    DD = DD->getLexicalParent();
  }

  // Reenter template scopes from outermost to innermost.
  for (ContainingDC CDC : reverse(DeclContextsToReenter)) {
    (void)S->ActOnReenterTemplateScope(S->getCurScope(),
                                       cast<Decl>(CDC.getDC()));
    if (CDC.shouldPushDC())
      S->PushDeclContext(S->getCurScope(), CDC.getDC());
  }
}

void CompilerData::emitAllNeeded(bool CheckExisting) {
  // There might have been functions/variables with local linkage that were
  // only used by JIT functions. These would not have been used during
  // initial code generation for this translation unit, and so not emitted.
  // We need to make sure that they're emited now (if they're now necessary).

  // Note that we skip having the code generator visiting the decl if it is
  // already defined or already present in our running module. Note that this
  // is not sufficient to prevent all redundant code generation (this might
  // also happen during the instantiation of the top-level function
  // template), and this is why we merge the running module into the new one
  // with the running-module overriding new entities.

  SmallSet<StringRef, 16> LastDeclNames;
  bool Changed;
  do {
    Changed = false;

    Consumer->getCodeGenerator()->CGM().EmitAllDeferred([&](GlobalDecl GD) {
      auto MName = Consumer->getCodeGenerator()->CGM().getMangledName(GD);
      if (!CheckExisting || !CJ->findSymbol(MName)) {
        Changed = true;
        return false;
      }

      return true;
    });

    SmallSet<StringRef, 16> DeclNames;
    for (auto &F : Consumer->getModule()->functions())
      if (F.isDeclaration() && !F.isIntrinsic())
        if (!LastDeclNames.count(F.getName()))
          DeclNames.insert(F.getName());

    for (auto &GV : Consumer->getModule()->global_values())
      if (GV.isDeclaration())
        if (!LastDeclNames.count(GV.getName()))
          DeclNames.insert(GV.getName());

    for (auto &DeclName : DeclNames) {
      if (CheckExisting && CJ->findSymbol(DeclName))
        continue;

      Decl *D = const_cast<Decl *>(
          Consumer->getCodeGenerator()->GetDeclForMangledName(DeclName));
      if (!D)
        continue;

      Consumer->HandleInterestingDecl(DeclGroupRef(D));
      LastDeclNames.insert(DeclName);
      Changed = true;
    }
  } while (Changed);
}

std::unique_ptr<llvm::Module> CompilerData::createModule(StringRef SMName) {
  auto *FDecl = Consumer->getCodeGenerator()->GetDeclForMangledName(SMName);

  emitAllNeeded();

  if (DevCD)
    DevCD->emitAllNeeded(false);

  // Before anything gets optimized, mark the top-level symbol we're
  // generating so that it doesn't get eliminated by the optimizer.

  auto *TopGV =
      cast<GlobalObject>(Consumer->getModule()->getNamedValue(SMName));
  assert(TopGV && "Didn't generate the desired top-level symbol?");

  TopGV->setLinkage(llvm::GlobalValue::ExternalLinkage);
  TopGV->setComdat(nullptr);

  // Finalize the module, generate module-level metadata, etc.

  if (DevCD) {
    DevCD->Consumer->HandleTranslationUnit(*DevCD->Ctx);
    DevCD->Consumer->EmitOptimized();

    // We have now created the PTX output, but what we really need as a
    // fatbin that the CUDA runtime will recognize.

    // The outer header of the fat binary is documented in the CUDA
    // fatbinary.h header. As mentioned there, the overall size must be a
    // multiple of eight, and so we must make sure that the PTX is.
    while (DevCD->DevAsm.size() % 7)
      DevCD->DevAsm += ' ';
    DevCD->DevAsm += '\0';

    // NVIDIA, unfortunatly, does not provide full documentation on their
    // fatbin format. There is some information on the outer header block in
    // the CUDA fatbinary.h header. Also, it is possible to figure out more
    // about the format by creating fatbins using the provided utilities
    // and then observing what cuobjdump reports about the resulting files.
    // There are some other online references which shed light on the format,
    // including https://reviews.llvm.org/D8397 and FatBinaryContext.{cpp,h}
    // from the GPU Ocelot project (https://github.com/gtcasl/gpuocelot).

    SmallString<128> FatBin;
    llvm::raw_svector_ostream FBOS(FatBin);

    struct FatBinHeader {
      uint32_t Magic;      // 0x00
      uint16_t Version;    // 0x04
      uint16_t HeaderSize; // 0x06
      uint32_t DataSize;   // 0x08
      uint32_t unknown0c;  // 0x0c
    public:
      FatBinHeader(uint32_t DataSize)
          : Magic(0xba55ed50), Version(1), HeaderSize(sizeof(*this)),
            DataSize(DataSize), unknown0c(0) {}
    };

    enum FatBinFlags {
      AddressSize64 = 0x01,
      HasDebugInfo = 0x02,
      ProducerCuda = 0x04,
      HostLinux = 0x10,
      HostMac = 0x20,
      HostWindows = 0x40
    };

    struct FatBinFileHeader {
      uint16_t Kind;             // 0x00
      uint16_t unknown02;        // 0x02
      uint32_t HeaderSize;       // 0x04
      uint32_t DataSize;         // 0x08
      uint32_t unknown0c;        // 0x0c
      uint32_t CompressedSize;   // 0x10
      uint32_t SubHeaderSize;    // 0x14
      uint16_t VersionMinor;     // 0x18
      uint16_t VersionMajor;     // 0x1a
      uint32_t CudaArch;         // 0x1c
      uint32_t unknown20;        // 0x20
      uint32_t unknown24;        // 0x24
      uint32_t Flags;            // 0x28
      uint32_t unknown2c;        // 0x2c
      uint32_t unknown30;        // 0x30
      uint32_t unknown34;        // 0x34
      uint32_t UncompressedSize; // 0x38
      uint32_t unknown3c;        // 0x3c
      uint32_t unknown40;        // 0x40
      uint32_t unknown44;        // 0x44
      FatBinFileHeader(uint32_t DataSize, uint32_t CudaArch, uint32_t Flags)
          : Kind(1 /*PTX*/), unknown02(0x0101), HeaderSize(sizeof(*this)),
            DataSize(DataSize), unknown0c(0), CompressedSize(0),
            SubHeaderSize(HeaderSize - 8), VersionMinor(2), VersionMajor(4),
            CudaArch(CudaArch), unknown20(0), unknown24(0), Flags(Flags),
            unknown2c(0), unknown30(0), unknown34(0), UncompressedSize(0),
            unknown3c(0), unknown40(0), unknown44(0) {}
    };

    uint32_t CudaArch;
    StringRef(DevCD->Invocation->getTargetOpts().CPU)
        .drop_front(3 /*sm_*/)
        .getAsInteger(10, CudaArch);

    uint32_t Flags = ProducerCuda;
    if (DevCD->Invocation->getCodeGenOpts().getDebugInfo() >=
        codegenoptions::LimitedDebugInfo)
      Flags |= HasDebugInfo;

    if (Triple(DevCD->Invocation->getTargetOpts().Triple).getArch() ==
        Triple::nvptx64)
      Flags |= AddressSize64;

    if (Triple(Invocation->getTargetOpts().Triple).isOSWindows())
      Flags |= HostWindows;
    else if (Triple(Invocation->getTargetOpts().Triple).isOSDarwin())
      Flags |= HostMac;
    else
      Flags |= HostLinux;

    FatBinFileHeader FBFHdr(DevCD->DevAsm.size(), CudaArch, Flags);
    FatBinHeader FBHdr(DevCD->DevAsm.size() + FBFHdr.HeaderSize);

    FBOS.write((char *)&FBHdr, FBHdr.HeaderSize);
    FBOS.write((char *)&FBFHdr, FBFHdr.HeaderSize);
    FBOS << DevCD->DevAsm;

    if (::getenv("CLANG_JIT_CUDA_DUMP_DYNAMIC_FATBIN")) {
      SmallString<128> Path;
      auto EC = llvm::sys::fs::createUniqueFile(
          llvm::Twine("clang-jit-") +
              llvm::sys::path::filename(
                  Invocation->getCodeGenOpts().MainFileName) +
              llvm::Twine("-%%%%.fatbin"),
          Path, llvm::sys::fs::owner_read | llvm::sys::fs::owner_write);
      if (!EC) {
        raw_fd_ostream DOS(Path, EC);
        if (!EC)
          DOS << FatBin;
      }
    }

    Consumer->getCodeGenerator()->CGM().getCodeGenOpts().GPUBinForJIT = FatBin;
    DevCD->DevAsm.clear();
  }

  // Finalize translation unit. No optimization yet.
  Consumer->HandleTranslationUnit(*Ctx);

  // First, mark everything we've newly generated with external linkage. When
  // we generate additional modules, we'll mark these functions as available
  // externally, and so we're likely to inline them, but if not, we'll need
  // to link with the ones generated here.

  for (auto &F : Consumer->getModule()->functions()) {
    F.setLinkage(llvm::GlobalValue::ExternalLinkage);
    F.setComdat(nullptr);

    //    if (!F.isDeclaration()) // TODO: Move into tuner driver
    //      JITCtx.ReplaceOnRecompilation.push_back(F.getName());
  }

  auto IsLocalUnnamedConst = [](llvm::GlobalValue &GV) {
    if (!GV.hasAtLeastLocalUnnamedAddr() || !GV.hasLocalLinkage())
      return false;

    auto *GVar = dyn_cast<llvm::GlobalVariable>(&GV);
    if (!GVar || !GVar->isConstant())
      return false;

    return true;
  };

  for (auto &GV : Consumer->getModule()->global_values()) {
    if (IsLocalUnnamedConst(GV) || GV.hasAppendingLinkage())
      continue;

    GV.setLinkage(llvm::GlobalValue::ExternalLinkage);
    if (auto *GO = dyn_cast<llvm::GlobalObject>(&GV))
      GO->setComdat(nullptr);
  }

  auto GenMod = Consumer->takeModule();
  // Reset for next instantiation
  Consumer->Initialize(*Ctx);

  return GenMod;
}

void CompilerData::prepareForLinking(llvm::Module *DstMod,
                                     const llvm::Module *SrcMod) {

  // During linking, all named metadata (except llvm.module.flags) from the
  // source module will be appended to the existing MD in the destination module
  // (see IRLinker::linkNamedMDNodes). This causes llvm.ident to double in size
  // every time its recompiled. To prevent this, we clear all named metadata of
  // the destination module that already exists in the source module.
  // TODO: Will this possibly destroy important information? If yes, fall back
  // to clearing llvm.ident directly.

  const NamedMDNode *ModFlags = DstMod->getModuleFlagsMetadata();
  for (NamedMDNode &NMD : DstMod->named_metadata()) {
    // Ignore module flags
    if (&NMD == ModFlags)
      continue;
    if (SrcMod->getNamedMetadata(NMD.getName())) {
      NMD.clearOperands();
    }
  }
}

void CompilerData::linkInAvailableDefs(llvm::Module &Mod,
                                       bool InvokeDefaultOpt) {

  auto IsLocalUnnamedConst = [](llvm::GlobalValue &GV) {
    if (!GV.hasAtLeastLocalUnnamedAddr() || !GV.hasLocalLinkage())
      return false;

    auto *GVar = dyn_cast<llvm::GlobalVariable>(&GV);
    if (!GVar || !GVar->isConstant())
      return false;

    return true;
  };

  // Here we link our previous cache of definitions, etc. into this module.
  // This includes all of our previously-generated functions (marked as
  // available externally). We prefer our previously-generated versions to
  // our current versions should both modules contain the same entities (as
  // the previously-generated versions have already been optimized).

  // We need to be specifically careful about constants in our module,
  // however. Clang will generate all string literals as .str (plus a
  // number), and these from previously-generated code will conflict with the
  // names chosen for string literals in this module.

  for (auto &GV : Mod.global_values()) {
    if (!IsLocalUnnamedConst(GV) && !GV.getName().startswith("__cuda_"))
      continue;

    if (!RunningMod->getNamedValue(GV.getName()))
      continue;

    llvm::SmallString<16> UniqueName(GV.getName());
    unsigned BaseSize = UniqueName.size();
    do {
      // Trim any suffix off and append the next number.
      UniqueName.resize(BaseSize);
      llvm::raw_svector_ostream S(UniqueName);
      S << "." << ++LastUnique;
    } while (RunningMod->getNamedValue(UniqueName));

    GV.setName(UniqueName);
  }

  // Clang will generate local init/deinit functions for variable
  // initialization, CUDA registration, etc. and these can't be shared with
  // the base part of the module (as they specifically initialize variables,
  // etc. that we just generated).

  for (auto &F : Mod.functions()) {
    // FIXME: This likely covers the set of TU-local init/deinit functions
    // that can't be shared with the base module. There should be a better
    // way to do this (e.g., we could record all functions that
    // CreateGlobalInitOrDestructFunction creates? - ___cuda_ would still be
    // a special case).
    if (!F.getName().startswith("__cuda_") &&
        !F.getName().startswith("_GLOBAL_") &&
        !F.getName().startswith("__GLOBAL_") &&
        !F.getName().startswith("__cxx_"))
      continue;

    if (!RunningMod->getFunction(F.getName()))
      continue;

    llvm::SmallString<16> UniqueName(F.getName());
    unsigned BaseSize = UniqueName.size();
    do {
      // Trim any suffix off and append the next number.
      UniqueName.resize(BaseSize);
      llvm::raw_svector_ostream S(UniqueName);
      S << "." << ++LastUnique;
    } while (RunningMod->getFunction(UniqueName));

    F.setName(UniqueName);
  }

  // TODO: This is a bug, there should not be any comdats in the saved module. This should be adressed before saving.
  for (auto &F : RunningMod->functions()) {
    if (F.isDeclarationForLinker() && F.hasComdat()) {
      errs() << "Saved module contains comdat for global " << F.getName() << ": deleting comdat\n";
      F.setComdat(nullptr);
    }
  }

  prepareForLinking(&Mod, RunningMod.get());
  if (Linker::linkModules(Mod, llvm::CloneModule(*RunningMod),
                          Linker::Flags::OverrideFromSrc))
    fatal();

  // Aliases are not allowed to point to functions with available_externally
  // linkage. We solve this by replacing these aliases with the definition of
  // the aliasee. Candidates are identified first, then erased in a second step
  // to avoid invalidating the iterator.
  SmallPtrSet<GlobalAlias *, 4> ToReplace;
  for (auto &Alias : Mod.aliases()) {
    // Aliases may point to other aliases but we only need to alter the lowest
    // level one Only function declarations are relevant
    auto Aliasee = dyn_cast<Function>(Alias.getAliasee());
    if (!Aliasee || !Aliasee->isDeclarationForLinker()) {
      continue;
    }
    assert(Aliasee->hasAvailableExternallyLinkage() &&
           "Broken module: alias points to declaration");
    ToReplace.insert(&Alias);
  }

  for (auto *Alias : ToReplace) {
    auto Aliasee = cast<Function>(Alias->getAliasee());

    llvm::ValueToValueMapTy VMap;
    Function *AliasReplacement = llvm::CloneFunction(Aliasee, VMap);

    AliasReplacement->setLinkage(Alias->getLinkage());
    Alias->replaceAllUsesWith(AliasReplacement);

    SmallString<32> AliasName = Alias->getName();
    Alias->eraseFromParent();
    AliasReplacement->setName(AliasName);
  }

  if (InvokeDefaultOpt) {
    assert(CJ && "This method must not be called from the device compiler");
    EmitBackendOutput(
        *Diagnostics, Invocation->getHeaderSearchOpts(),
        Invocation->getCodeGenOpts(), Invocation->getTargetOpts(),
        *Invocation->getLangOpts(), Ctx->getTargetInfo().getDataLayout(), &Mod,
        Backend_EmitNothing,
        std::unique_ptr<raw_pwrite_stream>(new llvm::raw_null_ostream));
  }
}

void CompilerData::makeDefsAvailable(std::unique_ptr<llvm::Module> NewMod) {
  // Now that we've generated code for this module, take them optimized code
  // and mark the definitions as available externally. We'll link them into
  // future modules this way so that they can be inlined.

  // Linker does not link definitions marked as available_externally by default,
  // that's why we need to specify OverrideFromSource.

  for (auto &F : NewMod->functions())
    if (!F.isDeclaration())
      F.setLinkage(llvm::GlobalValue::AvailableExternallyLinkage);

  for (auto &GV : NewMod->global_values())
    if (!GV.isDeclaration()) {
      if (GV.hasAppendingLinkage())
        cast<GlobalVariable>(GV).setInitializer(nullptr);
      else if (isa<GlobalAlias>(GV))
        // Aliases cannot have externally-available linkage, so give them
        // private linkage.
        GV.setLinkage(llvm::GlobalValue::PrivateLinkage);
      else
        GV.setLinkage(llvm::GlobalValue::AvailableExternallyLinkage);
    }

  prepareForLinking(RunningMod.get(), NewMod.get());

  if (Linker::linkModules(*RunningMod, std::move(NewMod),
                          Linker::Flags::OverrideFromSrc))
    fatal();
}

void updateActiveInstantiation(const InstInfo& Inst, InstData Data) {
  llvm::sys::ScopedLock Guard(IMutex);
  Instantiations[Inst] = std::move(Data);
}


} // namespace jit
} // namespace clang


extern "C"
#ifdef _MSC_VER
    __declspec(dllexport)
#endif
        void *__clang_jit(const void *CmdArgs, unsigned CmdArgsLen,
                          const void *ASTBuffer, size_t ASTBufferSize,
                          const void *IRBuffer, size_t IRBufferSize,
                          const void **LocalPtrs, unsigned LocalPtrsCnt,
                          const void **LocalDbgPtrs, unsigned LocalDbgPtrsCnt,
                          const DevData *DeviceData, unsigned DevCnt,
                          const void *NTTPValues, unsigned NTTPValuesSize,
                          const char **TypeStrings, unsigned TypeStringsCnt,
                          const char *InstKey, unsigned Idx) {

  bool AssumeInitialized = false;

  {
    llvm::sys::ScopedLock Guard(IMutex);
    auto II = Instantiations.find_as(ThisInstInfo(
        InstKey, NTTPValues, NTTPValuesSize, TypeStrings, TypeStringsCnt));
    if (II != Instantiations.end()) {
      if (II->second.UseFastLookup)
        return II->second.FPtr;
      AssumeInitialized = true;
    }
  }

  Driver *CDriver;
  // CompilerData *CD;
  if (!AssumeInitialized) {
    llvm::sys::ScopedLock Guard(Mutex);

    if (!InitializedTarget) {
      llvm::InitializeNativeTarget();
      llvm::InitializeNativeTargetAsmPrinter();
      llvm::InitializeNativeTargetAsmParser();

      LCtx.reset(new LLVMContext);

      InitializedTarget = true;
    }

    auto TUCDI = TUCompilerData.find(ASTBuffer);
    if (TUCDI == TUCompilerData.end()) {
#ifdef ENABLE_JIT_DEBUG
      std::call_once(LogLvlLoaded, []() {
        clang::jit::LogLvl = loadDebugLvlEnv();
      });
#endif

      auto *CD =
          new CompilerData(CmdArgs, CmdArgsLen, ASTBuffer, ASTBufferSize,
                           IRBuffer, IRBufferSize, LocalPtrs, LocalPtrsCnt,
                           LocalDbgPtrs, LocalDbgPtrsCnt, DeviceData, DevCnt);
      auto &TUD = TUCompilerData[ASTBuffer];
      TUD.CD.reset(CD);
      static const auto ActiveDriverType = loadDriverTypeEnv();
      switch(ActiveDriverType) {
        case DriverType::TUNER:
          JIT_INFO(llvm::dbgs() << "JIT Tuning enabled\n");
          TUD.CompilerDriver = std::make_unique<TunerDriver>(*CD);
          break;
        case DriverType::FAST:
          TUD.CompilerDriver = std::make_unique<SimpleDriver>(*CD);
          break;
      }
      CDriver = TUD.CompilerDriver.get();
    } else {
      CDriver = TUCDI->second.CompilerDriver.get();
    }
  } else {
    auto TUCDI = TUCompilerData.find(ASTBuffer);
    assert(TUCDI != TUCompilerData.end() &&
           "Compiler data for this TU is not initialized!");
    CDriver = TUCDI->second.CompilerDriver.get();
  }

  auto InstData =
      CDriver->resolve(ThisInstInfo(InstKey, NTTPValues, NTTPValuesSize,
                                    TypeStrings, TypeStringsCnt),
                       Idx);

  updateActiveInstantiation(InstInfo(InstKey, NTTPValues, NTTPValuesSize, TypeStrings,
                                     TypeStringsCnt), InstData);

  return InstData.FPtr;
}
