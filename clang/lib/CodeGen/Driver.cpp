//
// Created by sebastian on 08.11.19.
//

#include "Driver.h"
#include "Tuner/Debug.h"
#include "Tuner/Util.h"

using namespace llvm;

namespace clang {

namespace jit {

std::string TemplateInstantiationHelper::instantiate(CompilerData* TargetCD, const SmallVectorImpl<TemplateArgument>& ArgList) {
  auto& Ctx = TargetCD->Ctx;
  auto& S = TargetCD->S;
  auto& Consumer = TargetCD->Consumer;

  auto *FTSI = FD->getTemplateSpecializationInfo();

  SourceLocation Loc = FTSI->getPointOfInstantiation();
  auto *NewTAL = TemplateArgumentList::CreateCopy(*Ctx, ArgList);
  MultiLevelTemplateArgumentList SubstArgs(*NewTAL);

  auto *FunctionTemplate = FTSI->getTemplate();
  DeclContext *Owner = FunctionTemplate->getDeclContext();
  if (FunctionTemplate->getFriendObjectKind())
    Owner = FunctionTemplate->getLexicalDeclContext();

  std::string SMName;
  FunctionTemplateDecl *FTD = FTSI->getTemplate();
  sema::TemplateDeductionInfo Info(Loc);
  {
    Sema::InstantiatingTemplate Inst(
        *S, Loc, FTD, NewTAL->asArray(),
        Sema::CodeSynthesisContext::ExplicitTemplateArgumentSubstitution, Info);
    Sema::ContextRAII TUContext(*S, Ctx->getTranslationUnitDecl());

    auto *Specialization = cast_or_null<FunctionDecl>(
        S->SubstDecl(FunctionTemplate->getTemplatedDecl(), Owner, SubstArgs));
    if (!Specialization || Specialization->isInvalidDecl())
      fatal();

    Specialization->setTemplateSpecializationKind(TSK_ExplicitInstantiationDefinition, Loc);
    S->InstantiateFunctionDefinition(Loc, Specialization, true, true, true);

    SMName = Consumer->getCodeGenerator()->CGM().getMangledName(Specialization);
  }

  if (TargetCD->Diagnostics->hasErrorOccurred())
    fatal();

  return SMName;
}


std::string TemplateInstantiationHelper::instantiate(const SmallVectorImpl<TemplateArgument>& ArgList) {
  std::string SMName = instantiate(&CD, ArgList);
  if (CD.DevCD) {
    instantiate(CD.DevCD.get(), ArgList);
  }
  return SMName;
}

std::string SimpleDriver::instantiateTemplate(const void *NTTPValues, const char **TypeStrings,
                                              unsigned Idx) {
  TemplateInstantiationHelper InstHelper(CD, Idx);
  SmallVector<TemplateArgument, 8> TAs;
  InstHelper.processTemplateArgs(NTTPValues, TypeStrings, TAs, [](QualType CanonType, unsigned Pos, const SmallVectorImpl<uint64_t>& IntWords) -> Optional<TemplateArgument> {return {};});
  return InstHelper.instantiate(TAs);
}

InstData SimpleDriver::resolve(const ThisInstInfo &Inst, unsigned Idx) {
  std::string SMName = instantiateTemplate(Inst.NTTPValues, Inst.TypeStrings, Idx);

  // Now we know the name of the symbol, check to see if we already have it.
  if (auto SpecSymbol = CD.CJ->findSymbol(SMName))
    if (SpecSymbol.getAddress())
      return {(void *) llvm::cantFail(SpecSymbol.getAddress()), true};

  // Emit IR for generated template specialization.
  auto Mod = CD.createModule(SMName);
  JIT_DEBUG(util::dumpModule(*Mod, "Initial module"));
  // Link in existing definitions for inlining, execute default optimization pipeline.
  CD.linkInAvailableDefs(*Mod, true);
  JIT_DEBUG(util::dumpModule(*Mod, "Module after optimization"));
  auto ClonedMod = llvm::CloneModule(*Mod);
  // Add to the JIT engine.
  CD.CJ->addModule(std::move(Mod));
  // Make new functions available for future instantiations.
  CD.makeDefsAvailable(std::move(ClonedMod));

  // Lookup the address of the generated function.
  auto SpecSymbol = CD.CJ->findSymbol(SMName);
  assert(SpecSymbol && "Can't find the specialization just generated?");

  if (auto Err = SpecSymbol.takeError()) {
    errs() << "JIT Error: " << Err << "\n";
    fatal();
  }

  if (!SpecSymbol.getAddress())
    fatal();

  auto* FPtr = (void *) llvm::cantFail(SpecSymbol.getAddress());
  return {FPtr, true};
}

}
}