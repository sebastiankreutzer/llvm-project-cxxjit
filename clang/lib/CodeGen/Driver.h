//
// Created by sebastian on 08.11.19.
//

#ifndef CLANG_DRIVER_H
#define CLANG_DRIVER_H

#include "JIT.h"


namespace clang {
namespace jit {

class TemplateInstantiationHelper {
public:
  TemplateInstantiationHelper(CompilerData& CD,
                                 unsigned Idx) : CD(CD), Idx(Idx)
  {
    FD = CD.FuncMap[Idx];

    if (!FD)
      fatal();

  }

  template<typename SpecialArgHandler>
  void processTemplateArgs(const void *NTTPValues, const char **TypeStrings, SmallVectorImpl<TemplateArgument>& ArgList, SpecialArgHandler&& HandleSpecialArg);

  std::string instantiate(const SmallVectorImpl<TemplateArgument>& ArgList);

  std::unique_ptr<Module> emitModule();

private:
  std::string instantiate(CompilerData* TargetCD, const SmallVectorImpl<TemplateArgument>& ArgList);

private:
  CompilerData& CD;
  unsigned Idx;
  FunctionDecl* FD;
};

class Driver {
public:
  Driver(CompilerData& CD) : CD(CD)/*, UseFastLookup(false)*/ {};

  virtual InstData resolve(const ThisInstInfo& Inst, unsigned Idx) = 0;

//  // If this returns true and there is an existing entry for the instantiation, the call to resolve should be skipped
//  // and the existing version should be used.
//  bool shouldUseFastLookup() {
//    return UseFastLookup;
//  }
//
//  void setUseFastLookup(bool Fast) {
//    this->UseFastLookup = Fast;
//  }

protected:
  CompilerData& CD;

//private:
//  bool UseFastLookup;
};

class SimpleDriver: public Driver {
public:
  SimpleDriver(CompilerData& CD) : Driver(CD) {
//    setUseFastLookup(true);
  };

  InstData resolve(const ThisInstInfo& Inst, unsigned Idx) override;

private:
  std::string instantiateTemplate(const void *NTTPValues, const char **TypeStrings,
                                                unsigned Idx);

};

template<typename SpecialArgHandler>
void TemplateInstantiationHelper::processTemplateArgs(const void *NTTPValues, const char **TypeStrings, SmallVectorImpl<TemplateArgument>& ArgList, SpecialArgHandler&& HandleSpecialArg) {

  auto& Ctx = CD.Ctx;


  RecordDecl *RD =
      Ctx->buildImplicitRecord(llvm::Twine("__clang_jit_args_")
                                   .concat(llvm::Twine(Idx))
                                   .concat(llvm::Twine("_t"))
                                   .str());

  RD->startDefinition();

  enum TASaveKind {
    TASK_None,
    TASK_Type,
    TASK_Value
  };

  SmallVector<TASaveKind, 8> TAIsSaved;

  auto *FTSI = FD->getTemplateSpecializationInfo();
  for (auto &TA : FTSI->TemplateArguments->asArray()) {
    auto HandleTA = [&](const TemplateArgument &TA) {
      if (TA.getKind() == TemplateArgument::Type)
        if (TA.getAsType()->isJITFromStringType()) {
          TAIsSaved.push_back(TASK_Type);
          return;
        }

      if (TA.getKind() != TemplateArgument::Expression) {
        TAIsSaved.push_back(TASK_None);
        return;
      }

      SmallVector<PartialDiagnosticAt, 8> Notes;
      Expr::EvalResult Eval;
      Eval.Diag = &Notes;
      if (TA.getAsExpr()->
          EvaluateAsConstantExpr(Eval, Expr::EvaluateForMangling, *Ctx)) {
        TAIsSaved.push_back(TASK_None);
        return;
      }

      QualType FieldTy = TA.getNonTypeTemplateArgumentType();
      auto *Field = FieldDecl::Create(
          *Ctx, RD, SourceLocation(), SourceLocation(), /*Id=*/nullptr,
          FieldTy, Ctx->getTrivialTypeSourceInfo(FieldTy, SourceLocation()),
          /*BW=*/nullptr, /*Mutable=*/false, /*InitStyle=*/ICIS_NoInit);
      Field->setAccess(AS_public);
      RD->addDecl(Field);

      TAIsSaved.push_back(TASK_Value);
    };

    if (TA.getKind() == TemplateArgument::Pack) {
      for (auto &PTA : TA.getPackAsArray())
        HandleTA(PTA);
      continue;
    }

    HandleTA(TA);
  }

  RD->completeDefinition();
  RD->addAttr(PackedAttr::CreateImplicit(*Ctx));

  const ASTRecordLayout &RLayout = Ctx->getASTRecordLayout(RD);
  assert(Ctx->getCharWidth() == 8 && "char is not 8 bits!");

  QualType RDTy = Ctx->getRecordType(RD);
  auto Fields = cast<RecordDecl>(RDTy->getAsTagDecl())->field_begin();

  auto& PP = CD.PP;
  auto& S = CD.S;
  auto& CSFuncMap = CD.CSFuncMap;
  auto& Consumer = CD.Consumer;
  auto& NewLocalSymDecls = CD.NewLocalSymDecls;

  unsigned TAIdx = 0, TSIdx = 0;
  for (auto &TA : FTSI->TemplateArguments->asArray()) {
    auto HandleTA = [&](const TemplateArgument &TA,
                        SmallVectorImpl<TemplateArgument> &Builder) {
      if (TAIsSaved[TAIdx] == TASK_Type) {
        PP->ResetForJITTypes();

        PP->setPredefines(TypeStrings[TSIdx]);
        PP->EnterMainSourceFile();

        Parser P(*PP, *S, /*SkipFunctionBodies*/true, /*JITTypes*/true);

        // Reset this to nullptr so that when we call
        // Parser::Initialize it has the clean slate it expects.
        S->CurContext = nullptr;

        P.Initialize();

        Sema::ContextRAII TUContext(*S, Ctx->getTranslationUnitDecl());

        auto CSFMI = CSFuncMap.find(Idx);
        if (CSFMI != CSFuncMap.end()) {
          // Note that this restores the context of the function in which the
          // template was instantiated, but not the state *within* the
          // function, so local types will remain unavailable.

          auto *FunD = CSFMI->second;
          CD.restoreFuncDeclContext(FunD);
          S->CurContext = S->getContainingDC(FunD);
        }

        TypeResult TSTy = P.ParseTypeName();
        if (TSTy.isInvalid())
          fatal();

        QualType TypeFromString = Sema::GetTypeFromParser(TSTy.get());
        TypeFromString = Ctx->getCanonicalType(TypeFromString);

        Builder.push_back(TemplateArgument(TypeFromString));

        ++TSIdx;
        ++TAIdx;
        return;
      }

      if (TAIsSaved[TAIdx++] != TASK_Value) {
        Builder.push_back(TA);
        return;
      }

      assert(TA.getKind() == TemplateArgument::Expression &&
             "Only expressions template arguments handled here");

      QualType FieldTy = TA.getNonTypeTemplateArgumentType();

      assert(!FieldTy->isMemberPointerType() &&
             "Can't handle member pointers here without ABI knowledge");

      auto *Fld = *Fields++;
      unsigned Offset = RLayout.getFieldOffset(Fld->getFieldIndex()) / 8;
      unsigned Size = Ctx->getTypeSizeInChars(FieldTy).getQuantity();

      unsigned NumIntWords = llvm::alignTo<8>(Size);
      SmallVector<uint64_t, 2> IntWords(NumIntWords, 0);
      std::memcpy((char *) IntWords.data(),
                  ((const char *) NTTPValues) + Offset, Size);
      llvm::APInt IntVal(Size*8, IntWords);

      QualType CanonFieldTy = Ctx->getCanonicalType(FieldTy);

      llvm::Optional<TemplateArgument> SpecialArg = HandleSpecialArg(CanonFieldTy, TAIdx-1, IntWords); // TODO: Index correct?
      if (SpecialArg) {
        Builder.push_back(SpecialArg.getValue());
      } else if (FieldTy->isIntegralOrEnumerationType()) {
        llvm::APSInt SIntVal(IntVal,
                             FieldTy->isUnsignedIntegerOrEnumerationType());
        Builder.push_back(TemplateArgument(*Ctx, SIntVal, CanonFieldTy));
      } else {
        assert(FieldTy->isPointerType() || FieldTy->isReferenceType() ||
               FieldTy->isNullPtrType());
        if (IntVal.isNullValue()) {
          Builder.push_back(TemplateArgument(CanonFieldTy, /*isNullPtr*/true));
        } else {
          // Note: We always generate a new global for pointer values here.
          // This provides a new potential way to introduce an ODR violation:
          // If you also generate an instantiation using the same pointer value
          // using some other symbol name, this will generate a different
          // instantiation.

          // As we guarantee that the template parameters are not allowed to
          // point to subobjects, this is useful for optimization because each
          // of these resolve to distinct underlying objects.

          llvm::SmallString<256> GlobalName("__clang_jit_symbol_");
          IntVal.toString(GlobalName, 16, false);

          // To this base name we add the mangled type. Stack/heap addresses
          // can be reused with variables of different type, and these should
          // have different names even if they share the same address;
          auto &CGM = Consumer->getCodeGenerator()->CGM();
          llvm::raw_svector_ostream MOut(GlobalName);
          CGM.getCXXABI().getMangleContext().mangleTypeName(CanonFieldTy, MOut);

          auto NLDSI = NewLocalSymDecls.find(GlobalName);
          if (NLDSI != NewLocalSymDecls.end()) {
            Builder.push_back(TemplateArgument(NLDSI->second, CanonFieldTy));
          } else {
            Sema::ContextRAII TUContext(*S, Ctx->getTranslationUnitDecl());
            SourceLocation Loc = FTSI->getPointOfInstantiation();

            QualType STy = CanonFieldTy->getPointeeType();
            auto &II = PP->getIdentifierTable().get(GlobalName);

            if (STy->isFunctionType()) {
              auto *TAFD =
                  FunctionDecl::Create(*Ctx, S->CurContext, Loc, Loc, &II,
                                       STy, /*TInfo=*/nullptr, SC_Extern, false,
                                       STy->isFunctionProtoType());
              TAFD->setImplicit();

              if (const FunctionProtoType *FT = dyn_cast<FunctionProtoType>(STy)) {
                SmallVector<ParmVarDecl*, 16> Params;
                for (unsigned i = 0, e = FT->getNumParams(); i != e; ++i) {
                  ParmVarDecl *Parm =
                      ParmVarDecl::Create(*Ctx, TAFD, SourceLocation(), SourceLocation(),
                                          nullptr, FT->getParamType(i), /*TInfo=*/nullptr,
                                          SC_None, nullptr);
                  Parm->setScopeInfo(0, i);
                  Params.push_back(Parm);
                }

                TAFD->setParams(Params);
              }

              NewLocalSymDecls[II.getName()] = TAFD;
              Builder.push_back(TemplateArgument(TAFD, CanonFieldTy));
            } else {
              bool MadeArray = false;
              auto *TPL = FTSI->getTemplate()->getTemplateParameters();
              if (TPL->size() >= TAIdx) {
                auto *Param = TPL->getParam(TAIdx-1);
                if (NonTypeTemplateParmDecl *NTTP =
                    dyn_cast<NonTypeTemplateParmDecl>(Param)) {
                  QualType OrigTy = NTTP->getType()->getPointeeType();
                  OrigTy = OrigTy.getDesugaredType(*Ctx);

                  bool IsArray = false;
                  llvm::APInt Sz;
                  QualType ElemTy;
                  if (const auto *DAT = dyn_cast<DependentSizedArrayType>(OrigTy)) {
                    Expr* SzExpr = DAT->getSizeExpr();

                    // Get the already-processed arguments for potential substitution.
                    auto *NewTAL = TemplateArgumentList::CreateCopy(*Ctx, Builder);
                    MultiLevelTemplateArgumentList SubstArgs(*NewTAL);

                    SmallVector<Expr *, 1> NewSzExprVec;
                    if (!S->SubstExprs(SzExpr, /*IsCall*/ false, SubstArgs, NewSzExprVec)) {
                      Expr::EvalResult NewSzResult;
                      if (NewSzExprVec[0]->EvaluateAsInt(NewSzResult, *Ctx)) {
                        Sz = NewSzResult.Val.getInt();
                        ElemTy = DAT->getElementType();
                        IsArray = true;
                      }
                    }
                  } else if (const auto *CAT = dyn_cast<ConstantArrayType>(OrigTy)) {
                    Sz = CAT->getSize();
                    ElemTy = CAT->getElementType();
                    IsArray = true;
                  }

                  if (IsArray && (ElemTy->isIntegerType() ||
                                  ElemTy->isFloatingType())) {
                    QualType ArrTy =
                        Ctx->getConstantArrayType(ElemTy,
                                                  Sz, nullptr, clang::ArrayType::Normal, 0);

                    SmallVector<Expr *, 16> Vals;
                    unsigned ElemSize = Ctx->getTypeSizeInChars(ElemTy).getQuantity();
                    unsigned ElemNumIntWords = llvm::alignTo<8>(ElemSize);
                    const char *Elem = (const char *) IntVal.getZExtValue();
                    for (unsigned i = 0; i < Sz.getZExtValue(); ++i) {
                      SmallVector<uint64_t, 2> ElemIntWords(ElemNumIntWords, 0);

                      std::memcpy((char *) ElemIntWords.data(), Elem, ElemSize);
                      Elem += ElemSize;

                      llvm::APInt ElemVal(ElemSize*8, ElemIntWords);
                      if (ElemTy->isIntegerType()) {
                        Vals.push_back(new (*Ctx) IntegerLiteral(
                            *Ctx, ElemVal, ElemTy, Loc));
                      } else {
                        llvm::APFloat ElemValFlt(Ctx->getFloatTypeSemantics(ElemTy), ElemVal);
                        Vals.push_back(FloatingLiteral::Create(*Ctx, ElemValFlt,
                                                               false, ElemTy, Loc));
                      }
                    }

                    InitListExpr *InitL = new (*Ctx) InitListExpr(*Ctx, Loc, Vals, Loc);
                    InitL->setType(ArrTy);

                    auto *TAVD =
                        VarDecl::Create(*Ctx, S->CurContext, Loc, Loc, &II,
                                        ArrTy, Ctx->getTrivialTypeSourceInfo(ArrTy, Loc),
                                        SC_Extern);
                    TAVD->setImplicit();
                    TAVD->setConstexpr(true);
                    TAVD->setInit(InitL);

                    NewLocalSymDecls[II.getName()] = TAVD;
                    Builder.push_back(TemplateArgument(TAVD, Ctx->getLValueReferenceType(ArrTy)));

                    MadeArray = true;
                  }
                }
              }

              if (!MadeArray) {
                auto *TAVD =
                    VarDecl::Create(*Ctx, S->CurContext, Loc, Loc, &II,
                                    STy, Ctx->getTrivialTypeSourceInfo(STy, Loc),
                                    SC_Extern);
                TAVD->setImplicit();

                NewLocalSymDecls[II.getName()] = TAVD;
                Builder.push_back(TemplateArgument(TAVD, CanonFieldTy));
              }
            }

            CD.LocalSymAddrs[II.getName()] = (const void *) IntVal.getZExtValue();
          }
        }
      }
    };

    if (TA.getKind() == TemplateArgument::Pack) {
      SmallVector<TemplateArgument, 8> PBuilder;
      for (auto &PTA : TA.getPackAsArray())
        HandleTA(PTA, PBuilder);
      ArgList.push_back(TemplateArgument::CreatePackCopy(*Ctx, PBuilder));
      continue;
    }

    HandleTA(TA, ArgList);
  }
}


}
}


#endif //CLANG_DRIVER_H
