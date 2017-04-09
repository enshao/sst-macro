#include "frontendActions.h"
#include "globalVarNamespace.h"
#include "pragmas.h"
#include <sstream>
#include <fstream>
#include <iostream>

using namespace clang;
using namespace clang::driver;
using namespace clang::tooling;

ReplaceAction::ReplaceAction() :
  visitor_(rewriter_, globalNs_, deletedExprs_)
{
}

#define scase(type,s,pp) \
  case(clang::Stmt::type##Class): \
    return visit##type(clang::cast<type>(s),pp)

bool
ReplaceAction::BeginSourceFileAction(CompilerInstance &CI, llvm::StringRef Filename)
{
  ci_ = &CI;
  return true;
}

class DeleteOpenMPPragma : public PragmaHandler
{
 public:
  DeleteOpenMPPragma() : PragmaHandler("omp") {}
  void HandlePragma(Preprocessor &PP, PragmaIntroducerKind Introducer, Token &FirstToken) override {}
};

void
ReplaceAction::ExecuteAction()
{
  if (!ci_->hasSema()) ci_->createSema(getTranslationUnitKind(), nullptr);

  TranslationUnitKind TUKind = TU_Complete;
  ASTContext& Ctx = ci_->getASTContext();
  CodeCompleteConsumer* CompletionConsumer = nullptr;
  ASTConsumer& Consumer = ci_->getASTConsumer();
  Sema& S = ci_->getSema();

  //bool PrintStats = false;
  // Also turn on collection of stats inside of the Sema object.
  //bool OldCollectStats = PrintStats;
  //std::swap(OldCollectStats, S.CollectStats);

  bool SkipFunctionBodies = false;
  Parser P(S.getPreprocessor(), S, SkipFunctionBodies);

  //okay, super annoying - I have to DELETE the openmp handlers
  DeleteOpenMPPragma deleter; ci_->getPreprocessor().RemovePragmaHandler(&deleter);
  ci_->getPreprocessor().AddPragmaHandler("omp", new SSTOpenMPParallelPragmaHandler(
                     visitor_.getPragmas(), *ci_, visitor_, deletedExprs_)); //and put it back

  S.getPreprocessor().EnterMainSourceFile();
  P.Initialize();

  Parser::DeclGroupPtrTy ADecl;
  ExternalASTSource *External = S.getASTContext().getExternalSource();
  if (External)
    External->StartTranslationUnit(&Consumer);

  for (bool AtEOF = P.ParseFirstTopLevelDecl(ADecl); !AtEOF;
       AtEOF = P.ParseTopLevelDecl(ADecl)) {
    // If we got a null return and something *was* parsed, ignore it.  This
    // is due to a top-level semicolon, an action override, or a parse error
    // skipping something.
    if (ADecl && !Consumer.HandleTopLevelDecl(ADecl.get()))
      return;
  }

  // Process any TopLevelDecls generated by #pragma weak.
  for (Decl *D : S.WeakTopLevelDecls())
    Consumer.HandleTopLevelDecl(DeclGroupRef(D));

  Consumer.HandleTranslationUnit(S.getASTContext());
}

void
ReplaceAction::initPragmas(CompilerInstance& CI)
{
  CI.getPreprocessor().AddPragmaHandler("sst",
    new SSTDeletePragmaHandler(visitor_.getPragmas(), CI, visitor_, deletedExprs_));
  CI.getPreprocessor().AddPragmaHandler("sst",
    new SSTMallocPragmaHandler(visitor_.getPragmas(), CI, visitor_, deletedExprs_));
  CI.getPreprocessor().AddPragmaHandler("sst",
    new SSTNewPragmaHandler(visitor_.getPragmas(), CI, visitor_, deletedExprs_));
  CI.getPreprocessor().AddPragmaHandler("sst",
    new SSTComputePragmaHandler(visitor_.getPragmas(), CI, visitor_, deletedExprs_));
  CI.getPreprocessor().AddPragmaHandler("sst",
    new SSTReplacePragmaHandler(visitor_.getPragmas(), CI, visitor_, deletedExprs_));
  CI.getPreprocessor().AddPragmaHandler("sst",
    new SSTStartReplacePragmaHandler(visitor_.getPragmas(), CI, visitor_, deletedExprs_));
  CI.getPreprocessor().AddPragmaHandler("sst",
    new SSTStopReplacePragmaHandler(visitor_.getPragmas(), CI, visitor_, deletedExprs_));
}

void
ReplaceAction::EndSourceFileAction()
{
  SourceManager &SM = rewriter_.getSourceMgr();
  std::string sourceFile = SM.getFileEntryForID(SM.getMainFileID())->getName().str();
  std::string sstSourceFile, sstGlobalFile;
  std::size_t lastSlashPos = sourceFile.find_last_of("/");
  if (lastSlashPos == std::string::npos){
    sstSourceFile = "sst." + sourceFile;
    sstGlobalFile = "sstGlobals." + sourceFile + ".cpp";
  } else {
    lastSlashPos++;
    sstSourceFile = sourceFile.substr(0, lastSlashPos) + "sst." + sourceFile.substr(lastSlashPos);
    sstGlobalFile = sourceFile.substr(0, lastSlashPos) + "sstGlobals." + sourceFile.substr(lastSlashPos) + ".cpp";
  }

  std::error_code rc;
  llvm::raw_fd_ostream fs(sstSourceFile, rc, llvm::sys::fs::F_RW);
  rewriter_.getEditBuffer(rewriter_.getSourceMgr().getMainFileID()).write(fs);
  fs.close();



  std::ofstream ofs(sstGlobalFile.c_str());
  if (ofs.good()){
    //add the header files needed
    ofs << "#include <sstmac/software/process/global.h>\n\n";
    globalNs_.genSSTCode(ofs,"");
    if (visitor_.hasCStyleMain()){
      const char* appname = getenv("SSTMAC_APP_NAME");
      if (appname == nullptr){
        llvm::errs() << "Cannot refactor main function unless SSTMAC_APP_NAME environment var is defined\n";
        exit(EXIT_FAILURE);
      }
      ofs << "int user_skeleton_main_init_fxn(const char* name, int (*foo)(int,char**));\n"
         << "extern \"C\" int sstmac_user_main_" << appname << "(int argc, char** argv);\n"
         << "static int dont_ignore_this = user_skeleton_main_init_fxn("
           << "\"" << appname << "\",sstmac_user_main_" << appname << ");\n\n";
    }
  } else {
    llvm::errs() << "Failed opening " << sstGlobalFile << "\n";
    exit(EXIT_FAILURE);
  }
  ofs.close();
}
