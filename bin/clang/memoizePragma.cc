/**
Copyright 2009-2019 National Technology and Engineering Solutions of Sandia,
LLC (NTESS).  Under the terms of Contract DE-NA-0003525, the U.S.  Government
retains certain rights in this software.

Sandia National Laboratories is a multimission laboratory managed and operated
by National Technology and Engineering Solutions of Sandia, LLC., a wholly
owned subsidiary of Honeywell International, Inc., for the U.S. Department of
Energy's National Nuclear Security Administration under contract DE-NA0003525.

Copyright (c) 2009-2019, NTESS

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.

    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Questions? Contact sst-macro-help@sandia.gov
*/

#include "memoizePragma.h"
#include "astMatchers.h"
#include "clangGlobals.h"
#include "memoizeVariableCaptureAnalyzer.h"
#include "util.h"

namespace {

std::string cleanPath(std::string const &p) {
  // Don't care if it doesn't work on Windows
  auto root_end = p.find_last_of('/') + 1;
  auto out = p.substr(root_end);
  for (auto &c : out) {
    if (c == '.') {
      c = '_';
    }
  }

  return out;
}

std::string generateUniqueFunctionName(clang::SourceLocation const &Loc,
                                       clang::NamedDecl const *Decl,
                                       std::string Prefix) {
  static int counter = 0;
  Prefix += (Prefix.empty() ? "f" : "") + std::to_string(counter++);
  Prefix += "_" + Decl->getNameAsString() + "_";

  auto &SM = CompilerGlobals::SM();
  std::string path = SM.getFilename(Loc).str();
  Prefix += cleanPath(path) + std::to_string(SM.getPresumedLineNumber(Loc)) +
            "_memoize";

  return Prefix;
}

auto parseKeyword(PragmaArgMap const &Strings, std::string const &Key) {
  using ContainerType = std::vector<std::string>;

  // If Key was in the arguments and it is not an empty list return it.
  if (auto Vars = Strings.find(Key);
      Vars != Strings.end() && not Vars->second.empty()) {
    return std::make_optional<ContainerType>(Vars->second.begin(),
                                             Vars->second.end());
  }

  return std::optional<ContainerType>();
}

template <typename StmtDecl, typename Container>
auto getAllExprs(StmtDecl const *SD,
                 std::optional<Container> const &VariableNames,
                 capture::AutoCapture ac) {

  std::string NameRegex =
      (VariableNames) ? ::matchers::makeNameRegex(*VariableNames) : "";

  return memoizationAutoMatcher(SD, NameRegex, ac);
}

template <typename Fn>
std::string
parseExprString(std::vector<memoize::ExpressionStrings> const &Exprs, Fn &&f) {
  std::string out;

  bool isFirst = true;
  for (auto const &Expr : Exprs) {
    std::string comma = (isFirst) ? "" : ", ";
    isFirst = false;

    out += comma + f(Expr);
  }

  return out;
}

std::string declare_start_function(
    std::string const &FuncName,
    std::vector<memoize::ExpressionStrings> const &ExprStrs) {

  return sst::strings::replace_all(
      R"func(
#ifdef __cplusplus
extern "C" {
#endif

void $name_start($args);

#ifdef __cplusplus
}
#endif
)func",
      {{"$name", FuncName},
       {"$args",
        parseExprString(ExprStrs, [](memoize::ExpressionStrings const &es) {
          return es.getSrcFileType();
        })}});
}

std::string
declare_end_function(std::string const &FuncName,
                     std::vector<memoize::ExpressionStrings> const &) {
  return sst::strings::replace_all(
      R"func(
#ifdef __cplusplus
extern "C" {
#endif

void $func_end();

#ifdef __cplusplus
}
#endif
)func",
      "$func", FuncName);
}

std::string
start_call_site(std::string const &FuncName,
                std::vector<memoize::ExpressionStrings> const &ExprStrs) {
  auto call_args =
      parseExprString(ExprStrs, [](memoize::ExpressionStrings const &es) {
        return es.getExprSpelling();
      });

  return sst::strings::replace_all("$func_start($args);",
                                   {{"$func", FuncName}, {"$args", call_args}});
};

std::string
start_definition(std::string const &FuncName,
                 std::vector<memoize::ExpressionStrings> const &ExprStrs,
                 bool isOMPPragma) {

  auto TemplateArgs =
      parseExprString(ExprStrs, [](memoize::ExpressionStrings const &es) {
        auto type = es.getCppType();
        if (type == "const char *") { // Can't capture const char *
          type = "std::string";
        }
        return type;
      });

  auto FunctionParams = parseExprString(
      ExprStrs, [var = 0](memoize::ExpressionStrings const &es) mutable {
        auto v = es.getCppType() + " v" + std::to_string(var++);
        return v;
      });

  auto CaptureArgs = parseExprString(
      ExprStrs, [var = 0](memoize::ExpressionStrings const &es) mutable {
        auto v = "v" + std::to_string(var++);
        return v;
      });

  if (isOMPPragma) {
    TemplateArgs += ",int,int,int";
    CaptureArgs +=
        ",omp_get_max_threads(),omp_get_proc_bind(),omp_get_num_places()";
  }

  return sst::strings::replace_all(
      R"func(
extern "C"
void $FN_start($params){
  std::call_once($FN_capture_flag,
    []{$FN_capture_var = memoize::getCaptureType<$temp_args>("$FN");});
  $FN_capture_var->capture_start("$FN", $args);
}
)func",
      {{"$FN", FuncName},
       {"$params", FunctionParams},
       {"$temp_args", TemplateArgs},
       {"$args", CaptureArgs}});
}

std::string end_definition(std::string const &FuncName,
                           std::vector<memoize::ExpressionStrings> const &) {

  auto captureVarName = FuncName + "_capture_var";
  return sst::strings::replace_all(
      "extern \"C\" void $FN_end(){$CV->capture_stop(\"$FN\");}",
      {{"$FN", FuncName}, {"$CV", captureVarName}});
}

std::string end_call_site(std::string const &FuncName,
                          std::vector<memoize::ExpressionStrings> const &) {
  return FuncName + "_end();";
}

std::string write_static_capture_variable(
    std::string const &FuncName,
    std::vector<memoize::ExpressionStrings> const &ExprStrs, bool isOMPPragma) {

  std::string argList =
      parseExprString(ExprStrs, [](memoize::ExpressionStrings const &es) {
        auto type = es.getCppType();
        if (type == "const char *") {
          type = "std::string";
        }
        return type;
      });
  if (isOMPPragma) {
    // omp_num_threads, omp_proc_bind
    argList += ",int,int,int";
  }

  std::string out;
  llvm::raw_string_ostream os(out);

  os << "std::once_flag " << FuncName + "_capture_flag;\n";
  os << "static memoize::Capture<" << argList << "> * "
     << FuncName + "_capture_var = nullptr;";

  return out;
}

bool removedKeyword(std::vector<std::string> &strs, char const *keyword) {
  if (auto ac = std::find(strs.begin(), strs.end(), keyword);
      ac != strs.end()) {
    strs.erase(ac);
    return true;
  }

  return false;
}

capture::AutoCapture
parseCaptureOptions(std::optional<std::vector<std::string>> &VariableNames) {
  if (VariableNames) { // User provided specific variables to capture
    if (removedKeyword(*VariableNames, "auto")) {
      return capture::AutoCapture::AllConditions;
    } else if (removedKeyword(*VariableNames, "for")) {
      return capture::AutoCapture::ForLoopConditions;
    } else { // No special capture keywords don't do auto capture
      return capture::AutoCapture::None;
    }
  }

  return capture::AutoCapture::AllConditions;
}
} // namespace

namespace memoize {

SSTMemoizePragma::SSTMemoizePragma(clang::SourceLocation loc,
                                   PragmaArgMap &&PragmaStrings)
    : VariableNames_(parseKeyword(PragmaStrings, "variables")),
      ExtraExpressions_(parseKeyword(PragmaStrings, "extra_exressions")),
      DoAutoCapture_(parseCaptureOptions(VariableNames_)) {

  /* The following expressions are there to provide places for OPT to replace
     the captures with more information, unfortunately adding more information
     will require modifying the opt plugin, this file, and the capture file.
     TODO One day we can try to figure out a sane way to fix this.
   */
  auto &ctx = CompilerGlobals::ASTContext();
  // IR Info
  llvm::APInt api(64, -1);
  auto ilit = clang::IntegerLiteral::Create(ctx, api, ctx.IntTy, loc);
  ExprStrs_.emplace_back(ilit); // Stores
  ExprStrs_.emplace_back(ilit); // Loads

  auto ConstChar = clang::QualType(ctx.CharTy);
  ConstChar.addConst();

  auto StrTy = ctx.getPointerType(ConstChar);

  clang::StringRef strRef("\"unknown\"");
  auto target = clang::StringLiteral::Create(
      ctx, strRef, clang::StringLiteral::StringKind::Ascii, false, StrTy, loc);
  ExprStrs_.emplace_back(target); // target
  ExprStrs_.emplace_back(target); // omp_region_attributes
}

ExpressionStrings::ExpressionStrings(clang::Expr const *e)
    : spelling(getStmtSpelling(e)) {
  if (CompilerGlobals::LangOpts().CPlusPlus) {
    cppType = getExprDesugaredTypeSpelling(e);
  } else {
    cType = getExprDesugaredTypeSpelling(e);

    auto cppLO = clang::LangOptions();
    cppLO.CPlusPlus = true;
    cppType = getExprDesugaredTypeSpelling(e, &cppLO);
  }
}
std::string ExpressionStrings::getExpressionLabel() const {
  return "\"" + cType.value_or(cppType) + " " + spelling + "\"";
}
std::string ExpressionStrings::getSrcFileType() const {
  return cType.value_or(cppType);
}
std::string const &ExpressionStrings::getCppType() const { return cppType; }
std::string const &ExpressionStrings::getExprSpelling() const {
  return spelling;
}

void SSTMemoizePragma::activate(clang::Stmt *S) {
  for (auto Expr : getAllExprs(S, VariableNames_, DoAutoCapture_)) {
    ExprStrs_.emplace_back(Expr);
  }

  auto ParentDecl = getNonNull(matchers::getParentDecl(S));
  auto FuncName = generateUniqueFunctionName(getStart(S), ParentDecl, "");

  static_capture_decl_ =
      write_static_capture_variable(FuncName, ExprStrs_, isOMP());
  start_capture_definition_ = start_definition(FuncName, ExprStrs_, isOMP());
  stop_capture_definition_ = end_definition(FuncName, ExprStrs_);

  auto &R = CompilerGlobals::rewriter;
  R.InsertTextBefore(getStart(ParentDecl),
                     declare_start_function(FuncName, ExprStrs_) + "\n");
  R.InsertTextBefore(getStart(ParentDecl),
                     declare_end_function(FuncName, ExprStrs_) + "\n");

  R.InsertTextBefore(pragmaDirectiveLoc,
                     start_call_site(FuncName, ExprStrs_) + "\n");
  if(auto FS = llvm::dyn_cast<clang::ForStmt>(S)){
  }
  R.InsertTextAfterToken(getEnd(S), end_call_site(FuncName, ExprStrs_) + "\n");
}

void SSTMemoizePragma::activate(clang::Decl *D) {}

void SSTMemoizePragma::deactivate() {
  std::string headers = R"lit(
  #include "capture.h"
  #if defined(_OPENMP)
  #include <omp.h>
  #else
  #define omp_get_max_threads() 1
  #define omp_get_proc_bind() 0
  #define omp_get_num_places() 1
  #endif
  )lit";

  auto &vec = CompilerGlobals::toolInfoRegistration.globalCppFunctionsToWrite;
  auto pragma = static_cast<SSTPragma *>(this);

  if (std::none_of(vec.begin(), vec.end(), [&](auto const &pragma_string) {
        return pragma_string.second == headers;
      })) {
    vec.push_back(std::make_pair(this, headers));
  }

  vec.push_back(std::make_pair(this, static_capture_decl_ + "\n"));
  vec.push_back(std::make_pair(this, start_capture_definition_ + "\n"));
  vec.push_back(std::make_pair(this, stop_capture_definition_ + "\n"));
}
} // namespace memoize

static PragmaRegister<SSTArgMapPragmaShim, memoize::SSTMemoizePragma, true>
    memoizePragma("sst", "memoize", modes::MEMOIZE);

static PragmaRegister<SSTArgMapPragmaShim, memoize::SSTMemoizeOMPPragma, false>
    ompMemoizePragma("omp", "parallel", modes::MEMOIZE);
