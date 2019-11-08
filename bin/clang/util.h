/**
Copyright 2009-2018 National Technology and Engineering Solutions of Sandia, 
LLC (NTESS).  Under the terms of Contract DE-NA-0003525, the U.S.  Government 
retains certain rights in this software.

Sandia National Laboratories is a multimission laboratory managed and operated
by National Technology and Engineering Solutions of Sandia, LLC., a wholly 
owned subsidiary of Honeywell International, Inc., for the U.S. Department of 
Energy's National Nuclear Security Administration under contract DE-NA0003525.

Copyright (c) 2009-2018, NTESS

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

#ifndef bin_clang_util_h
#define bin_clang_util_h

#include "clangHeaders.h"
#include <iostream>

struct PrettyPrinter
{
  clang::LangOptions LangOpts;
  clang::PrintingPolicy Policy;
  llvm::raw_string_ostream os;
  PrettyPrinter() : Policy(LangOpts), os(baseStr)
  {
    LangOpts.CPlusPlus = true;
  }

  ~PrettyPrinter(){
    //must be flushed
    //prior to deleting string
    os.flush();
  }

  void print(const clang::Stmt* s){
    s->printPretty(os, nullptr, Policy);
  }

  void print(const clang::Decl* d){
    d->print(os, Policy);
  }

  void print(clang::QualType ty){
    ty.print(os,Policy);
  }

  std::string print(const clang::BuiltinType* ty){
    return ty->getName(Policy).str();
  }

  void dump(std::ostream& sos = std::cout){
    sos << os.str() << std::endl;
  }

  std::string str() {
    return os.str();
  }

 private:
  std::string baseStr;
};

template <typename T> 
T getNonNull(T t) {
  static_assert(std::is_pointer<T>::value, 
                "getNonNull expects pointer arguments");

  if (t == nullptr) {
    llvm::errs() << "Tried to access null pointer in getNonNull\n";
    exit(EXIT_FAILURE);
  }

  return t;
}

/** getStmtSpelling will print the stmt in a way that is consistent with the
 * language specified by the second argument.  If default or nullptr is passed
 * is uses the globally stored language option.
 **/
std::string getStmtSpelling(clang::Stmt const *s, 
                            clang::LangOptions const* = nullptr); 

/** getExprDesugaredTypeSpelling will return a string with the type of the
 * expresision in a way that is consistent with the language specified by the
 * second argument.  If default or nullptr is passed is uses the globally
 * stored language option.
 **/
std::string getExprDesugaredTypeSpelling(clang::Expr const *e, 
                                         clang::LangOptions const* = nullptr); 


bool isCxx(const std::string& filename);
bool isValidSrc(const std::string& filename);

void internalError(const std::string& error);

void internalError(clang::SourceLocation loc, clang::CompilerInstance& CI, const std::string& error);
void errorAbort(clang::SourceLocation loc, clang::CompilerInstance& CI, const std::string& error);
void warn(clang::SourceLocation loc, clang::CompilerInstance& CI, const std::string& warning);

void warn(const clang::Stmt* s, clang::CompilerInstance& CI, const std::string& error);
void warn(const clang::Decl* decl, clang::CompilerInstance& CI, const std::string& error);

void errorAbort(const clang::Stmt* s, clang::CompilerInstance& CI, const std::string& error);
void errorAbort(const clang::Decl* decl, clang::CompilerInstance& CI, const std::string& error);

void internalError(const clang::Stmt* s, clang::CompilerInstance& CI, const std::string& error);
void internalError(const clang::Decl* decl, clang::CompilerInstance& CI, const std::string& error);

void insertBefore(const clang::Stmt* s, clang::Rewriter& r, const std::string& text);
void insertAfter(const clang::Stmt* s, clang::Rewriter& r, const std::string& text);

inline bool operator<=(const clang::SourceLocation &LHS, const clang::SourceLocation &RHS) {
  return LHS < RHS || LHS == RHS;
}

inline bool operator>(const clang::SourceLocation &LHS, const clang::SourceLocation &RHS) {
  return !(LHS < RHS) && !(LHS == RHS);
}

inline bool operator>=(const clang::SourceLocation &LHS, const clang::SourceLocation &RHS) {
  return !(LHS < RHS);
}

std::string makeCxxName(const std::string& name);

void replace(clang::SourceRange rng, clang::Rewriter& r,
             const std::string& repl, clang::CompilerInstance& CI);

void replace(const clang::Stmt* s, clang::Rewriter& r,
             const std::string& repl, clang::CompilerInstance& CI);

void replace(const clang::Decl* d, clang::Rewriter& r,
             const std::string& repl, clang::CompilerInstance& CI);

static inline std::string getString(clang::SourceLocation loc, clang::CompilerInstance& CI){
  return loc.printToString(CI.getSourceManager());
}

static inline std::string getStartLocString(const clang::Expr* e, clang::CompilerInstance& CI){
  return getString(getStart(e), CI);
}

static inline std::string getStartLocString(const clang::Decl* d, clang::CompilerInstance& CI){
  return getString(getStart(d), CI);
}

#endif
