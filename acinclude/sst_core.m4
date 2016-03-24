
AC_DEFUN([CHECK_SST_CORE], [

have_integrated_core="no"
AC_ARG_WITH([sst-core],
    AS_HELP_STRING([--with-sst-core@<:@=DIR@:>@],
        [Build shared library compatible with integrated SST core (optional).]
    ), [
      SST="$withval"
      have_integrated_core="yes"
    ], [
      AC_DEFINE_UNQUOTED([INTEGRATED_SST_CORE], 0, [Do not run on integrated SST core])
      AM_CONDITIONAL([INTEGRATED_SST_CORE], false)
      SUMI_CPPFLAGS=""
    ]
)

if test "X$have_integrated_core" = "Xyes"; then
  if test "X$HAVE_BOOST" = "Xyes"; then
    AC_MSG_ERROR([Please don't specify --with-boost when compiling for integrated core, core's boost is automatically used.])
  fi
fi

if test "X$have_integrated_core" = "Xyes"; then
    AC_CONFIG_FILES([skeletons/sst/env.sh skeletons/sst/config.py])
    AC_CONFIG_FILES([skeletons/sst/run], [chmod +x skeletons/sst/run])
    AC_CONFIG_FILES([bin/sstmac], [chmod +x bin/sstmac])
    AC_CONFIG_FILES([bin/sstmac-check], [chmod +x bin/sstmac-check])
    AC_CONFIG_FILES([tests/api/mpi/testexec], [chmod +x tests/api/mpi/testexec])
    AC_CONFIG_FILES([tests/api/globals/testexec], [chmod +x tests/api/globals/testexec])
    AC_DEFINE_UNQUOTED([INTEGRATED_SST_CORE], 1, [Run on integrated SST core])
    AC_SUBST([sst_prefix], "$SST")
    AM_CONDITIONAL([INTEGRATED_SST_CORE], true)
    SST_INCLUDES="-I$SST/include -I$SST/include/sst"
    SST_CPPFLAGS="-DSSTMAC_INTEGRATED_SST_CORE=1 $SST_INCLUDES"
    SAVE_CPPFLAGS="$CPPFLAGS"
    PY_INCLUDES="`python-config --includes`"
    SST_CPPFLAGS="$SST_CPPFLAGS $PY_INCLUDES"
    CPPFLAGS="$CPPFLAGS $SST_CPPFLAGS"
    AC_CHECK_HEADERS([Python.h], [],
        [AC_MSG_ERROR([Could not locate Python installation needed by SST core])])
    AC_CHECK_HEADERS([sst/core/element.h], [],
        [AC_MSG_ERROR([Could not locate SST core header files at $SST])])
    SUMI_CPPFLAGS="$SST_INCLUDES"
    AC_SUBST(SST_CPPFLAGS)
    CPPFLAGS="$SAVE_CPPFLAGS"

    # Already failed if user tried to specify --with-boost.  We insist on using whatever sst-core
    # was configured with.
    BOOST_CPPFLAGS="`$SST/bin/sst-config --BOOST_CPPFLAGS`"
    BOOST_LDFLAGS="`$SST/bin/sst-config --BOOST_LDFLAGS`"
    AC_SUBST(BOOST_CPPFLAGS)
    AC_SUBST(BOOST_LDFLAGS)
    AM_CONDITIONAL(EXTERNAL_BOOST, true)
fi

])

