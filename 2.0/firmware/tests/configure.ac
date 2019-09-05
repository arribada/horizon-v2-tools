AC_INIT([tests],[0.1.0],[support@icoteq.com])
AC_CONFIG_SRCDIR([gtest/Fs.cpp])

LT_INIT
AC_CONFIG_MACRO_DIR([m4])

# Enable all warnings and make them errors.
# Build .tar.bz2 source distributions by default.
# foreign will make it skip the GNU standard files.
AM_INIT_AUTOMAKE([foreign -Wgnu -Wobsolete -Woverride -Wsyntax -Wunsupported -Wall -Wno-portability dist-bzip2 parallel-tests])

# Setup the programs we need to build
AC_CANONICAL_BUILD
AC_CANONICAL_HOST
AC_PROG_CC
AC_PROG_RANLIB

dnl We need to find a compiler for compiling a program to run on the local host
dnl while building. It needs to be different from CC when cross-compiling.
dnl There is a macro called AC_PROG_CC_FOR_BUILD in the GNU archive for
dnl figuring this out automatically. Unfortunately, it does not work with the
dnl latest versions of autoconf. So for the moment, we just default to the
dnl same values as the "main" compiler. People who are cross-compiling will
dnl just have to adjust the Makefile by hand or set these values when they
dnl run "configure".
CC_FOR_BUILD=${CC_FOR_BUILD:-'cc'}
CFLAGS_FOR_BUILD=${CFLAGS_FOR_BUILD:-'$(CFLAGS)'}
CPPFLAGS_FOR_BUILD=${CPPFLAGS_FOR_BUILD:-'$(CPPFLAGS)'}
BUILD_EXEEXT=${BUILD_EXEEXT:-'$(EXEEXT)'}
BUILD_OBJEXT=${BUILD_OBJEXT:-'$(OBJEXT)'}

AC_SUBST(BUILD_EXEEXT)
AC_SUBST(BUILD_OBJEXT)
AC_SUBST(CC_FOR_BUILD)
AC_SUBST(CFLAGS_FOR_BUILD)
AC_SUBST(CPPFLAGS_FOR_BUILD)

# We need conditional compile
AC_PROG_CC_C_O
AM_PROG_CC_C_O

# If the user has googletest installed and has not passed --without-unittest
# then we will build with unit testing. This is not suitable for running on
# hardware so we warn about it at the end.
AC_PROG_CXX
AC_LANG_PUSH([C++])
AC_ARG_WITH([unittest],
            [AS_HELP_STRING([--without-unittest],
                            [Disable unittesting])],
            [],
            [with_unittest=yes])
AS_IF([test "x$with_unittest" == "xyes"],
      [UNITTEST_MISSING=0],
      [UNITTEST_MISSING=1])
AC_LANG_POP([C++])
AM_CONDITIONAL([BUILD_UNITTEST], [test "x$UNITTEST_MISSING" == "x0"])

# Check we have everything we need to build the application, disable it otherwise
MISSING_APP_REQ=0
AM_CONDITIONAL([BUILD_APP], [test "x$MISSING_APP_REQ" == "x0"])

AC_CONFIG_FILES([Makefile gtest/Makefile])
AC_OUTPUT

AS_IF([test "x$MISSING_APP_REQ" != "x0"],
      [echo ""]
      [AC_MSG_NOTICE([You are missing some headers or libraries needed to build
           the application. Check the warnings above for details])]
      [echo ""])

AS_IF([test "x$UNITTEST_MISSING" == "x0"],
      [echo ""]
      [AC_MSG_NOTICE([This build has unit testing enabled. This is not suitable
           for running on target hardware. If that was your intention
           make sure you are using the correct compiler or run configure
           with --without-unittest])]
      [echo ""])
