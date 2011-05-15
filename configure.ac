AC_PREREQ([2.63])

AC_INIT(nox, 0.0.1, [], noxe)
AM_INIT_AUTOMAKE([])

AC_PROG_CXX

AC_CHECK_PROG(NASM, nasm, yes,  no)
if test "$NASM" = "no"; then
    AC_MSG_ERROR([nasm not found!])
fi

AC_CHECK_HEADERS(linux/kvm.h)

PKG_CHECK_MODULES(X11, x11)
AC_SUBST(X11_CFLAGS)
AC_SUBST(X11_LIBS)

AC_CONFIG_FILES(Makefile)
AC_OUTPUT

