ACLOCAL_AMFLAGS = -I m4

lib_LTLIBRARIES = libmodule_pd_preprocessor.la

libmodule_pd_preprocessor_la_SOURCES  = pd_preprocessor.cpp
libmodule_pd_preprocessor_la_CPPFLAGS = -Wall -I$(srcdir) -I$(builddir) \
                                    @ROBOTKERNEL_CFLAGS@
libmodule_pd_preprocessor_la_LDFLAGS  = -Wall -release $(PACKAGE_VERSION)

