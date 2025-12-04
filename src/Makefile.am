ACLOCAL_AMFLAGS = -I m4

lib_LTLIBRARIES = libmodule_pdpreprocessor.la

libmodule_pdpreprocessor_la_SOURCES  = pdpreprocessor.cpp
libmodule_pdpreprocessor_la_CPPFLAGS = -Wall -I$(srcdir) -I$(builddir) \
                                    @ROBOTKERNEL_CFLAGS@ @SERVICE_PROVIDER_KEY_VALUE_CFLAGS@
libmodule_pdpreprocessor_la_LDFLAGS  = -Wall -release $(PACKAGE_VERSION)

