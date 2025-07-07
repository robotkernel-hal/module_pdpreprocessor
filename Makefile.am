ACLOCAL_AMFLAGS = -I m4
SUBDIRS=src

pkgconfig_DATA = module_pd_preprocessor.pc

install-exec-hook:
	mkdir -p $(DESTDIR)$(prefix)/doc
	cp -ru $(top_builddir)/doc/* $(DESTDIR)$(prefix)/doc

