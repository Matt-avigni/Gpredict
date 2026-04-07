ACLOCAL_AMFLAGS = -I m4
SUBDIRS = src doc icons pixmaps data po

BUILT_SOURCES = $(top_srcdir)/.version
$(top_srcdir)/.version:
	echo $(VERSION) > $@-t && mv $@-t $@
dist-hook:
	echo $(VERSION) > $(distdir)/.tarball-version

install-data-local:
	@$(NORMAL_INSTALL)
	$(INSTALL_DATA) $(top_srcdir)/AUTHORS $(DESTDIR)$(pkgdatadir)
	$(INSTALL_DATA) $(top_srcdir)/NEWS $(DESTDIR)$(pkgdatadir)
	$(INSTALL_DATA) $(top_srcdir)/README $(DESTDIR)$(pkgdatadir)
	$(INSTALL_DATA) $(top_srcdir)/COPYING $(DESTDIR)$(pkgdatadir)

EXTRA_DIST = \
	.gitattributes \
	.github/workflows/windows-installer.yml \
	autogen.sh            \
	git-version-gen       \
	intltool-extract.in   \
	intltool-merge.in     \
	intltool-update.in    \
	packaging/windows/README.md \
	packaging/windows/build-installer.sh \
	packaging/windows/gpredict-launcher.cmd.in \
	packaging/windows/gpredict-windows-installer.nsi \
	packaging/windows/hamlib-source.env \
	packaging/windows/patches/0001-hamlib-fix-spid-windows-build.patch \
	snap/snapcraft.yaml   \
	win32/README          \
	win32/config.mk       \
	win32/gpredict.rc        \
	win32/loaders.cache   \
	win32/resource.h      \
	win32/icons/gpredict-icon.ico
