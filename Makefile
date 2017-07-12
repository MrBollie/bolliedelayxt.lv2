#!/usr/bin/make -f

include Makefile.mk

# --------------------------------------------------------------

PREFIX  ?= /usr/local
DESTDIR ?=
BUILDDIR ?= build/bolliedelayxt.lv2

# --------------------------------------------------------------
# Default target is to build all plugins

all: build
build: bolliedelayxt

# --------------------------------------------------------------
# bolliedelayxt build rules

bolliedelayxt: $(BUILDDIR) $(BUILDDIR)/bolliedelayxt$(LIB_EXT) $(BUILDDIR)/manifest.ttl $(BUILDDIR)/modgui.ttl $(BUILDDIR)/bolliedelayxt.ttl $(BUILDDIR)/modgui

$(BUILDDIR):
	mkdir -p $(BUILDDIR)

$(BUILDDIR)/bolliefilter.o: src/bolliefilter.c
	$(CC) $^ $(BUILD_C_FLAGS) $(LINK_FLAGS) -lm -o $@ -c

$(BUILDDIR)/bolliedelayxt.o: src/bollie-delay-xt.c
	$(CC) $^ $(BUILD_C_FLAGS) $(LINK_FLAGS) -lm -o $@ -c

$(BUILDDIR)/bolliedelayxt$(LIB_EXT): $(BUILDDIR)/bolliefilter.o $(BUILDDIR)/bolliedelayxt.o
	$(CC) $^ $(BUILD_C_FLAGS) $(LINK_FLAGS) -lm $(SHARED) -o $@

$(BUILDDIR)/manifest.ttl: lv2ttl/manifest.ttl.in
	sed -e "s|@LIB_EXT@|$(LIB_EXT)|" $< > $@

$(BUILDDIR)/modgui.ttl: lv2ttl/modgui.ttl.in
	sed -e "s|@LIB_EXT@|$(LIB_EXT)|" $< > $@

$(BUILDDIR)/bolliedelayxt.ttl: lv2ttl/bolliedelayxt.ttl
	cp $< $@

$(BUILDDIR)/modgui: modgui
	mkdir -p $@ 
	cp -rv $^/* $@/

# --------------------------------------------------------------

clean:
	rm -f $(BUILDDIR)/bolliedelayxt* $(BUILDDIR)/bolliefilter* $(BUILDDIR)/*.ttl
	rm -fr $(BUILDDIR)/modgui

# --------------------------------------------------------------

install: build
	echo "Install"
	install -d $(DESTDIR)$(PREFIX)/lib/lv2/bolliedelayxt.lv2
	install -d $(DESTDIR)$(PREFIX)/lib/lv2/bolliedelayxt.lv2/modgui

	install -m 644 $(BUILDDIR)/*.so  $(DESTDIR)$(PREFIX)/lib/lv2/bolliedelayxt.lv2/
	install -m 644 $(BUILDDIR)/*.ttl $(DESTDIR)$(PREFIX)/lib/lv2/bolliedelayxt.lv2/
	cp -rv $(BUILDDIR)/modgui/* $(DESTDIR)$(PREFIX)/lib/lv2/bolliedelayxt.lv2/modgui/

# --------------------------------------------------------------
uninstall:
	echo "Uninstall"
	rm -fr $(DESTDIR)$(PREFIX)/lib/lv2/bolliedelayxt.lv2
	
# --------------------------------------------------------------
