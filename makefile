# MacCam-6 — Makefile de la version X11
#
# Ecrit pour compiler avec N'IMPORTE QUEL make : GNU Make (meme l'ancien
# 3.81 de macOS), les make natifs des BSD, un make POSIX strict. Aucune
# syntaxe evoluee -- ni $(shell), ni !=, ni ifeq. La detection de X11 se
# fait entierement dans la recette, en shell, ce que tout make execute
# sans le comprendre lui-meme.
#
#   make            construit camx11
#   make selftest   verifie le moteur sans ecran
#   make run        lance avec le dossier regles/
#   make install    copie dans $(PREFIX)/bin
#   make clean
#
# Prerequis : compilateur C, en-tetes X11, pkg-config.
#   Debian/Ubuntu  apt install libx11-dev pkg-config
#   OpenBSD        X11 dans le systeme de base ; pkg_add pkgconf
#   macOS+XQuartz  brew install pkg-config

CC      = cc
CFLAGS  = -O2
PREFIX  = /usr/local

SRC = cam_core.c cam_forth.c fhp.c camx11.c

all: camx11

# Toute la detection X11 est faite ici, par le shell, a la compilation.
# On aide d'abord pkg-config a trouver XQuartz sur macOS (PKG_CONFIG_PATH),
# puis on lui demande les flags. S'il echoue, on essaie les chemins
# XQuartz en dur (macOS sans pkg-config), sinon -lX11 (Linux standard).
# Rien de tout ceci n'est de la syntaxe make : c'est du /bin/sh, compris
# par tous les make sans exception.
camx11: $(SRC) cam_core.h cam_forth.h fhp.h
	@PKG_CONFIG_PATH=/opt/X11/lib/pkgconfig:$$PKG_CONFIG_PATH; export PKG_CONFIG_PATH; \
	XFLAGS=`pkg-config --cflags --libs x11 2>/dev/null`; \
	if [ -z "$$XFLAGS" ]; then \
	  if [ -d /opt/X11/include ]; then \
	    XFLAGS="-I/opt/X11/include -L/opt/X11/lib -lX11"; \
	  else \
	    XFLAGS="-lX11"; \
	  fi; \
	fi; \
	echo "$(CC) $(CFLAGS) -o camx11 $(SRC) $$XFLAGS -lm"; \
	$(CC) $(CFLAGS) -o camx11 $(SRC) $$XFLAGS -lm

selftest: camx11
	./camx11 --selftest

run: camx11
	./camx11 --size 256 --dir regles

install: camx11
	mkdir -p $(DESTDIR)$(PREFIX)/bin
	cp camx11 $(DESTDIR)$(PREFIX)/bin/camx11
	chmod 755 $(DESTDIR)$(PREFIX)/bin/camx11

uninstall:
	rm -f $(DESTDIR)$(PREFIX)/bin/camx11

clean:
	rm -f camx11
