# MacCam-6 — Makefile de la version X11
#
# Le moteur (cam_core, cam_forth, fhp) est du C pur, sans dependance.
# Seule l'interface a besoin de Xlib. Rien d'autre : ni GTK, ni SDL,
# ni pkg-config obligatoire pour le moteur lui-meme.
#
#   make            construit camx11
#   make selftest   verifie le moteur SANS ecran (machine distante, CI)
#   make run        lance avec le dossier regles/
#   make install    copie dans /usr/local/bin (PREFIX=... pour changer)
#   make clean
#
# Prerequis :
#   Debian / Ubuntu   sudo apt install libx11-dev pkg-config
#   Fedora / RHEL     sudo dnf install libX11-devel pkgconf
#   Arch              sudo pacman -S libx11 pkgconf
#   OpenBSD / *BSD    pkg_add ... (X11 est dans le systeme de base)
#   macOS + XQuartz   brew install pkg-config ; XQuartz fournit X11

CC      ?= cc
CFLAGS  ?= -O2 -Wall -Wextra -Wno-unused-parameter
PREFIX  ?= /usr/local

# --- Detection de X11 par pkg-config -------------------------------------
# pkg-config interroge le systeme et rend les bons chemins d'en-tetes et de
# bibliotheques, QUELLE QUE SOIT leur emplacement : /usr/lib sous Linux,
# /usr/X11R6 sous OpenBSD, /opt/X11 pour XQuartz sous macOS. C'est la seule
# facon reellement portable -- coder "-lX11" et "/opt/X11" en dur cassait
# la compilation sur OpenBSD (merci crc pour le signalement).
X11_CFLAGS := $(shell pkg-config --cflags x11 2>/dev/null)
X11_LIBS   := $(shell pkg-config --libs   x11 2>/dev/null)

# Repli si pkg-config est absent ou ne connait pas x11 : on tente le
# classique -lX11, qui marche sur la plupart des Linux sans pkg-config.
ifeq ($(strip $(X11_LIBS)),)
X11_LIBS := -lX11
endif

CFLAGS += $(X11_CFLAGS)
LDLIBS  = $(X11_LIBS) -lm

SRC = cam_core.c cam_forth.c fhp.c camx11.c
HDR = cam_core.h cam_forth.h fhp.h

all: camx11

# Compilation en un seul appel : quatre fichiers, la separation en .o ne
# ferait gagner que des fractions de seconde.
camx11: $(SRC) $(HDR)
	$(CC) $(CFLAGS) -o $@ $(SRC) $(LDLIBS)

# Verifie le moteur sans ouvrir de fenetre : utile sur un serveur, ou pour
# savoir si un echec vient du calcul ou de l'affichage.
selftest: camx11
	./camx11 --selftest

run: camx11
	./camx11 --size 256 --dir regles

install: camx11
	install -d $(DESTDIR)$(PREFIX)/bin
	install -m 755 camx11 $(DESTDIR)$(PREFIX)/bin/camx11

uninstall:
	rm -f $(DESTDIR)$(PREFIX)/bin/camx11

clean:
	rm -f camx11

.PHONY: all selftest run install uninstall clean
