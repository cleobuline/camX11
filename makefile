# MacCam-6 — Makefile de la version X11
#
# Le moteur (cam_core, cam_forth, fhp) est du C pur, sans dependance.
# Seule l'interface a besoin de Xlib. Rien d'autre : ni GTK, ni SDL,
# ni pkg-config.
#
#   make            construit camx11
#   make selftest   verifie le moteur SANS ecran (machine distante, CI)
#   make run        lance avec le dossier regles/
#   make install    copie dans /usr/local/bin (PREFIX=... pour changer)
#   make clean
#
# Prerequis :
#   Debian / Ubuntu   sudo apt install libx11-dev
#   Fedora / RHEL     sudo dnf install libX11-devel
#   Arch              sudo pacman -S libx11
#   macOS + XQuartz   les en-tetes sont dans /opt/X11 (voir plus bas)

CC      ?= cc
CFLAGS  ?= -O2 -Wall -Wextra -Wno-unused-parameter
LDLIBS   = -lX11 -lm
PREFIX  ?= /usr/local

# XQuartz n'installe pas ses en-tetes dans les chemins par defaut :
# on les ajoute seulement si le dossier existe, pour ne rien casser
# ailleurs.
ifneq ($(wildcard /opt/X11/include),)
CFLAGS  += -I/opt/X11/include
LDFLAGS += -L/opt/X11/lib
endif

SRC = cam_core.c cam_forth.c fhp.c camx11.c
HDR = cam_core.h cam_forth.h fhp.h

all: camx11

# Compilation en un seul appel : quatre fichiers, la separation en .o
# ne ferait gagner que des fractions de seconde et ajouterait des
# regles a lire.
camx11: $(SRC) $(HDR)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(SRC) $(LDLIBS)

# Verifie le moteur sans ouvrir de fenetre : utile sur un serveur, ou
# pour savoir si un echec vient du calcul ou de l'affichage.
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
