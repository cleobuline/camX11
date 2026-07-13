//
//  camx11.c — MacCam-6 sous X11
//
//  Le moteur (cam_core.c, cam_forth.c, fhp.c) est du C pur et n'est
//  PAS modifié : ce fichier remplace exactement ce que Cocoa faisait,
//  ni plus ni moins — une fenêtre, un tampon de pixels, des touches.
//
//  Compilation :
//      cc -O2 -o camx11 cam_core.c cam_forth.c fhp.c camx11.c -lX11 -lm
//
//  Usage :
//      ./camx11                          grille 256, règle par défaut
//      ./camx11 --size 512
//      ./camx11 --rule regles/cavalier.forth
//      ./camx11 --selftest               tourne sans écran (pour le VPS)
//
//  Fonctionne à travers `ssh -X` : le tampon est envoyé en XImage, pas
//  d'accélération, pas de GLX. Sur un lien lent, baisse les FPS (touches
//  < et >) — c'est le débit d'images qui coûte, pas le calcul.
//
//  ---- CLAVIER ----------------------------------------------------
//   espace   play / pause          s   un seul pas
//   b        un pas EN ARRIÈRE (si la règle est réversible)
//   r        semer aléatoirement   c   tout effacer
//   0 1 2 3  plan de dessin        ! @ # $  afficher/cacher ce plan
//   p o q e y  pinceau, cercle, carré, gomme, spray
//   + -      taille du pinceau     < >  moins / plus d'images par seconde
//   f        mode FHP (gaz)        h   vue hydrodynamique
//   [ ]      rayon de lissage      w   vent continu   x   bord droit ouvert
//   i        spray isotrope
//   ESC / Q  quitter
//
//  La souris dessine. Comme sur le Mac : la simulation démarre EN PAUSE,
//  charger une règle ne lance rien, on reste maître de l'atelier.
//

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/keysym.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>
#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>   // usleep

#include "cam_core.h"
#include "cam_forth.h"
#include "fhp.h"

// ---- outils ---------------------------------------------------------
typedef enum { T_PENCIL, T_CIRCLE, T_SQUARE, T_ERASER, T_SPRAY } Tool;

static const char *RULE_DEFAUT =
    ": VIE\n"
    "  NORTH SOUTH + EAST + WEST + N.EAST + N.WEST + S.EAST + S.WEST +\n"
    "  CENTER IF DUP 2 = SWAP 3 = OR ELSE 3 = THEN ;\n"
    "MAKE-TABLE VIE\n";

// ---- état global de l'interface --------------------------------------
static CAMState *cam;
static FHPState *fhp;
static int   running     = 0;      // démarre EN PAUSE, toujours
static int   fhp_mode    = 0;
static int   hydro_view  = 0;
static int   hydro_r     = 4;
static int   iso_spray   = 0;
static int   wind        = 0;
static int   open_edge   = 0;
static Tool  tool        = T_PENCIL;
static int   brush       = 3;
static int   density     = 30;
static int   plane       = 0;
static int   vis_mask    = 0xF;
static int   fps         = 30;
static float *hydro_buf  = NULL;
// Affichage a l'ecran : l'aide s'ouvre AU DEMARRAGE (on ne devine pas
// des raccourcis) et se referme d'une touche. Le bandeau d'etat, lui,
// reste : c'est ce que la palette Cocoa montrait en permanence.
static int   show_help   = 1;
static XFontStruct *font = NULL;

// --- Bibliotheque de regles -------------------------------------------
// Xlib n'a pas de selecteur de fichier, et c'est tres bien : le dossier
// `regles/` EST le menu. On le lit au demarrage, on navigue avec n / N,
// et surtout on RECHARGE a chaud (touche l, ou automatiquement quand le
// fichier change sur le disque). Editer la regle dans son editeur et la
// voir recompiler sans quitter la fenetre bat n'importe quelle boite de
// dialogue -- c'est le cycle de travail du Forth, pas celui d'un GUI.
#define MAX_REGLES 128
static char  regles[MAX_REGLES][256];
static int   n_regles = 0, i_regle = -1;
// Chargement differe : appuyer sur n/N ne fait que NOTER la regle voulue.
// La compilation (couteuse pour les regles Margolus : ~200 ms) se fait au
// tour de boucle suivant, une seule fois, meme si on martele la touche.
// Sans ca, dix "n" rapides declenchaient dix compilations en rafale et
// l'interface semblait bloquee.
static int   regle_demandee = -1;
static char  regle_dir[192] = "regles";
static time_t regle_mtime = 0;
static char  msg[128] = "";          // message ephemere du bandeau
static int   msg_ttl = 0;            // en images

static uint32_t *pixels;           // ARGB, taille grille × grille
static uint32_t *screen;           // tampon affiché, mis à l'échelle
static int screen_side = 0;        // côté courant de `screen`
static int G;                      // côté de la grille

static uint32_t rgb(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

// ---- rendu ------------------------------------------------------------
static void render_cam(void) {
    uint32_t table[16];
    for (int n = 0; n < 16; n++) {
        uint8_t r, g, b;
        cam_palette(n & 1, (n >> 1) & 1, (n >> 2) & 1, (n >> 3) & 1, &r, &g, &b);
        table[n] = rgb(r, g, b);
    }
    for (int i = 0; i < G * G; i++) {
        int nib = ((vis_mask & 1) ? cam->plane0_a[i] : 0)
                | (((vis_mask & 2) ? cam->plane1_a[i] : 0) << 1)
                | (((vis_mask & 4) ? cam->plane2_a[i] : 0) << 2)
                | (((vis_mask & 8) ? cam->plane3_a[i] : 0) << 3);
        pixels[i] = table[nib];
    }
}

static void render_fhp(void) {
    if (hydro_view) {
        double rho0 = fhp_coarse_field(fhp, hydro_buf, hydro_r);
        float amp = 1e-6f;
        for (int i = 0; i < G * G; i++) {
            if (fhp->obstacle[i]) continue;
            float e = fabsf(hydro_buf[i] - (float)rho0);
            if (e > amp) amp = e;
        }
        for (int i = 0; i < G * G; i++) {
            if (fhp->obstacle[i]) { pixels[i] = rgb(255, 140, 0); continue; }
            float e = (hydro_buf[i] - (float)rho0) / amp;
            pixels[i] = (e > 0) ? rgb((uint8_t)(255 * e), (uint8_t)(60 * e), 0)
                                : rgb(0, (uint8_t)(140 * -e), (uint8_t)(255 * -e));
        }
        return;
    }
    for (int y = 0; y < G; y++)
        for (int x = 0; x < G; x++) {
            int i = y * G + x;
            if (fhp->obstacle[i]) { pixels[i] = rgb(255, 140, 0); continue; }
            int d = fhp_local_density(fhp, x, y);
            uint8_t l = (uint8_t)(d * 255 / 6);
            uint8_t b = d ? (uint8_t)(l * 0.6 + 60) : 0;
            pixels[i] = rgb(l, l, b);
        }
}

// ---- dessin -----------------------------------------------------------
static void paint(int gx, int gy) {
    if (gx < 0 || gy < 0 || gx >= G || gy >= G) return;

    if (fhp_mode) {
        int r = (tool == T_PENCIL || tool == T_ERASER) ? brush / 2 : brush;
        for (int dy = -r; dy <= r; dy++)
            for (int dx = -r; dx <= r; dx++) {
                if (tool == T_CIRCLE && dx * dx + dy * dy > r * r) continue;
                int nx = gx + dx, ny = gy + dy;
                if (nx < 0 || ny < 0 || nx >= G || ny >= G) continue;
                if (tool == T_SPRAY) {
                    if (dx * dx + dy * dy > r * r) continue;
                    // isotrope : les 6 canaux ; sinon vent dirigé sur HEX-E
                    int lo = iso_spray ? 0 : FHP_DIR_E;
                    int hi = iso_spray ? 6 : FHP_DIR_E + 1;
                    for (int d = lo; d < hi; d++)
                        if ((int)(random() % 100) < density)
                            fhp->dir_a[d][ny * G + nx] = 1;
                } else {
                    fhp_set_obstacle(fhp, nx, ny, tool == T_ERASER ? 0 : 1);
                }
            }
        return;
    }

    uint8_t *grids[4] = { cam->plane0_a, cam->plane1_a, cam->plane2_a, cam->plane3_a };
    uint8_t *g = grids[plane & 3];
    uint8_t  v = (tool == T_ERASER) ? 0 : 1;
    int r = (tool == T_PENCIL || tool == T_ERASER) ? brush / 2 : brush;

    for (int dy = -r; dy <= r; dy++)
        for (int dx = -r; dx <= r; dx++) {
            if ((tool == T_CIRCLE || tool == T_SPRAY) && dx * dx + dy * dy > r * r) continue;
            int nx = gx + dx, ny = gy + dy;
            if (nx < 0 || ny < 0 || nx >= G || ny >= G) continue;
            if (tool == T_SPRAY) {
                if ((int)(random() % 100) < density) g[ny * G + nx] = 1;
            } else g[ny * G + nx] = v;
        }
}

// ---- pas de simulation ------------------------------------------------
static void step_once(void) {
    if (fhp_mode) {
        fhp->open_right_edge = open_edge;
        if (wind)   // réinjecte du gaz sur le bord gauche, canal EST
            for (int y = 0; y < G; y++)
                if ((int)(random() % 100) < density) fhp->dir_a[FHP_DIR_E][y * G] = 1;
        fhp_step(fhp);
    } else cam_step(cam);
}

static void clear_all(void) {
    if (fhp_mode) { fhp_clear(fhp); return; }
    size_t n = (size_t)G * G;
    memset(cam->plane0_a, 0, n); memset(cam->plane0_b, 0, n);
    memset(cam->plane1_a, 0, n); memset(cam->plane1_b, 0, n);
    memset(cam->plane2_a, 0, n); memset(cam->plane2_b, 0, n);
    memset(cam->plane3_a, 0, n); memset(cam->plane3_b, 0, n);
}

static void randomize(void) {
    if (fhp_mode) {
        for (int d = 0; d < 6; d++) fhp_seed_random(fhp, (FHPDirection)d, density);
        return;
    }
    for (int i = 0; i < G * G; i++)
        cam->plane0_a[i] = ((int)(random() % 100) < density) ? 1 : 0;
}


static void titre(Display *dpy, Window w) {
    static const char *tn[] = {"pinceau","cercle","carre","gomme","spray"};
    char hyd[24] = "";
    if (fhp_mode && hydro_view) snprintf(hyd, sizeof hyd, " hydro r=%d", hydro_r);
    char buf[256];
    snprintf(buf, sizeof buf,
        "MacCam-6 [%s] %s%s | plan %d | %s %d | %d fps",
        running ? "play" : "pause",
        fhp_mode ? "FHP" : "CAM", hyd,
        plane, tn[tool], brush, fps);
    XStoreName(dpy, w, buf);
}


// Bandeau d'etat + aide, dessines PAR-DESSUS la grille. Xlib n'a pas de
// widgets : on ecrit du texte sur des rectangles noirs, ce qui suffit
// amplement et n'ajoute aucune dependance.
static void ligne(Display *dpy, Drawable d, GC gc, int x, int y, const char *t) {
    XDrawString(dpy, d, gc, x, y, t, (int)strlen(t));
}

static void dessine_hud(Display *dpy, Drawable w, GC gc, int scr) {
    static const char *tn[] = {"pinceau","cercle","carre","gomme","spray"};
    char buf[192], hyd[32] = "";
    if (fhp_mode && hydro_view) snprintf(hyd, sizeof hyd, "  hydro r=%d", hydro_r);

    if (fhp_mode)
        snprintf(buf, sizeof buf, "%s  FHP%s  %s %d  densite %d%%  %d fps%s%s",
                 running ? "PLAY " : "PAUSE", hyd, tn[tool], brush, density, fps,
                 wind ? "  vent" : "", iso_spray ? "  iso" : "");
    else
        snprintf(buf, sizeof buf, "%s  CAM  plan %d  visible %c%c%c%c  %s %d  %d fps",
                 running ? "PLAY " : "PAUSE", plane,
                 (vis_mask&1)?'0':'.', (vis_mask&2)?'1':'.',
                 (vis_mask&4)?'2':'.', (vis_mask&8)?'3':'.',
                 tn[tool], brush, fps);

    XSetForeground(dpy, gc, BlackPixel(dpy, scr));
    XFillRectangle(dpy, w, gc, 0, 0, 8 + 7 * (int)strlen(buf), 20);
    XSetForeground(dpy, gc, WhitePixel(dpy, scr));
    ligne(dpy, w, gc, 6, 14, buf);

    // Deuxieme ligne : la regle chargee, ou le dernier message.
    const char *l2 = NULL;
    char rbuf[200];
    if (msg_ttl > 0) l2 = msg;
    else if (i_regle >= 0) {
        const char *b = strrchr(regles[i_regle], '/');
        snprintf(rbuf, sizeof rbuf, "%.60s   [%d/%d]  n N  l",
                 b ? b + 1 : regles[i_regle], i_regle + 1, n_regles);
        l2 = rbuf;
    }
    if (l2 && (!show_help || msg_ttl > 0)) {
        int yy = show_help ? 20 : 20;
        XSetForeground(dpy, gc, BlackPixel(dpy, scr));
        XFillRectangle(dpy, w, gc, 0, yy, 8 + 7 * (int)strlen(l2), 18);
        XSetForeground(dpy, gc, WhitePixel(dpy, scr));
        ligne(dpy, w, gc, 6, yy + 13, l2);
    }

    if (!show_help) return;
    int y0 = 40;   // sous les deux lignes du bandeau

    static const char *h[] = {
        " espace  play / pause          s  un pas",
        " b  un pas en arriere          r  semer aleatoirement",
        " c  effacer tout               ?  fermer cette aide",
        "",
        " p pinceau   o cercle   a carre   e gomme   y spray",
        " + -  taille du pinceau        < >  images par seconde",
        " 0 1 2 3  plan de dessin       ! @ # $  afficher un plan",
        "",
        " f  gaz FHP                    h  vue hydrodynamique",
        " [ ]  rayon de lissage         i  spray isotrope",
        " w  vent continu               x  bord droit ouvert",
        " v V  viscosite                d D  densite du semis",
        "",
        " n N  regle suivante / precedente (dossier regles/)",
        " l    recharger  (ou sauvez le fichier : recompilation auto)",
        "",
        " ESC ou Q  quitter",
        NULL
    };
    int nl = 0; while (h[nl]) nl++;
    int hh = 14 * nl + 16;
    XSetForeground(dpy, gc, BlackPixel(dpy, scr));
    XFillRectangle(dpy, w, gc, 0, y0, 400, hh);
    XSetForeground(dpy, gc, WhitePixel(dpy, scr));
    XDrawRectangle(dpy, w, gc, 0, y0, 399, hh - 1);
    for (int i = 0; i < nl; i++) ligne(dpy, w, gc, 8, y0 + 16 + 14 * i, h[i]);
}

static int cmp_str(const void *a, const void *b) {
    return strcmp((const char *)a, (const char *)b);
}

// Extensions acceptees. `.rule` d'abord : c'est celle qu'utilise le
// projet Xcode. Filtrer sur la seule `.forth` faisait passer le dossier
// pour vide alors qu'il etait plein -- une extension n'est pas un
// contrat, et le programme n'a pas a imposer la sienne.
static int est_une_regle(const char *nom) {
    static const char *ext[] = { ".rule", ".forth", ".fth", ".f", NULL };
    const char *dot = strrchr(nom, '.');
    if (!dot) return 0;
    for (int i = 0; ext[i]; i++) if (!strcmp(dot, ext[i])) return 1;
    return 0;
}

static void scan_regles(void) {
    n_regles = 0;
    DIR *d = opendir(regle_dir);
    if (!d) { perror(regle_dir); return; }   // dis POURQUOI, pas juste "aucune"
    struct dirent *e;
    while ((e = readdir(d)) && n_regles < MAX_REGLES) {
        if (e->d_name[0] == '.') continue;    // pas de fichiers caches
        if (!est_une_regle(e->d_name)) continue;
        snprintf(regles[n_regles], sizeof regles[0], "%.180s/%.60s", regle_dir, e->d_name);
        n_regles++;
    }
    closedir(d);
    qsort(regles, n_regles, sizeof regles[0], cmp_str);
}

static void flash(const char *fmt, ...) {
    va_list ap; va_start(ap, fmt);
    vsnprintf(msg, sizeof msg, fmt, ap);
    va_end(ap);
    msg_ttl = 90;                     // ~3 secondes a 30 images/s
    printf("%s\n", msg); fflush(stdout);
}

// Charge et compile. Ne touche NI a la grille NI au play/pause : charger
// une regle ne lance rien et n'efface rien, comme sur le Mac.
static void charge_regle(int idx) {
    if (idx < 0 || idx >= n_regles) return;
    FILE *f = fopen(regles[idx], "rb");
    if (!f) { flash("illisible : %s", regles[idx]); return; }
    fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
    char *src = malloc(n + 1);
    if (fread(src, 1, n, f) != (size_t)n) { free(src); fclose(f); return; }
    src[n] = 0; fclose(f);

    cam_set_rule(src);
    free(src);
    i_regle = idx;

    struct stat st;
    regle_mtime = (stat(regles[idx], &st) == 0) ? st.st_mtime : 0;

    const char *base = strrchr(regles[idx], '/');
    flash("regle : %s%s", base ? base + 1 : regles[idx],
          running ? "" : "  (toujours en pause)");
}

// Recompile si le fichier a change sur le disque. Appele une fois par
// image : un stat() par image est indolore, et permet de garder son
// editeur ouvert a cote de la fenetre.
static void surveille_regle(void) {
    if (i_regle < 0) return;
    struct stat st;
    if (stat(regles[i_regle], &st) != 0) return;
    if (st.st_mtime == regle_mtime) return;
    charge_regle(i_regle);
    flash("recompilee (fichier modifie)");
}

static void aide(void) {
    printf(
    "\n  espace  play/pause      s  un pas        b  un pas en arriere\n"
    "  r  semer               c  effacer\n"
    "  0 1 2 3  plan de dessin        ! @ # $  afficher/cacher un plan\n"
    "  p pinceau  o cercle  a carre  e gomme  y spray\n"
    "  + -  taille du pinceau         < >  images par seconde\n"
    "  f  mode FHP (gaz)      h  vue hydrodynamique   [ ]  lissage\n"
    "  w  vent continu        x  bord droit ouvert    i  spray isotrope\n"
    "  ?  cette aide          ESC ou Q  quitter\n\n");
    fflush(stdout);
}

// ---- selftest : tourne sans écran, pour le VPS ------------------------
static int selftest(void) {
    printf("selftest : moteur CAM + FHP, sans écran\n");
    cam = cam_create(CAM_SIZE_128); G = 128;
    cam_set_rule(RULE_DEFAUT);
    for (int i = 0; i < 128 * 128; i++) cam->plane0_a[i] = (random() % 100 < 30);
    int p0 = 0; for (int i = 0; i < 128 * 128; i++) p0 += cam->plane0_a[i];
    for (int s = 0; s < 100; s++) cam_step(cam);
    int p1 = 0; for (int i = 0; i < 128 * 128; i++) p1 += cam->plane0_a[i];
    printf("  Life 128x128, 100 pas : %d -> %d cellules  %s\n", p0, p1,
           (p1 > 200 && p1 < p0) ? "ok" : "SUSPECT");

    fhp = fhp_create(128, 128);
    for (int d = 0; d < 6; d++) fhp_seed_random(fhp, (FHPDirection)d, 30);
    int n0 = fhp_total_population(fhp);
    for (int s = 0; s < 100; s++) fhp_step(fhp);
    int n1 = fhp_total_population(fhp);
    printf("  FHP 128x128, 100 pas : %d -> %d particules  %s\n", n0, n1,
           n0 == n1 ? "ok (conservation exacte)" : "ECHEC");

    float *b = malloc(sizeof(float) * 128 * 128);
    double m = fhp_coarse_field(fhp, b, 4);
    printf("  coarse_field r=4 : densité moyenne %.4f  ok\n", m);
    free(b);
    cam_destroy(cam); fhp_destroy(fhp);
    return (n0 == n1) ? 0 : 1;
}

// ---- principal --------------------------------------------------------
int main(int argc, char **argv) {
    G = 256;
    const char *rule_path = NULL;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--selftest")) return selftest();
        if (!strcmp(argv[i], "--size") && i + 1 < argc) G = atoi(argv[++i]);
        if (!strcmp(argv[i], "--rule") && i + 1 < argc) rule_path = argv[++i];
        if (!strcmp(argv[i], "--dir")  && i + 1 < argc)
            snprintf(regle_dir, sizeof regle_dir, "%s", argv[++i]);
    }
    if (G != 128 && G != 256 && G != 512 && G != 1024) {
        fprintf(stderr, "taille : 128, 256, 512 ou 1024\n"); return 1;
    }

    cam = cam_create((CAMGridSize)G);
    fhp = fhp_create(G, G);
    pixels    = malloc(sizeof(uint32_t) * G * G);
    hydro_buf = malloc(sizeof(float) * G * G);

    scan_regles();
    cam_set_rule(RULE_DEFAUT);
    if (rule_path) {
        // --rule pointe un fichier precis : on le place dans la liste
        // s'il n'y est pas deja, pour que n / N restent utilisables.
        int found = -1;
        for (int i = 0; i < n_regles; i++)
            if (!strcmp(regles[i], rule_path)) found = i;
        if (found < 0 && n_regles < MAX_REGLES) {
            snprintf(regles[n_regles], sizeof regles[0], "%s", rule_path);
            found = n_regles++;
        }
        charge_regle(found);
    }
    if (n_regles == 0)
        printf("AUCUNE regle dans « %s/ » — extensions reconnues : "
               ".rule .forth .fth .f\n"
               "(lance depuis le bon dossier, ou utilise --dir chemin/vers/regles)\n",
               regle_dir);
    else {
        printf("%d regle(s) dans « %s/ » — n / N pour naviguer, l pour recharger :\n",
               n_regles, regle_dir);
        for (int i = 0; i < n_regles; i++) printf("   [%d] %s\n", i + 1, regles[i]);
    }

    Display *dpy = XOpenDisplay(NULL);
    if (!dpy) { fprintf(stderr, "pas de serveur X (essayez --selftest)\n"); return 1; }
    int scr = DefaultScreen(dpy);
    int win_side = (G > 512) ? 768 : ((G < 256) ? 512 : G * 2);

    Window w = XCreateSimpleWindow(dpy, RootWindow(dpy, scr), 0, 0,
                                   win_side, win_side, 0,
                                   BlackPixel(dpy, scr), BlackPixel(dpy, scr));
    XSelectInput(dpy, w, ExposureMask | KeyPressMask | ButtonPressMask |
                         Button1MotionMask | StructureNotifyMask);
    XMapWindow(dpy, w);
    GC gc = XCreateGC(dpy, w, 0, NULL);

    // Police serveur classique. Si le serveur X distant ne l'a pas, on
    // continue avec la police par defaut du GC plutot que de refuser de
    // demarrer : le bandeau est un confort, pas une condition.
    font = XLoadQueryFont(dpy, "fixed");
    if (font) XSetFont(dpy, gc, font->fid);

    printf("MacCam-6 / X11 — grille %d. La simulation demarre EN PAUSE.\n", G);
    aide();

    // L'XImage est (re)créée à chaque changement de taille de fenêtre,
    // car elle enveloppe directement le tampon `screen`. On prend la
    // profondeur réelle du serveur plutôt que 24 en dur : sur un
    // affichage distant en 16 bits, XCreateImage refuserait.
    int depth = DefaultDepth(dpy, scr);
    XImage *img = NULL;

    // Double tampon. Sans lui, l'image est envoyee a la fenetre PUIS le
    // bandeau par-dessus : X affiche l'etat intermediaire et le texte
    // clignote a la cadence de la simulation. On compose tout hors ecran
    // dans un Pixmap, on copie d'un seul coup. C'est le b.a.-ba de Xlib,
    // et je l'avais oublie.
    Pixmap back = None;
    int back_w = 0, back_h = 0;

    int win_w = win_side, win_h = win_side;
    struct timeval last; gettimeofday(&last, NULL);

    for (;;) {
        while (XPending(dpy)) {
            XEvent e; XNextEvent(dpy, &e);
            if (e.type == ConfigureNotify) { win_w = e.xconfigure.width; win_h = e.xconfigure.height; }
            else if (e.type == KeyPress) {
                // XLookupString respecte la DISPOSITION du clavier : sur un
                // AZERTY, XLookupKeysym(...,0) rend `ampersand` la ou l'on a
                // tape 1, et toutes les touches chiffrees deviennent muettes.
                // On lit donc le caractere effectivement produit, et on garde
                // le keysym uniquement pour les touches sans caractere (Echap).
                char c[8] = {0};
                KeySym ks;
                int n = XLookupString(&e.xkey, c, sizeof c - 1, &ks, NULL);
                char ch = (n > 0) ? c[0] : 0;
                const char *act = NULL;

                if (ks == XK_Escape) goto fin;
                switch (ch) {
                    case 'q': case 'Q': goto fin;
                    case ' ': running = !running; act = running ? "play" : "pause"; break;
                    case 's': step_once(); act = "un pas"; break;
                    case 'b':
                        if (cam_can_reverse()) { cam_step_back(cam); act = "un pas en arriere"; }
                        else act = "regle non reversible";
                        break;
                    case 'r': randomize(); act = "graine aleatoire"; break;
                    case 'c': clear_all();  act = "efface"; break;
                    case 'p': tool = T_PENCIL; act = "pinceau"; break;
                    case 'o': tool = T_CIRCLE; act = "cercle"; break;
                    case 'a': tool = T_SQUARE; act = "carre"; break;
                    case 'e': tool = T_ERASER; act = "gomme"; break;
                    case 'y': tool = T_SPRAY;  act = "spray"; break;
                    case 'f': fhp_mode = !fhp_mode;   act = fhp_mode ? "mode FHP" : "mode CAM"; break;
                    case 'h': hydro_view = !hydro_view; act = hydro_view ? "vue hydro" : "vue brute"; break;
                    case 'i': iso_spray = !iso_spray; act = iso_spray ? "spray isotrope" : "spray dirige"; break;
                    case 'w': wind = !wind;           act = wind ? "vent" : "sans vent"; break;
                    case 'x': open_edge = !open_edge; act = open_edge ? "bord ouvert" : "tore"; break;
                    case '[': if (hydro_r > 0)  hydro_r--; act = "lissage -"; break;
                    case ']': if (hydro_r < 12) hydro_r++; act = "lissage +"; break;
                    case '+': case '=': if (brush < 40) brush++; act = "pinceau +"; break;
                    case '-': if (brush > 1) brush--;           act = "pinceau -"; break;
                    case '<': fps -= 5; if (fps < 1) fps = 1;   act = "moins vite"; break;
                    case '>': if (fps < 120) fps += 5;          act = "plus vite"; break;
                    case '0': case '1': case '2': case '3':
                        plane = ch - '0'; act = "plan de dessin"; break;
                    case '!': vis_mask ^= 1; act = "visibilite plan 0"; break;
                    case '@': vis_mask ^= 2; act = "visibilite plan 1"; break;
                    case '#': vis_mask ^= 4; act = "visibilite plan 2"; break;
                    case '$': vis_mask ^= 8; act = "visibilite plan 3"; break;
                    case '?': show_help = !show_help; act = "aide"; break;
                    case 'n':
                        if (n_regles) {
                            int cur = (regle_demandee >= 0) ? regle_demandee : i_regle;
                            regle_demandee = (cur + 1) % n_regles;
                            flash("-> %s", strrchr(regles[regle_demandee], '/')
                                           ? strrchr(regles[regle_demandee], '/') + 1
                                           : regles[regle_demandee]);
                            act = "regle suivante";
                        } else { flash("aucune regle dans %s/", regle_dir); act = "aucune regle"; }
                        break;
                    case 'N':
                        if (n_regles) {
                            int cur = (regle_demandee >= 0) ? regle_demandee : i_regle;
                            regle_demandee = (cur - 1 + n_regles) % n_regles;
                            flash("-> %s", strrchr(regles[regle_demandee], '/')
                                           ? strrchr(regles[regle_demandee], '/') + 1
                                           : regles[regle_demandee]);
                            act = "regle precedente";
                        } else { flash("aucune regle dans %s/", regle_dir); act = "aucune regle"; }
                        break;
                    case 'l':
                        if (i_regle >= 0) { charge_regle(i_regle); flash("rechargee"); act = "recharge"; }
                        else act = "aucune regle chargee";
                        break;
                    case 'd': density -= 5; if (density < 0)   density = 0;   flash("densite %d%%", density); break;
                    case 'D': density += 5; if (density > 100) density = 100; flash("densite %d%%", density); break;
                    case 'v': if (fhp->p_rest_convert > 0)  fhp->p_rest_convert -= 5;
                              flash("viscosite %d%%", fhp->p_rest_convert); break;
                    case 'V': if (fhp->p_rest_convert < 100) fhp->p_rest_convert += 5;
                              flash("viscosite %d%%", fhp->p_rest_convert); break;
                    default: break;
                }
                // Retour dans le terminal : sans lui, impossible de savoir si
                // une touche a seulement ete recue -- c'est ce qui manquait.
                if (act && ch != '?') show_help = 0;   // la premiere action referme l'aide
                if (act) printf("[%c] %s\n", ch ? ch : '?', act), fflush(stdout);
                titre(dpy, w);
            }
            else if (e.type == ButtonPress || e.type == MotionNotify) {
                int px = (e.type == ButtonPress) ? e.xbutton.x : e.xmotion.x;
                int py = (e.type == ButtonPress) ? e.xbutton.y : e.xmotion.y;
                int m = (win_w < win_h) ? win_w : win_h;
                int ox = (win_w - m) / 2, oy = (win_h - m) / 2;
                paint((px - ox) * G / m, (py - oy) * G / m);
            }
        }

        // Chargement differe : on compile la derniere regle demandee, une
        // seule fois par tour de boucle. Marteler n/N ne compile donc que
        // la regle finale, pas toutes les intermediaires.
        if (regle_demandee >= 0 && regle_demandee != i_regle) {
            charge_regle(regle_demandee);
            regle_demandee = -1;
        } else {
            regle_demandee = -1;
        }
        surveille_regle();          // recompile si le .forth a change
        if (msg_ttl > 0) msg_ttl--;
        if (running) step_once();

        if (fhp_mode) render_fhp(); else render_cam();

        // Mise à l'échelle par plus proche voisin, faite ici plutôt que
        // par X : XCopyArea ne sait PAS redimensionner, et l'interpolation
        // n'aurait de toute façon rien à faire sur un automate cellulaire
        // — un pixel de la grille doit rester un carré net, comme sur le
        // CAM-8 d'origine. Le carré est centré dans la fenêtre.
        int m = (win_w < win_h) ? win_w : win_h;
        if (m < 1) m = 1;
        if (m != screen_side) {
            free(screen);
            screen = malloc(sizeof(uint32_t) * (size_t)m * m);
            screen_side = m;
            if (img) { img->data = NULL; XDestroyImage(img); }
            img = XCreateImage(dpy, DefaultVisual(dpy, scr), depth, ZPixmap, 0,
                               (char *)screen, m, m, 32, 0);
        }
        for (int y = 0; y < m; y++) {
            const uint32_t *row = pixels + (size_t)(y * G / m) * G;
            uint32_t *dst = screen + (size_t)y * m;
            for (int x = 0; x < m; x++) dst[x] = row[x * G / m];
        }

        if (back == None || back_w != win_w || back_h != win_h) {
            if (back != None) XFreePixmap(dpy, back);
            back = XCreatePixmap(dpy, w, win_w, win_h, depth);
            back_w = win_w; back_h = win_h;
        }
        // fond noir, image centree, bandeau par-dessus — le tout hors ecran
        XSetForeground(dpy, gc, BlackPixel(dpy, scr));
        XFillRectangle(dpy, back, gc, 0, 0, win_w, win_h);
        XPutImage(dpy, back, gc, img, 0, 0, (win_w - m) / 2, (win_h - m) / 2, m, m);
        dessine_hud(dpy, back, gc, scr);
        XCopyArea(dpy, back, w, gc, 0, 0, win_w, win_h, 0, 0);
        XFlush(dpy);

        // cadence
        struct timeval now; gettimeofday(&now, NULL);
        long us = (now.tv_sec - last.tv_sec) * 1000000 + (now.tv_usec - last.tv_usec);
        long cible = 1000000 / fps;
        if (us < cible) usleep(cible - us);
        last = now;
    }
fin:
    if (img) { img->data = NULL; XDestroyImage(img); }
    if (back != None) XFreePixmap(dpy, back);
    free(screen);
    free(pixels);
    if (font) XFreeFont(dpy, font);
    XFreeGC(dpy, gc);
    XCloseDisplay(dpy);
    free(hydro_buf);
    cam_destroy(cam);
    fhp_destroy(fhp);
    return 0;
}
