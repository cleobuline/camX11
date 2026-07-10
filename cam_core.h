//
//  cam_core.h
//  cam-8
//

#ifndef CAM8_CORE_H
#define CAM8_CORE_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "cam_forth.h"

typedef enum {
    CAM_SIZE_128  = 128,
    CAM_SIZE_256  = 256,
    CAM_SIZE_512  = 512,
    CAM_SIZE_1024 = 1024
} CAMGridSize;

// Deux régimes d'évolution :
//  - CAM_MODE_LUT      : voisinage de Moore + LUT globale (règles CAM-FORTH classiques)
//  - CAM_MODE_MARGOLUS : partitionnement en blocs 2x2 (le vrai voisinage du CAM-8 original)
typedef enum {
    CAM_MODE_LUT      = 0,
    CAM_MODE_MARGOLUS = 1,
    CAM_MODE_HEX      = 2, // chapitre 16 : grille pseudo-hexagonale (FHP)
    CAM_MODE_CUSTOM   = 3  // chapitre 7 : voisinage libre N/CUSTOM (offsets déclarés)
} CAMStepMode;

typedef struct {
    uint8_t  *plane0_a;   // plan 0 courant  (CAM-A, vivant)
    uint8_t  *plane0_b;   // plan 0 suivant
    uint8_t  *plane1_a;   // plan 1 courant  (CAM-A, réfractaire/écho)
    uint8_t  *plane1_b;   // plan 1 suivant
    uint8_t  *plane2_a;   // plan 2 courant  (CAM-B)
    uint8_t  *plane2_b;   // plan 2 suivant
    uint8_t  *plane3_a;   // plan 3 courant  (CAM-B)
    uint8_t  *plane3_b;   // plan 3 suivant
    uint32_t  width;
    uint32_t  height;
    CAMGridSize size;

    // Parité de la partition Margolus : propre à CHAQUE grille (donc à chaque
    // clone utilisé pour l'export vidéo), pas globale — sinon un export
    // désynchroniserait la phase de la grille visible à l'écran.
    int margolus_phase;
} CAMState;

CAMState* cam_create(CAMGridSize size);
void      cam_destroy(CAMState *cam);
void      cam_swap_buffers(CAMState *cam);
void      cam_set_rule(const char *rule);
void      cam_step(CAMState *cam);

// LUT au format 2 bits par entrée : bit0 = nouveau plan 0,
// bit1 = nouveau plan 1 (l'ECHO y est déjà encodé par défaut).
void cam_apply_lut(uint8_t *lut);

// Applique en une fois les tables produites par forth_compile, selon le
// masque `built` (FORTH_BUILT_LUT / FORTH_BUILT_MARGOLUS), et bascule le
// mode d'évolution correspondant. Ne touche ni à la grille ni au play/pause.
void cam_apply_forth_tables(const ForthTables *tables, int built);

// --- Margolus ---
// Bascule le mode d'évolution global. cam_set_rule() repasse automatiquement
// en CAM_MODE_LUT (une règle Forth classique n'a de sens qu'en mode Moore).
void cam_set_step_mode(CAMStepMode mode);
CAMStepMode cam_get_step_mode(void);

// Table brute : 16 entrées, index = NW(bit0) NE(bit1) SW(bit2) SE(bit3),
// valeur = même encodage pour le bloc de sortie.
void cam_set_margolus_table(const uint8_t table[16]);

// --- Renversement du temps (chapitre 14) ---
// En mode Margolus, si la table est inversible (bijection par phase,
// indépendante du plan 1, sans expédition >PLN1), le moteur calcule
// automatiquement la table inverse au chargement de la règle.
// cam_step_back exécute alors un pas en ARRIÈRE : rebascule la phase,
// puis applique la table inverse sur la partition correspondante.
// Retourne 1 si le pas arrière a été fait, 0 si impossible (mode LUT,
// ou table non inversible).
int  cam_can_reverse(void);
int  cam_step_back(CAMState *cam);

// Préréglages emblématiques du bouquin Toffoli & Margolus.
// name: "CRITTERS" ou "BBM" (insensible à la casse).
// Retourne 1 si le nom est reconnu, 0 sinon.
int cam_set_margolus_preset(const char *name);

// Palette officielle 4 plans du CAM-8 labynet — UNIQUE source de vérité
// des couleurs, partagée par l'affichage (CAMView) et l'export vidéo.
// Priorité : plan 0 blanc > plan 1 cyan > CAM-B (plans 2-3) :
// valeur 3 jaune, 2 orange, 1 vert ; vide noir.
static inline void cam_palette(uint8_t p0, uint8_t p1, uint8_t p2, uint8_t p3,
                                uint8_t *r, uint8_t *g, uint8_t *b) {
    if (p0)            { *r = 255; *g = 255; *b = 255; }
    else if (p1)       { *r = 0;   *g = 200; *b = 255; }
    else if (p2 && p3) { *r = 220; *g = 210; *b = 0;   }
    else if (p3)       { *r = 255; *g = 140; *b = 0;   }
    else if (p2)       { *r = 0;   *g = 210; *b = 90;  }
    else               { *r = 0;   *g = 0;   *b = 0;   }
}

// accès inline aux cellules
static inline uint8_t cam_get_p0(const CAMState *cam, int x, int y) {
    return cam->plane0_a[y * cam->width + x];
}

static inline uint8_t cam_get_p1(const CAMState *cam, int x, int y) {
    return cam->plane1_a[y * cam->width + x];
}

static inline void cam_set_p0(CAMState *cam, int x, int y, uint8_t val) {
    cam->plane0_b[y * cam->width + x] = val;
}

static inline void cam_set_p1(CAMState *cam, int x, int y, uint8_t val) {
    cam->plane1_b[y * cam->width + x] = val;
}

#endif /* CAM8_CORE_H */
