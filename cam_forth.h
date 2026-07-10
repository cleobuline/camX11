#ifndef CAM8_FORTH_H
#define CAM8_FORTH_H

#include <stdint.h>

#define FORTH_STACK_SIZE    64
// LUT : 10 bits de voisinage + RAND (bit 10, chapitre 8)
//     + sondes &CENTER (bit 11) et &CENTER' (bit 12) vers l'autre
//       demi-machine (chapitre 9) = 8192 entrées
#define FORTH_LUT_SIZE      8192
#define FORTH_MARGOLUS_SIZE 16    // bloc 2x2 d'un seul plan (presets)
// Table Margolus complète (tableau 7.2 + chapitres 8 et 11) :
// index = nibble plan 0 (bits 0-3) | nibble plan 1 (bits 4-7)
//       | PHASE (bit 8) | RAND (bit 9) | PHASE' (bit 10)
// 32768 = 2^15 : p0(4) + p1(4) + phase(1) + rand(1) + phase'(1) +
// sonde-croisee-CENTER' (4, une par coin — voir &/CENTERS en Margolus).
// Avant l'extension du 6/7, cette table faisait 2048 (2^11) : aucune
// place pour les sondes croisees, qui n'existaient qu'en mode LUT.
#define FORTH_MARG_TABLE_SIZE 32768
// Longueur maximale d'un run-cycle (§11.5)
#define FORTH_CYCLE_MAX 16
#define FORTH_MAX_TOKENS    256
#define FORTH_DICT_SIZE     32
#define FORTH_WORD_MAXLEN   32
#define FORTH_BODY_MAXLEN   512

// Position d'une cellule dans un bloc 2x2 de Margolus. L'encodage du bloc
// (bit0=NW bit1=NE bit2=SW bit3=SE) est celui déjà utilisé par cam_core.c.
typedef enum {
    FORTH_CORNER_NW = 0,
    FORTH_CORNER_NE = 1,
    FORTH_CORNER_SW = 2,
    FORTH_CORNER_SE = 3
} ForthCorner;

// forth_compile() retourne un OU de ces bits selon les tables construites.
#define FORTH_BUILT_LUT      0x1
#define FORTH_BUILT_MARGOLUS 0x2
#define FORTH_BUILT_LUT_B    0x4
#define FORTH_BUILT_MARGOLUS_B 0x8

typedef struct {
    char name[FORTH_WORD_MAXLEN];
    char body[FORTH_BODY_MAXLEN];
    // Cache des tokens de "body", calcule UNE FOIS a la definition (:)
    // plutot qu'a chaque appel du mot. Sans ce cache, un mot invoque
    // a l'interieur d'une regle Margolus/CAM-B est re-tokenise a
    // chaque coin de chaque entree de table -- des milliers de fois
    // pour le meme texte immuable.
    char tokens[FORTH_MAX_TOKENS][32];
    int  token_count;
} ForthWord;

// Voisinage déclaré dans la source Forth (N/MOORE, N/VONN, N/MARG) —
// détermine ce que MAKE-TABLE construit, comme dans le vrai CAM Forth (§7.3).
typedef enum {
    FORTH_NEIGHBORHOOD_MOORE    = 0,
    FORTH_NEIGHBORHOOD_VONN     = 1,
    FORTH_NEIGHBORHOOD_MARGOLUS = 2,
    FORTH_NEIGHBORHOOD_HEX      = 3,
    FORTH_NEIGHBORHOOD_CUSTOM   = 4  // N/CUSTOM ... END-CUSTOM (voisinage libre)
} ForthNeighborhood;

// N/HEX (chapitre 16, FHP) : grille pseudo-hexagonale par decalage en
// quinconce, une ligne sur deux. 6 directions a 60 degres au lieu des
// 4/8 habituelles. Encodage horaire a partir de l'est : E, NE, NW, W,
// SW, SE. bit0=CENTER bit1..6=E,NE,NW,W,SW,SE bit7=RAND
#define FORTH_HEX_LUT_SIZE 256

// N/CUSTOM (chapitre 7 du livre, voisinages arbitraires) : jusqu'a 12
// voisins declares librement par des triplets (dx, dy, plane). 12 est
// la limite du vrai CAM-8 -- et pas un hasard ici non plus : 12 voisins
// + le pseudo-voisin RAND = 13 bits = 8192 entrees, EXACTEMENT
// FORTH_LUT_SIZE. La LUT custom reutilise donc le buffer lut existant
// sans en changer la taille.
// Convention BOUSSOLE (celle du livre, pas celle de la memoire) :
//   dx > 0 = est, dy > 0 = NORD (vers le haut de l'ecran).
// plane : 0 ou 1 -- les plans PROPRES de la demi-machine courante,
// comme partout ailleurs (CAM-A : plans 0-1).
#define FORTH_CUSTOM_MAX 12

// Un voisin declare : deplacement (dx, dy) et plan source. int8_t
// suffit largement (le vrai CAM-8 limitait les kicks bien avant ±127).
typedef struct {
    int8_t  dx;
    int8_t  dy;
    uint8_t plane; // 0 ou 1
} ForthCustomNbr;

typedef struct {
    int32_t   stack[FORTH_STACK_SIZE];
    int       sp;

    uint8_t center, center_prime;
    uint8_t north, south, east, west;
    uint8_t neast, nwest, seast, swest;

    // Voisins hexagonaux (N/HEX, chapitre 16) : 6 directions a 60 degres.
    uint8_t hex_e, hex_ne, hex_nw, hex_w, hex_sw, hex_se;

    // N/VONN (tableau 7.2) : voisins von Neumann du plan 1.
    uint8_t north_p, south_p, east_p, west_p;

    // Voisinage de Margolus (§12.5, tableau 7.2 complet) :
    // CENTER/CW/CCW/OPP lisent le bloc du plan 0, CENTER'/CW'/CCW'/OPP'
    // (champs *_p) le bloc du plan 1. "Position-smart" — leur sens dépend
    // du coin (NW/NE/SW/SE) qu'occupe la cellule dans son bloc 2x2.
    uint8_t cw, ccw, opp;
    uint8_t cw_p, ccw_p, opp_p;

    // Pseudo-voisins de N/MARG-PH et N/MARG-HV (§12.7, §12.8) :
    // phase/phase' = les deux bits de phase temporelle pilotés par le
    // run-cycle (§11.5), horz/vert = parités spatiales de la cellule.
    uint8_t phase, phase_prime, horz, vert;

    // Sondes vers l'autre demi-machine (chapitre 9) : &CENTER et
    // &CENTER' lisent les deux plans du centre de l'autre moitié
    // (CAM-B vus depuis CAM-A, et réciproquement).
    uint8_t amp_center, amp_center_prime;

    // Demi-machine courante (sections CAM-A / CAM-B de la source).
    int half; // 0 = CAM-A (défaut), 1 = CAM-B

    // Pseudo-voisin bruyant RAND (chapitre 8) : bit aléatoire tiré par
    // le moteur à chaque évaluation (par cellule en LUT, par bloc en
    // Margolus). Une règle qui le lit devient probabiliste — et donc
    // non réversible, ce que le calcul d'inverse détecte.
    uint8_t rnd;

    // Voisinage courant, sélectionné par les déclarations N/MOORE,
    // N/VONN et N/MARG dans la source. Défaut : Moore.
    ForthNeighborhood neighborhood;

    // N/CUSTOM : la declaration courante (liste de triplets) et les
    // valeurs decodees pour l'entree en cours. Les mots NBR0..NBR11
    // lisent custom[] dans l'ORDRE de declaration. Si un triplet
    // (0,0,0) ou (0,0,1) est declare, CENTER / CENTER' sont cables en
    // plus, par confort -- une regle existante qui lit CENTER reste
    // lisible telle quelle.
    ForthCustomNbr custom_def[FORTH_CUSTOM_MAX];
    int            custom_count;
    uint8_t        custom[FORTH_CUSTOM_MAX];

    // Expédition vers les plans (>PLN0 .. >PLN3, §5.4 du livre) :
    // out_val[n] reçoit la valeur dépilée par >PLNn, out_mask trace quels
    // plans ont été explicitement adressés pendant une évaluation.
    // Les plans 2 et 3 (CAM-B) sont acceptés mais non câblés au moteur.
    uint8_t out_val[4];
    uint8_t out_mask;

    ForthWord dict[FORTH_DICT_SIZE];
    int       dict_size;
} ForthVM;

// Tables produites par la compilation d'une source CAM-FORTH.
//  - lut : 2 bits par entrée — bit0 = nouveau plan 0, bit1 = nouveau plan 1.
//    Si la règle n'expédie pas >PLN1, le bit1 reçoit l'ECHO (= CENTER).
//    lut_neighborhood précise l'encodage des 10 bits d'entrée :
//    MOORE (diagonales plan 0) ou VONN (voisins primés plan 1).
//  - margolus_p0 / margolus_p1 : tables de bloc complètes
//    (FORTH_MARG_TABLE_SIZE entrées : plan 0 + plan 1 + PHASE).
//    margolus_p1_used vaut 1 si la règle a expédié >PLN1 en mode Margolus ;
//    sinon le plan 1 traverse inchangé.
// Source des deux fils de sonde (bits 11-12 des LUT), choisie par la
// déclaration mineure de chaque demi-machine (§7.3.2) :
//   &/CENTERS : centres de l'autre moitié (défaut historique)
//   &/PHASES  : PHASE et PHASE' du run-cycle
//   &/HV      : parités spatiales HORZ et VERT de la cellule
typedef enum {
    FORTH_PROBE_CENTERS = 0,
    FORTH_PROBE_PHASES  = 1,
    FORTH_PROBE_HV      = 2
} ForthProbeSource;

// Un pas d'un run-cycle (§11.5) : origine de la partition (0 = paire,
// 1 = impaire), et les deux bits de phase vus par PHASE et PHASE'.
typedef struct {
    uint8_t org, phase, phase_prime;
} ForthCycleStep;

typedef struct {
    uint8_t lut[FORTH_LUT_SIZE];      // CAM-A : bit0 = plan 0, bit1 = plan 1
    int     lut_neighborhood;         // FORTH_NEIGHBORHOOD_MOORE ou _VONN
    uint8_t lut_b[FORTH_LUT_SIZE];    // CAM-B : bit0 = plan 2, bit1 = plan 3
    int     lut_b_neighborhood;

    // Source des sondes de chaque moitié (déclarations &/CENTERS,
    // &/PHASES, &/HV dans les sections CAM-A / CAM-B).
    int     probe_source_a;           // ForthProbeSource
    int     probe_source_b;
    uint8_t margolus_p0[FORTH_MARG_TABLE_SIZE];
    uint8_t margolus_p1[FORTH_MARG_TABLE_SIZE];
    int     margolus_p1_used;

    // CAM-B a sa PROPRE table Margolus, independante de celle de CAM-A.
    // Dans la vraie machine des annees 80, les deux modules etaient
    // symetriques : n'importe lequel pouvait executer n'importe quel
    // voisinage, Margolus inclus. Cette paire de tables comble le
    // manque de notre portage (qui ne l'autorisait qu'a CAM-A).
    uint8_t margolus_p0_b[FORTH_MARG_TABLE_SIZE];
    uint8_t margolus_p1_b[FORTH_MARG_TABLE_SIZE];
    int     margolus_p1_used_b;

    // Run-cycle déclaré par CYCLE ... END-CYCLE dans la source.
    // cycle_len == 0 : pas de déclaration, le moteur garde ALT-GRID-PH.
    ForthCycleStep cycle[FORTH_CYCLE_MAX];
    int            cycle_len;

    // N/CUSTOM : quand lut_neighborhood == FORTH_NEIGHBORHOOD_CUSTOM,
    // la LUT (dans le champ lut habituel, 2^(count+1) entrees utilisees,
    // RAND sur le bit count) ne suffit pas au moteur : il lui faut AUSSI
    // la liste des offsets a echantillonner. custom_p1_used suit la meme
    // logique que margolus_p1_used : 0 = le plan 1 traverse inchange
    // (une LUT custom sans (0,0,0) declare ne PEUT PAS calculer l'ECHO,
    // donc pas de valeur par defaut raisonnable autre que l'identite).
    ForthCustomNbr custom_nbrs[FORTH_CUSTOM_MAX];
    int            custom_count;
    int            custom_p1_used;
} ForthTables;

void    forth_init(ForthVM *vm);
void    forth_push(ForthVM *vm, int32_t val);
int32_t forth_pop(ForthVM *vm);
void    forth_decode_entry(ForthVM *vm, uint32_t entry);
void    forth_decode_entry_vonn(ForthVM *vm, uint32_t entry);
void    forth_decode_entry_hex(ForthVM *vm, uint32_t entry, int row_odd);
// N/CUSTOM : bit i de l'entree = voisin custom_def[i] (ordre de
// declaration), bit custom_count = RAND. Cable aussi CENTER/CENTER'
// si les triplets (0,0,0)/(0,0,1) font partie de la declaration.
void    forth_decode_entry_custom(ForthVM *vm, uint32_t entry);
void    forth_decode_margolus_corner(ForthVM *vm, uint8_t nw, uint8_t ne,
                                      uint8_t sw, uint8_t se, ForthCorner corner);
// Décodage complet : bloc plan 0, bloc plan 1 (voisins primés) et PHASE
// (les parités spatiales HORZ/VERT en sont déduites, cf. §11.2/§12.8).
// probe_p1_nibble : sonde croisee &CENTER' (chapitre 9/16), un bit par
// coin -- valeur de l'AUTRE demi-machine (plan3 pour CAM-A, plan1 pour
// CAM-B) a la position ABSOLUE de ce coin. 0 si aucune table de l'autre
// moitie n'est chargee (comportement inchange pour les regles qui ne
// l'utilisent pas).
void    forth_decode_margolus_full(ForthVM *vm, uint8_t p0_nibble,
                                    uint8_t p1_nibble, uint8_t phase,
                                    uint8_t phase_prime, uint8_t probe_p1_nibble,
                                    ForthCorner corner);
int32_t forth_eval(ForthVM *vm, const char *rule);

// tables: réceptacle des tables construites (peut être NULL : la source
// est alors analysée mais rien n'est construit).
// Comme dans le vrai CAM Forth : la source déclare son voisinage avec
// N/MOORE (défaut) ou N/MARG, et MAKE-TABLE construit la table du voisinage
// courant. MAKE-TABLE-MARGOLUS est accepté comme raccourci (équivaut à
// N/MARG suivi de MAKE-TABLE). Les mots >PLN0 et >PLN1 expédient le
// résultat vers le plan correspondant ; >PLN2/>PLN3 (CAM-B) sont acceptés
// avec avertissement. Retourne un OU de FORTH_BUILT_LUT /
// FORTH_BUILT_MARGOLUS selon ce qui a été effectivement construit.
int     forth_compile(ForthVM *vm, ForthTables *tables, const char *source);

// Compile une règle "nue" (expression sans définitions) en LUT Moore.
void    make_table(uint8_t *lut, const char *rule);

#endif
