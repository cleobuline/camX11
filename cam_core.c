//
//  cam_core.c
//  cam-8
//

#include "cam_core.h"
#include "cam_forth.h"
#include <stdio.h>
#include <strings.h> // strcasecmp


// Source de bruit du chapitre 8 : xorshift32, largement suffisant pour un
// noisy neighbor et ~100x plus rapide qu'arc4random à 2M tirages/seconde.
static uint32_t g_rng_state = 0xC0FFEE42;
static inline uint32_t rng_next(void) {
    uint32_t x = g_rng_state;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    return g_rng_state = x;
}
static inline uint8_t rng_bit(void) { return (uint8_t)(rng_next() & 1); }

static uint8_t g_lut[FORTH_LUT_SIZE];
static uint8_t g_lut_hex[FORTH_HEX_LUT_SIZE];
static int     g_lut_hex_ready = 0;
static int     g_lut_ready = 0;

// N/CUSTOM (chapitre 7) : LUT dediee + liste des offsets a
// echantillonner. Buffer SEPARE de g_lut, comme g_lut_hex : recompiler
// une regle Moore ne doit pas ecraser la derniere regle custom chargee
// (et reciproquement), chaque mode garde sa table prete.
static uint8_t        g_lut_custom[FORTH_LUT_SIZE];
static ForthCustomNbr g_custom_nbrs[FORTH_CUSTOM_MAX];
static int            g_custom_count = 0;
static int            g_custom_p1_used = 0;
static int            g_custom_ready = 0;

static CAMStepMode g_step_mode = CAM_MODE_LUT;
static int          g_lut_neighborhood = FORTH_NEIGHBORHOOD_MOORE; // MOORE ou VONN
static uint8_t      g_lut_b[FORTH_LUT_SIZE];
static int           g_lut_b_ready = 0;
static int           g_lut_b_neighborhood = FORTH_NEIGHBORHOOD_MOORE;
static uint8_t      g_margolus_table[FORTH_MARG_TABLE_SIZE];
static uint8_t      g_margolus_table_p1[FORTH_MARG_TABLE_SIZE];
static int           g_margolus_p1_used = 0;
static int           g_margolus_ready = 0;
static uint8_t      g_margolus_inv[FORTH_MARG_TABLE_SIZE];
static int           g_margolus_invertible = 0;

// CAM-B a sa PROPRE table Margolus, totalement independante de celle de
// CAM-A -- meme principe que g_lut_b pour le mode LUT. Pas de table
// inverse pour l'instant : le premier usage (generateur de bruit,
// §15.7-16) n'a pas besoin d'etre reversible.
static uint8_t      g_margolus_table_b[FORTH_MARG_TABLE_SIZE];
static uint8_t      g_margolus_table_p1_b[FORTH_MARG_TABLE_SIZE];
static int           g_margolus_p1_used_b = 0;
static int           g_margolus_b_ready = 0;

// Run-cycle courant (§11.5). Défaut : ALT-GRID-PH, le cycle historique —
// origine et PHASE alternent ensemble, PHASE' reste à 0.
static ForthCycleStep g_cycle[FORTH_CYCLE_MAX] = {
    { 0, 0, 0 }, { 1, 1, 0 }
};
static int g_cycle_len = 2;
static int g_probe_a = FORTH_PROBE_CENTERS;
static int g_probe_b = FORTH_PROBE_CENTERS;

static void cycle_reset_default(void) {
    g_cycle[0] = (ForthCycleStep){ 0, 0, 0 };
    g_cycle[1] = (ForthCycleStep){ 1, 1, 0 };
    g_cycle_len = 2;
}

CAMState* cam_create(CAMGridSize size) {
    CAMState *cam = calloc(1, sizeof(CAMState));
    if (!cam) return NULL;

    cam->width  = (uint32_t)size;
    cam->height = (uint32_t)size;
    cam->size   = size;
    cam->margolus_phase = 0;

    cam->plane0_a = calloc(size * size, sizeof(uint8_t));
    cam->plane0_b = calloc(size * size, sizeof(uint8_t));
    cam->plane1_a = calloc(size * size, sizeof(uint8_t));
    cam->plane1_b = calloc(size * size, sizeof(uint8_t));
    cam->plane2_a = calloc(size * size, sizeof(uint8_t));
    cam->plane2_b = calloc(size * size, sizeof(uint8_t));
    cam->plane3_a = calloc(size * size, sizeof(uint8_t));
    cam->plane3_b = calloc(size * size, sizeof(uint8_t));

    if (!cam->plane0_a || !cam->plane0_b ||
        !cam->plane1_a || !cam->plane1_b ||
        !cam->plane2_a || !cam->plane2_b ||
        !cam->plane3_a || !cam->plane3_b) {
        cam_destroy(cam);
        return NULL;
    }

    return cam;
}
void cam_apply_lut(uint8_t *lut) {
    memcpy(g_lut, lut, FORTH_LUT_SIZE);
    g_lut_ready = 1;
}


// Calcule la table inverse de g_margolus_table si possible.
// Condition VRAIMENT nécessaire (chapitre 14) : pour CHAQUE valeur du
// nibble du plan 1 (mur/décor, jamais réécrit si non expédié via
// >PLN1) et CHAQUE couple de phase utilisé, la carte plan0-entrée ->
// plan0-sortie doit être une bijection sur les 16 blocs -- ET la sortie
// doit être indépendante de RAND (on ne peut pas inverser un tirage de
// pile ou face). Le plan 1 lui-même n'a PAS besoin d'être identique
// selon les cas : une règle peut très bien se comporter différemment
// selon qu'un bloc touche un mur ou non (ex. GAZ-MURS, §15.2), tant que
// CHAQUE comportement pris isolément reste une bijection. L'ancienne
// version de cette fonction exigeait, par erreur, que la sortie soit
// la MÊME quel que soit le plan 1 -- une condition suffisante mais pas
// nécessaire, qui rejetait à tort toute règle avec parois.
static void margolus_compute_inverse(void) {
    g_margolus_invertible = 0;
    if (g_margolus_p1_used) return; // les murs doivent rester éternels

    // indépendance vis-à-vis de RAND (bit 9) ET de la sonde croisée
    // &CENTER' (bits 11-14) -- une regle qui lit CAM-B (ex. DENDRITE
    // pilotee par le generateur de bruit, p.167-168) ne peut pas etre
    // inversee sans aussi connaitre l'HISTOIRE de CAM-B : on la traite
    // honnetement comme une source de hasard externe, au meme titre
    // que RAND. Le plan 1 (bits 4-7) reste la SEULE dependance admise
    // (bijection par tranche, cf. GAZ-MURS).
    for (uint32_t idx = 0; idx < FORTH_MARG_TABLE_SIZE; idx++) {
        uint32_t base = idx & ~(uint32_t)(0x200 | 0x7800); // RAND + sonde à 0
        if (g_margolus_table[idx] != g_margolus_table[base]) return;
    }

    // bijection et inversion, par couple (PHASE, PHASE') ET par valeur
    // du nibble du plan 1 : g_margolus_inv est indexé par les MÊMES
    // bits que g_margolus_table (p0 | p1<<4 | phase<<8 | phase'<<10),
    // donc chaque tranche de plan 1 reçoit sa PROPRE permutation inverse
    // -- pas une permutation unique diffusée à toutes les tranches.
    int bijective[4] = {0, 0, 0, 0};
    for (uint32_t pp = 0; pp < 4; pp++) {
        uint32_t hi = ((pp & 1) << 8) | (((pp >> 1) & 1) << 10);
        int pp_ok = 1;

        for (uint32_t p1 = 0; p1 < 16 && pp_ok; p1++) {
            uint32_t p1_bits = p1 << 4;
            uint8_t seen[16] = {0};
            for (uint32_t in = 0; in < 16; in++) {
                uint8_t out = g_margolus_table[in | p1_bits | hi] & 0xF;
                if (seen[out]) { pp_ok = 0; break; }
                seen[out] = 1;
            }
        }
        bijective[pp] = pp_ok;
        if (!pp_ok) continue;

        for (uint32_t p1 = 0; p1 < 16; p1++) {
            uint32_t p1_bits = p1 << 4;
            for (uint32_t in = 0; in < 16; in++) {
                uint8_t out = g_margolus_table[in | p1_bits | hi] & 0xF;
                // diffuse sur les 2 valeurs de RAND ET les 16 valeurs de
                // la sonde croisee : l'independance verifiee plus haut
                // garantit que n'importe laquelle donne le meme resultat.
                for (uint32_t r = 0; r < 2; r++)
                    for (uint32_t pr = 0; pr < 16; pr++)
                        g_margolus_inv[out | p1_bits | hi | (r << 9) | (pr << 11)] = (uint8_t)in;
            }
        }
    }

    // inversible ssi chaque couple de phases UTILISÉ par le cycle l'est
    for (int s = 0; s < g_cycle_len; s++) {
        int pp = g_cycle[s].phase | (g_cycle[s].phase_prime << 1);
        if (!bijective[pp]) return;
    }
    g_margolus_invertible = 1;
}

int cam_can_reverse(void) {
    return (g_step_mode == CAM_MODE_MARGOLUS) && g_margolus_ready
           && g_margolus_invertible;
}

void cam_apply_forth_tables(const ForthTables *tables, int built) {
    // Margolus d'abord, LUT ensuite : si la source contient les deux,
    // le mode Moore/LUT l'emporte (comportement historique), mais la
    // table Margolus reste chargée.
    if (built & FORTH_BUILT_MARGOLUS) {
        memcpy(g_margolus_table,    tables->margolus_p0, FORTH_MARG_TABLE_SIZE);
        memcpy(g_margolus_table_p1, tables->margolus_p1, FORTH_MARG_TABLE_SIZE);
        g_margolus_p1_used = tables->margolus_p1_used;
        g_margolus_ready = 1;
        g_step_mode = CAM_MODE_MARGOLUS;
        if (tables->cycle_len > 0) {
            memcpy(g_cycle, tables->cycle,
                   sizeof(ForthCycleStep) * tables->cycle_len);
            g_cycle_len = tables->cycle_len;
        } else {
            cycle_reset_default();
        }
        margolus_compute_inverse();
    }
    if (built & FORTH_BUILT_LUT) {
        if (tables->lut_neighborhood == FORTH_NEIGHBORHOOD_HEX) {
            memcpy(g_lut_hex, tables->lut, FORTH_HEX_LUT_SIZE);
            g_lut_hex_ready = 1;
            g_step_mode = CAM_MODE_HEX;
        } else if (tables->lut_neighborhood == FORTH_NEIGHBORHOOD_CUSTOM) {
            memcpy(g_lut_custom, tables->lut, FORTH_LUT_SIZE);
            memcpy(g_custom_nbrs, tables->custom_nbrs, sizeof(g_custom_nbrs));
            g_custom_count   = tables->custom_count;
            g_custom_p1_used = tables->custom_p1_used;
            g_custom_ready   = 1;
            g_step_mode = CAM_MODE_CUSTOM;
        } else {
            memcpy(g_lut, tables->lut, FORTH_LUT_SIZE);
            g_lut_neighborhood = tables->lut_neighborhood;
            g_lut_ready = 1;
            g_step_mode = CAM_MODE_LUT;
        }
    }
    // La source décrit toute la machine : CAM-B n'est active que si cette
    // compilation a construit sa table. Les trois etats (rien, LUT,
    // Margolus) sont mutuellement exclusifs pour CAM-B, comme pour CAM-A.
    if (built & (FORTH_BUILT_LUT | FORTH_BUILT_MARGOLUS | FORTH_BUILT_LUT_B | FORTH_BUILT_MARGOLUS_B)) {
        if (built & FORTH_BUILT_MARGOLUS_B) {
            memcpy(g_margolus_table_b,    tables->margolus_p0_b, FORTH_MARG_TABLE_SIZE);
            memcpy(g_margolus_table_p1_b, tables->margolus_p1_b, FORTH_MARG_TABLE_SIZE);
            g_margolus_p1_used_b = tables->margolus_p1_used_b;
            g_margolus_b_ready = 1;
            g_lut_b_ready = 0;
        } else if (built & FORTH_BUILT_LUT_B) {
            memcpy(g_lut_b, tables->lut_b, FORTH_LUT_SIZE);
            g_lut_b_neighborhood = tables->lut_b_neighborhood;
            g_lut_b_ready = 1;
            g_margolus_b_ready = 0;
        } else {
            g_lut_b_ready = 0;
            g_margolus_b_ready = 0;
        }
        g_probe_a = tables->probe_source_a;
        g_probe_b = tables->probe_source_b;
    }
}
void cam_destroy(CAMState *cam) {
    if (!cam) return;
    free(cam->plane0_a);
    free(cam->plane0_b);
    free(cam->plane1_a);
    free(cam->plane1_b);
    free(cam->plane2_a);
    free(cam->plane2_b);
    free(cam->plane3_a);
    free(cam->plane3_b);
    free(cam);
}

void cam_swap_buffers(CAMState *cam) {
    uint8_t *tmp;
    tmp = cam->plane0_a; cam->plane0_a = cam->plane0_b; cam->plane0_b = tmp;
    tmp = cam->plane1_a; cam->plane1_a = cam->plane1_b; cam->plane1_b = tmp;
    tmp = cam->plane2_a; cam->plane2_a = cam->plane2_b; cam->plane2_b = tmp;
    tmp = cam->plane3_a; cam->plane3_a = cam->plane3_b; cam->plane3_b = tmp;
}

void cam_set_rule(const char *rule) {
    // Détection intelligente du type de code Forth.
    if (strstr(rule, ":") != NULL || strstr(rule, "MAKE-TABLE") != NULL) {
        ForthVM vm;
        forth_init(&vm);

        ForthTables tables;
        memset(&tables, 0, sizeof(tables));
        int built = forth_compile(&vm, &tables, rule);
        cam_apply_forth_tables(&tables, built);
    } else {
        make_table(g_lut, rule);
        g_lut_neighborhood = FORTH_NEIGHBORHOOD_MOORE;
        g_lut_ready = 1;
        cam_set_step_mode(CAM_MODE_LUT); // une règle Forth "nue" implique le voisinage de Moore
    }
}

static void cam_step_lut(CAMState *cam) {
    if (!g_lut_ready) {
        cam_set_rule("NORTH SOUTH XOR EAST WEST XOR XOR");
    }

    uint32_t w = cam->width;
    uint32_t h = cam->height;

    // Le run-cycle bat à chaque STEP, quel que soit le voisinage : les
    // sondes &/PHASES lisent le pas courant.
    ForthCycleStep cyc = g_cycle[cam->margolus_phase % g_cycle_len];
    (void)cyc;

    for (uint32_t y = 0; y < h; y++) {
        uint32_t y_up   = (y - 1 + h) % h;
        uint32_t y_down = (y + 1) % h;

        for (uint32_t x = 0; x < w; x++) {
            uint32_t x_left  = (x - 1 + w) % w;
            uint32_t x_right = (x + 1) % w;

            // lecture plane 0 (vivant)
            uint8_t center = cam->plane0_a[y * w + x]            ? 1 : 0;
            uint8_t north  = cam->plane0_a[y_up   * w + x]        ? 1 : 0;
            uint8_t south  = cam->plane0_a[y_down * w + x]        ? 1 : 0;
            uint8_t east   = cam->plane0_a[y * w + x_right]       ? 1 : 0;
            uint8_t west   = cam->plane0_a[y * w + x_left]        ? 1 : 0;

            // lecture plane 1 (réfractaire) — centre dans les deux cas
            uint8_t center_prime = cam->plane1_a[y * w + x] ? 1 : 0;

            uint32_t entry;
            if (g_lut_neighborhood == FORTH_NEIGHBORHOOD_VONN) {
                // N/VONN (tableau 7.2) : les diagonales sont remplacées
                // par les voisins von Neumann du plan 1.
                uint8_t east_p  = cam->plane1_a[y * w + x_right]  ? 1 : 0;
                uint8_t west_p  = cam->plane1_a[y * w + x_left]   ? 1 : 0;
                uint8_t south_p = cam->plane1_a[y_down * w + x]   ? 1 : 0;
                uint8_t north_p = cam->plane1_a[y_up   * w + x]   ? 1 : 0;

                entry = center
                      | (center_prime << 1)
                      | (east_p  << 2)
                      | (west_p  << 3)
                      | (south_p << 4)
                      | (north_p << 5)
                      | (east    << 6)
                      | (west    << 7)
                      | (south   << 8)
                      | (north   << 9);
            } else {
                // N/MOORE : les 8 voisins du plan 0
                uint8_t neast  = cam->plane0_a[y_up   * w + x_right]  ? 1 : 0;
                uint8_t nwest  = cam->plane0_a[y_up   * w + x_left]   ? 1 : 0;
                uint8_t seast  = cam->plane0_a[y_down * w + x_right]  ? 1 : 0;
                uint8_t swest  = cam->plane0_a[y_down * w + x_left]   ? 1 : 0;

                entry = center
                      | (center_prime << 1)
                      | (north  << 2)
                      | (south  << 3)
                      | (east   << 4)
                      | (west   << 5)
                      | (neast  << 6)
                      | (nwest  << 7)
                      | (seast  << 8)
                      | (swest  << 9);
            }

            // pseudo-voisin bruyant : bit 10 de l'entrée (chapitre 8)
            entry |= ((uint32_t)rng_bit() << 10);

            // sondes (bits 11-12) : leur source dépend de la déclaration
            // mineure de CAM-A (§7.3.2)
            uint8_t pa0, pa1;
            if (g_probe_a == FORTH_PROBE_PHASES) {
                pa0 = cyc.phase; pa1 = cyc.phase_prime;
            } else if (g_probe_a == FORTH_PROBE_HV) {
                pa0 = x & 1; pa1 = y & 1;
            } else {
                pa0 = cam->plane2_a[y * w + x] ? 1 : 0;
                pa1 = cam->plane3_a[y * w + x] ? 1 : 0;
            }
            entry |= ((uint32_t)pa0 << 11) | ((uint32_t)pa1 << 12);

            // nouvelle valeur des plans 0-1 via la LUT de CAM-A
            uint8_t out = g_lut[entry];
            cam->plane0_b[y * w + x] = out & 1;
            cam->plane1_b[y * w + x] = (out >> 1) & 1;

            // --- demi-machine B (plans 2-3), simultanée : elle lit le
            // même instant t que CAM-A, avec son propre voisinage sur
            // ses propres plans, et sonde les centres de CAM-A.
            if (g_lut_b_ready) {
                uint32_t eb;
                if (g_lut_b_neighborhood == FORTH_NEIGHBORHOOD_VONN) {
                    eb = (cam->plane2_a[y * w + x]        ? 1 : 0)
                       | ((cam->plane3_a[y * w + x]       ? 1 : 0) << 1)
                       | ((cam->plane3_a[y * w + x_right] ? 1 : 0) << 2)
                       | ((cam->plane3_a[y * w + x_left]  ? 1 : 0) << 3)
                       | ((cam->plane3_a[y_down * w + x]  ? 1 : 0) << 4)
                       | ((cam->plane3_a[y_up   * w + x]  ? 1 : 0) << 5)
                       | ((cam->plane2_a[y * w + x_right] ? 1 : 0) << 6)
                       | ((cam->plane2_a[y * w + x_left]  ? 1 : 0) << 7)
                       | ((cam->plane2_a[y_down * w + x]  ? 1 : 0) << 8)
                       | ((cam->plane2_a[y_up   * w + x]  ? 1 : 0) << 9);
                } else {
                    eb = (cam->plane2_a[y * w + x]              ? 1 : 0)
                       | ((cam->plane3_a[y * w + x]             ? 1 : 0) << 1)
                       | ((cam->plane2_a[y_up   * w + x]        ? 1 : 0) << 2)
                       | ((cam->plane2_a[y_down * w + x]        ? 1 : 0) << 3)
                       | ((cam->plane2_a[y * w + x_right]       ? 1 : 0) << 4)
                       | ((cam->plane2_a[y * w + x_left]        ? 1 : 0) << 5)
                       | ((cam->plane2_a[y_up   * w + x_right]  ? 1 : 0) << 6)
                       | ((cam->plane2_a[y_up   * w + x_left]   ? 1 : 0) << 7)
                       | ((cam->plane2_a[y_down * w + x_right]  ? 1 : 0) << 8)
                       | ((cam->plane2_a[y_down * w + x_left]   ? 1 : 0) << 9);
                }
                eb |= ((uint32_t)rng_bit() << 10);
                // sondes de CAM-B, selon sa propre déclaration mineure
                uint8_t pb0, pb1;
                if (g_probe_b == FORTH_PROBE_PHASES) {
                    pb0 = cyc.phase; pb1 = cyc.phase_prime;
                } else if (g_probe_b == FORTH_PROBE_HV) {
                    pb0 = x & 1; pb1 = y & 1;
                } else {
                    pb0 = center; pb1 = center_prime;
                }
                eb |= ((uint32_t)pb0 << 11) | ((uint32_t)pb1 << 12);

                uint8_t outb = g_lut_b[eb];
                cam->plane2_b[y * w + x] = outb & 1;
                cam->plane3_b[y * w + x] = (outb >> 1) & 1;
            } else {
                cam->plane2_b[y * w + x] = cam->plane2_a[y * w + x];
                cam->plane3_b[y * w + x] = cam->plane3_a[y * w + x];
            }
        }
    }

    cam->margolus_phase = (cam->margolus_phase + 1) % g_cycle_len;
    cam_swap_buffers(cam);
}

// --- Margolus : construction des préréglages ---
//
// Encodage d'un bloc 2x2 sur 4 bits :
//   bit0 = NW, bit1 = NE, bit2 = SW, bit3 = SE
//
// popcount4 / rotate180_4 / invert4 sont de petits utilitaires sur ce nibble.

static int popcount4(uint8_t nibble) {
    int n = 0;
    for (int i = 0; i < 4; i++) {
        if (nibble & (1 << i)) n++;
    }
    return n;
}

static uint8_t invert4(uint8_t nibble) {
    return (~nibble) & 0x0F;
}

static uint8_t rotate180_4(uint8_t nibble) {
    // NW<->SE, NE<->SW
    uint8_t nw = (nibble >> 0) & 1;
    uint8_t ne = (nibble >> 1) & 1;
    uint8_t sw = (nibble >> 2) & 1;
    uint8_t se = (nibble >> 3) & 1;
    return (uint8_t)(se | (sw << 1) | (ne << 2) | (nw << 3));
}

// "Critters" (Toffoli) : on inverse toujours le bloc ; si le bloc D'ORIGINE
// contenait exactement 2 cellules vivantes, on tourne en plus de 180°.
// Réversible par construction (inversion et rotation 180° sont chacune
// leur propre inverse, et la condition porte sur l'état d'origine).
static void build_critters_table(uint8_t table[16]) {
    for (int in = 0; in < 16; in++) {
        uint8_t out = invert4((uint8_t)in);
        if (popcount4((uint8_t)in) == 2) {
            out = rotate180_4(out);
        }
        table[in] = out;
    }
}

// BBM (Billiard Ball Model), table 18.1 du bouquin :
//  - une particule seule se propage vers son coin opposé (comme dans
//    SWAP-ON-DIAG / HPP-GAS) — c'est ce qui fait avancer les billes ;
//  - deux particules sur une diagonale entrent en collision et repartent
//    sur l'autre diagonale ;
//  - paires adjacentes, trois particules (miroirs) et bloc plein sont
//    inchangés — c'est ce qui permet aux miroirs de rester en place.
static void build_bbm_table(uint8_t table[16]) {
    for (int in = 0; in < 16; in++) {
        table[in] = (uint8_t)in; // identité par défaut
    }
    // propagation : particule isolée → coin opposé
    table[1 << 0] = 1 << 3; // NW → SE
    table[1 << 1] = 1 << 2; // NE → SW
    table[1 << 2] = 1 << 1; // SW → NE
    table[1 << 3] = 1 << 0; // SE → NW
    // collision : NW+SE (bits 0 et 3) <-> NE+SW (bits 1 et 2)
    uint8_t diag1 = (1 << 0) | (1 << 3); // NW, SE
    uint8_t diag2 = (1 << 1) | (1 << 2); // NE, SW
    table[diag1] = diag2;
    table[diag2] = diag1;
}

void cam_set_step_mode(CAMStepMode mode) {
    g_step_mode = mode;
}

CAMStepMode cam_get_step_mode(void) {
    return g_step_mode;
}

void cam_set_margolus_table(const uint8_t table[16]) {
    // Un preset 16 entrées ne dépend ni du plan 1 ni de la phase : on le
    // réplique sur toute la table complète (512 entrées).
    for (uint32_t idx = 0; idx < FORTH_MARG_TABLE_SIZE; idx++) {
        g_margolus_table[idx] = table[idx & 0xF];
    }
    g_margolus_p1_used = 0; // les presets n'expédient rien vers le plan 1
    g_margolus_ready = 1;
    g_lut_b_ready = 0;
    g_step_mode = CAM_MODE_MARGOLUS;
    cycle_reset_default();
    margolus_compute_inverse();
}

int cam_set_margolus_preset(const char *name) {
    uint8_t table[16];

    if (strcasecmp(name, "CRITTERS") == 0) {
        build_critters_table(table);
    } else if (strcasecmp(name, "BBM") == 0) {
        build_bbm_table(table);
    } else {
        return 0;
    }

    cam_set_margolus_table(table);
    return 1;
}

static void cam_step_margolus(CAMState *cam) {
    if (!g_margolus_ready) {
        cam_set_margolus_preset("CRITTERS");
    }

    uint32_t w = cam->width;
    uint32_t h = cam->height;

    // le pas courant du run-cycle pilote la partition et les phases --
    // CALCULE ICI, avant les deux boucles de blocs (CAM-A et CAM-B
    // partagent la MEME horloge physique dans la vraie machine).
    ForthCycleStep step = g_cycle[cam->margolus_phase % g_cycle_len];
    int offset = step.org ? 1 : 0;

    // Plans 2-3 (CAM-B) : si une table Margolus dediee a ete construite
    // pour cette moitie, elle tourne EXACTEMENT comme CAM-A -- meme
    // partition de blocs, meme run-cycle (les deux modules partagent la
    // meme horloge physique dans la vraie machine). Sinon, comportement
    // historique : passage inchange.
    if (g_margolus_b_ready) {
        for (uint32_t by = 0; by < h / 2; by++) {
            uint32_t y0 = (2 * by + offset) % h;
            uint32_t y1 = (2 * by + offset + 1) % h;
            for (uint32_t bx = 0; bx < w / 2; bx++) {
                uint32_t x0 = (2 * bx + offset) % w;
                uint32_t x1 = (2 * bx + offset + 1) % w;

                uint8_t nw = cam->plane2_a[y0 * w + x0] ? 1 : 0;
                uint8_t ne = cam->plane2_a[y0 * w + x1] ? 1 : 0;
                uint8_t sw = cam->plane2_a[y1 * w + x0] ? 1 : 0;
                uint8_t se = cam->plane2_a[y1 * w + x1] ? 1 : 0;

                uint8_t nw1 = cam->plane3_a[y0 * w + x0] ? 1 : 0;
                uint8_t ne1 = cam->plane3_a[y0 * w + x1] ? 1 : 0;
                uint8_t sw1 = cam->plane3_a[y1 * w + x0] ? 1 : 0;
                uint8_t se1 = cam->plane3_a[y1 * w + x1] ? 1 : 0;

                uint8_t p0_nibble = nw  | (ne  << 1) | (sw  << 2) | (se  << 3);
                uint8_t p1_nibble = nw1 | (ne1 << 1) | (sw1 << 2) | (se1 << 3);

                // CAM-B ne sonde pas encore CAM-A en retour (aucune regle
                // du bestiaire n'en a besoin pour l'instant) -- le nibble
                // de sonde reste a 0, mais doit exister pour que l'index
                // ait la MEME largeur que la table construite a la
                // compilation (32768 entrees, meme mise en page que CAM-A).
                uint32_t idx = p0_nibble
                             | ((uint32_t)p1_nibble << 4)
                             | ((uint32_t)step.phase << 8)
                             | ((uint32_t)rng_bit() << 9)
                             | ((uint32_t)step.phase_prime << 10);
                uint8_t out_nibble = g_margolus_table_b[idx];

                cam->plane2_b[y0 * w + x0] = (out_nibble >> 0) & 1;
                cam->plane2_b[y0 * w + x1] = (out_nibble >> 1) & 1;
                cam->plane2_b[y1 * w + x0] = (out_nibble >> 2) & 1;
                cam->plane2_b[y1 * w + x1] = (out_nibble >> 3) & 1;

                if (g_margolus_p1_used_b) {
                    uint8_t p1_out = g_margolus_table_p1_b[idx];
                    cam->plane3_b[y0 * w + x0] = (p1_out >> 0) & 1;
                    cam->plane3_b[y0 * w + x1] = (p1_out >> 1) & 1;
                    cam->plane3_b[y1 * w + x0] = (p1_out >> 2) & 1;
                    cam->plane3_b[y1 * w + x1] = (p1_out >> 3) & 1;
                } else {
                    cam->plane3_b[y0 * w + x0] = cam->plane3_a[y0 * w + x0];
                    cam->plane3_b[y0 * w + x1] = cam->plane3_a[y0 * w + x1];
                    cam->plane3_b[y1 * w + x0] = cam->plane3_a[y1 * w + x0];
                    cam->plane3_b[y1 * w + x1] = cam->plane3_a[y1 * w + x1];
                }
            }
        }
    } else {
        memcpy(cam->plane2_b, cam->plane2_a, (size_t)w * h);
        memcpy(cam->plane3_b, cam->plane3_a, (size_t)w * h);
    }

    // Plan 1 : si la règle expédie >PLN1, il sera écrit bloc par bloc dans
    // la boucle ci-dessous (table p1 indexée par le bloc du plan 0). Sinon,
    // comportement historique : il traverse inchangé.
    if (!g_margolus_p1_used) {
        memcpy(cam->plane1_b, cam->plane1_a, (size_t)w * h);
    }

    // le pas courant du run-cycle pilote la partition et les phases

    for (uint32_t by = 0; by < h / 2; by++) {
        uint32_t y0 = (2 * by + offset) % h;
        uint32_t y1 = (2 * by + offset + 1) % h;

        for (uint32_t bx = 0; bx < w / 2; bx++) {
            uint32_t x0 = (2 * bx + offset) % w;
            uint32_t x1 = (2 * bx + offset + 1) % w;

            uint8_t nw = cam->plane0_a[y0 * w + x0] ? 1 : 0;
            uint8_t ne = cam->plane0_a[y0 * w + x1] ? 1 : 0;
            uint8_t sw = cam->plane0_a[y1 * w + x0] ? 1 : 0;
            uint8_t se = cam->plane0_a[y1 * w + x1] ? 1 : 0;

            uint8_t nw1 = cam->plane1_a[y0 * w + x0] ? 1 : 0;
            uint8_t ne1 = cam->plane1_a[y0 * w + x1] ? 1 : 0;
            uint8_t sw1 = cam->plane1_a[y1 * w + x0] ? 1 : 0;
            uint8_t se1 = cam->plane1_a[y1 * w + x1] ? 1 : 0;

            uint8_t p0_nibble = nw  | (ne  << 1) | (sw  << 2) | (se  << 3);
            uint8_t p1_nibble = nw1 | (ne1 << 1) | (sw1 << 2) | (se1 << 3);

            // Sonde croisee &CENTER' (chapitre 9/16) : lit le plan 3
            // (CAM-B) aux 4 positions ABSOLUES de ce bloc. Ne s'active
            // reellement que si une regle Margolus l'utilise vraiment
            // (ex. DENDRITE piloté par le generateur de bruit de CAM-B,
            // p.167-168) ; sinon ces bits restent a 0 sans consequence
            // puisqu'aucune regle ne les lit.
            uint8_t nwp = cam->plane3_a[y0 * w + x0] ? 1 : 0;
            uint8_t nep = cam->plane3_a[y0 * w + x1] ? 1 : 0;
            uint8_t swp = cam->plane3_a[y1 * w + x0] ? 1 : 0;
            uint8_t sep = cam->plane3_a[y1 * w + x1] ? 1 : 0;
            uint8_t probe_p1_nibble = nwp | (nep << 1) | (swp << 2) | (sep << 3);

            // index complet : plan 0 | plan 1 | PHASE | RAND | PHASE' | sonde
            uint32_t idx = p0_nibble
                         | ((uint32_t)p1_nibble << 4)
                         | ((uint32_t)step.phase << 8)
                         | ((uint32_t)rng_bit() << 9)
                         | ((uint32_t)step.phase_prime << 10)
                         | ((uint32_t)probe_p1_nibble << 11);
            uint8_t out_nibble = g_margolus_table[idx];

            cam->plane0_b[y0 * w + x0] = (out_nibble >> 0) & 1;
            cam->plane0_b[y0 * w + x1] = (out_nibble >> 1) & 1;
            cam->plane0_b[y1 * w + x0] = (out_nibble >> 2) & 1;
            cam->plane0_b[y1 * w + x1] = (out_nibble >> 3) & 1;

            if (g_margolus_p1_used) {
                uint8_t p1_out = g_margolus_table_p1[idx];
                cam->plane1_b[y0 * w + x0] = (p1_out >> 0) & 1;
                cam->plane1_b[y0 * w + x1] = (p1_out >> 1) & 1;
                cam->plane1_b[y1 * w + x0] = (p1_out >> 2) & 1;
                cam->plane1_b[y1 * w + x1] = (p1_out >> 3) & 1;
            }
        }
    }

    cam->margolus_phase = (cam->margolus_phase + 1) % g_cycle_len;

    cam_swap_buffers(cam);
}


// Un pas en ARRIÈRE (chapitre 14) : rebascule d'abord la phase pour
// retrouver la partition du dernier pas exécuté, puis applique la table
// INVERSE sur cette partition. Aucun basculement après : la phase est
// alors prête pour rejouer ce même pas en avant.
int cam_step_back(CAMState *cam) {
    if (!cam_can_reverse()) return 0;

    uint32_t w = cam->width;
    uint32_t h = cam->height;

    cam->margolus_phase = (cam->margolus_phase + g_cycle_len - 1) % g_cycle_len;
    ForthCycleStep step = g_cycle[cam->margolus_phase]; // le dernier pas exécuté
    int offset = step.org ? 1 : 0;

    // plans 1, 2 et 3 : traversent inchangés
    memcpy(cam->plane1_b, cam->plane1_a, (size_t)w * h);
    memcpy(cam->plane2_b, cam->plane2_a, (size_t)w * h);
    memcpy(cam->plane3_b, cam->plane3_a, (size_t)w * h);

    for (uint32_t by = 0; by < h / 2; by++) {
        uint32_t y0 = (2 * by + offset) % h;
        uint32_t y1 = (2 * by + offset + 1) % h;

        for (uint32_t bx = 0; bx < w / 2; bx++) {
            uint32_t x0 = (2 * bx + offset) % w;
            uint32_t x1 = (2 * bx + offset + 1) % w;

            uint8_t nw = cam->plane0_a[y0 * w + x0] ? 1 : 0;
            uint8_t ne = cam->plane0_a[y0 * w + x1] ? 1 : 0;
            uint8_t sw = cam->plane0_a[y1 * w + x0] ? 1 : 0;
            uint8_t se = cam->plane0_a[y1 * w + x1] ? 1 : 0;

            // Le plan 1 (murs/décor) doit être lu ICI AUSSI, exactement
            // comme le pas avant : la permutation inverse à appliquer
            // dépend de la présence d'un mur dans ce bloc précis (voir
            // margolus_compute_inverse, une tranche par valeur de p1).
            // Avant ce correctif, cam_step_back ignorait totalement le
            // plan 1 -- toute règle dont la sortie depend des murs
            // (GAZ-MURS) etait donc mal inversee, meme quand le moteur
            // la déclarait à tort réversible.
            uint8_t nw1 = cam->plane1_a[y0 * w + x0] ? 1 : 0;
            uint8_t ne1 = cam->plane1_a[y0 * w + x1] ? 1 : 0;
            uint8_t sw1 = cam->plane1_a[y1 * w + x0] ? 1 : 0;
            uint8_t se1 = cam->plane1_a[y1 * w + x1] ? 1 : 0;
            uint8_t p1_nibble = nw1 | (ne1 << 1) | (sw1 << 2) | (se1 << 3);

            uint32_t idx = (uint32_t)(nw | (ne << 1) | (sw << 2) | (se << 3))
                         | ((uint32_t)p1_nibble << 4)
                         | ((uint32_t)step.phase << 8)
                         | ((uint32_t)step.phase_prime << 10);
            uint8_t out = g_margolus_inv[idx];

            cam->plane0_b[y0 * w + x0] = (out >> 0) & 1;
            cam->plane0_b[y0 * w + x1] = (out >> 1) & 1;
            cam->plane0_b[y1 * w + x0] = (out >> 2) & 1;
            cam->plane0_b[y1 * w + x1] = (out >> 3) & 1;
        }
    }

    cam_swap_buffers(cam);
    return 1;
}

// N/HEX (chapitre 16, FHP) : grille pseudo-hexagonale par decalage en
// quinconce ("brickwork", encodage even-r classique). Les lignes paires
// et impaires lisent leurs 6 voisins a 60 degres a des offsets
// DIFFERENTS -- c'est cette bascule qui simule l'hexagone sur une
// grille carree. Regle mono-plan : seul le plan 0 evolue ; les autres
// plans traversent inchanges (decor, compteurs...).
static void cam_step_hex(CAMState *cam) {
    if (!g_lut_hex_ready) return; // pas de regle chargee : rien a faire

    uint32_t w = cam->width;
    uint32_t h = cam->height;

    for (uint32_t y = 0; y < h; y++) {
        uint32_t y_up   = (y - 1 + h) % h;
        uint32_t y_down = (y + 1) % h;
        int row_odd = y & 1;

        for (uint32_t x = 0; x < w; x++) {
            uint32_t x_right = (x + 1) % w;
            uint32_t x_left  = (x - 1 + w) % w;

            uint8_t center = cam->plane0_a[y * w + x] ? 1 : 0;
            uint8_t e = cam->plane0_a[y * w + x_right] ? 1 : 0;
            uint8_t we = cam->plane0_a[y * w + x_left]  ? 1 : 0;

            uint8_t ne, nw, se, sw;
            if (row_odd) {
                // ligne impaire (encodage even-r) : NE/SE decales a l'est
                ne = cam->plane0_a[y_up   * w + x_right] ? 1 : 0;
                nw = cam->plane0_a[y_up   * w + x]       ? 1 : 0;
                se = cam->plane0_a[y_down * w + x_right] ? 1 : 0;
                sw = cam->plane0_a[y_down * w + x]       ? 1 : 0;
            } else {
                // ligne paire : NW/SW decales a l'ouest
                ne = cam->plane0_a[y_up   * w + x]       ? 1 : 0;
                nw = cam->plane0_a[y_up   * w + x_left]  ? 1 : 0;
                se = cam->plane0_a[y_down * w + x]       ? 1 : 0;
                sw = cam->plane0_a[y_down * w + x_left]  ? 1 : 0;
            }

            uint32_t entry = center
                           | ((uint32_t)e  << 1)
                           | ((uint32_t)ne << 2)
                           | ((uint32_t)nw << 3)
                           | ((uint32_t)we << 4)
                           | ((uint32_t)sw << 5)
                           | ((uint32_t)se << 6)
                           | ((uint32_t)rng_bit() << 7);

            cam->plane0_b[y * w + x] = g_lut_hex[entry] ? 1 : 0;
        }
    }

    // les autres plans traversent inchanges (regle mono-plan)
    memcpy(cam->plane1_b, cam->plane1_a, (size_t)w * h);
    memcpy(cam->plane2_b, cam->plane2_a, (size_t)w * h);
    memcpy(cam->plane3_b, cam->plane3_a, (size_t)w * h);

    cam_swap_buffers(cam);
}

// N/CUSTOM (chapitre 7) : voisinage arbitraire. Pour chaque cellule, le
// moteur echantillonne les offsets declares (convention boussole :
// dx > 0 = est, dy > 0 = nord, donc y - dy en memoire ou y croit vers
// le bas), assemble l'entree bit par bit dans l'ORDRE de declaration,
// ajoute RAND en dernier bit, et consulte la LUT custom. Regle de la
// demi-machine A uniquement pour l'instant : les plans 2-3 traversent
// inchanges, comme en mode HEX.
static void cam_step_custom(CAMState *cam) {
    if (!g_custom_ready || g_custom_count <= 0) return; // pas de regle chargee

    uint32_t w = cam->width;
    uint32_t h = cam->height;
    int n = g_custom_count;

    // Plan 1 : ecrit bloc la boucle si la regle expedie >PLN1, sinon il
    // traverse inchange -- sans (0,0,0) declare, une LUT custom ne peut
    // meme pas calculer l'ECHO, l'identite est le seul defaut honnete.
    if (!g_custom_p1_used)
        memcpy(cam->plane1_b, cam->plane1_a, (size_t)w * h);
    memcpy(cam->plane2_b, cam->plane2_a, (size_t)w * h);
    memcpy(cam->plane3_b, cam->plane3_a, (size_t)w * h);

    for (uint32_t y = 0; y < h; y++) {
        // Rangee source de chaque voisin, precalculee UNE fois par ligne
        // (le modulo signe ne se paie qu'ici, pas a chaque cellule).
        size_t         rowbase[FORTH_CUSTOM_MAX];
        const uint8_t *src[FORTH_CUSTOM_MAX];
        for (int i = 0; i < n; i++) {
            long ny = (long)y - g_custom_nbrs[i].dy; // dy > 0 = nord
            ny %= (long)h; if (ny < 0) ny += (long)h;
            rowbase[i] = (size_t)ny * w;
            src[i] = g_custom_nbrs[i].plane ? cam->plane1_a : cam->plane0_a;
        }

        for (uint32_t x = 0; x < w; x++) {
            uint32_t entry = 0;
            for (int i = 0; i < n; i++) {
                long nx = (long)x + g_custom_nbrs[i].dx;
                nx %= (long)w; if (nx < 0) nx += (long)w;
                if (src[i][rowbase[i] + (size_t)nx])
                    entry |= (1u << i);
            }
            entry |= ((uint32_t)rng_bit() << n); // RAND, dernier bit

            uint8_t out = g_lut_custom[entry];
            size_t  idx = (size_t)y * w + x;
            cam->plane0_b[idx] = out & 1;
            if (g_custom_p1_used)
                cam->plane1_b[idx] = (out >> 1) & 1;
        }
    }

    cam_swap_buffers(cam);
}

void cam_step(CAMState *cam) {
    if (g_step_mode == CAM_MODE_MARGOLUS) {
        cam_step_margolus(cam);
    } else if (g_step_mode == CAM_MODE_HEX) {
        cam_step_hex(cam);
    } else if (g_step_mode == CAM_MODE_CUSTOM) {
        cam_step_custom(cam);
    } else {
        cam_step_lut(cam);
    }
}
