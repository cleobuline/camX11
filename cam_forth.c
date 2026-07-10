#include "cam_forth.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

void forth_init(ForthVM *vm) {
    memset(vm, 0, sizeof(ForthVM));
    vm->sp = 0;
    vm->dict_size = 0;
    vm->neighborhood = FORTH_NEIGHBORHOOD_MOORE;
}

void forth_push(ForthVM *vm, int32_t val) {
    if (vm->sp < FORTH_STACK_SIZE) {
        vm->stack[vm->sp++] = val;
    }
}

int32_t forth_pop(ForthVM *vm) {
    if (vm->sp > 0) {
        return vm->stack[--vm->sp];
    }
    return 0;
}

// N/HEX (chapitre 16, FHP) : grille pseudo-hexagonale par decalage en
// quinconce. Le plan 0 est lu comme d'habitude (rangees paires et
// impaires alignees dans la memoire), mais les 6 voisins hexagonaux se
// lisent a des offsets DIFFERENTS selon la parite de la ligne (row_odd) :
// sur une ligne paire, les voisins diagonaux NE/NW/SE/SW pointent vers
// la ligne du dessus/dessous a la MEME colonne et la colonne+1 (a l'est) ;
// sur une ligne impaire, vers la colonne et la colonne-1 (a l'ouest).
// C'est l'astuce classique du "brickwork" : une ligne sur deux est
// visuellement decalee d'une demi-cellule.
// L'appelant (cam_core.c) calcule les 6 bits ligne par ligne selon cette
// regle puis assemble l'entree ; ce decodeur n'assigne que les champs
// de la VM a partir d'une entree deja assemblee (bit0=CENTER,
// bits 1-6 = E,NE,NW,W,SW,SE, bit7=RAND). row_odd n'est ici qu'a titre
// documentaire pour les appelants qui voudraient l'utiliser eux-memes.
void forth_decode_entry_hex(ForthVM *vm, uint32_t entry, int row_odd) {
    (void)row_odd;
    vm->center = entry & 1;
    vm->hex_e  = (entry >> 1) & 1;
    vm->hex_ne = (entry >> 2) & 1;
    vm->hex_nw = (entry >> 3) & 1;
    vm->hex_w  = (entry >> 4) & 1;
    vm->hex_sw = (entry >> 5) & 1;
    vm->hex_se = (entry >> 6) & 1;
    vm->rnd    = (entry >> 7) & 1;
}

// N/CUSTOM (chapitre 7, voisinages arbitraires) : bit i de l'entree =
// i-eme voisin declare, bit custom_count = RAND. Les autres champs de
// voisinage sont remis a zero -- une regle custom qui lirait NORTH par
// habitude verrait toujours 0, plutot qu'un residu de la LUT precedente.
// Confort : si (0,0,0) et/ou (0,0,1) font partie de la declaration,
// CENTER et CENTER' sont cables sur ces bits-la.
void forth_decode_entry_custom(ForthVM *vm, uint32_t entry) {
    vm->center = vm->center_prime = 0;
    vm->north = vm->south = vm->east = vm->west = 0;
    vm->neast = vm->nwest = vm->seast = vm->swest = 0;
    vm->north_p = vm->south_p = vm->east_p = vm->west_p = 0;

    for (int i = 0; i < vm->custom_count; i++) {
        uint8_t bit = (entry >> i) & 1;
        vm->custom[i] = bit;
        if (vm->custom_def[i].dx == 0 && vm->custom_def[i].dy == 0) {
            if (vm->custom_def[i].plane == 0) vm->center       = bit;
            else                              vm->center_prime = bit;
        }
    }
    vm->rnd = (entry >> vm->custom_count) & 1;
}

// N/VONN, tableau 7.2 : les diagonales du plan 0 sont remplacées par les
// voisins von Neumann du plan 1.
//   bit0=CENTER bit1=CENTER' bit2=EAST' bit3=WEST' bit4=SOUTH' bit5=NORTH'
//   bit6=EAST bit7=WEST bit8=SOUTH bit9=NORTH
void forth_decode_entry_vonn(ForthVM *vm, uint32_t entry) {
    vm->center       = (entry >> 0) & 1;
    vm->center_prime = (entry >> 1) & 1;
    vm->east_p       = (entry >> 2) & 1;
    vm->west_p       = (entry >> 3) & 1;
    vm->south_p      = (entry >> 4) & 1;
    vm->north_p      = (entry >> 5) & 1;
    vm->east         = (entry >> 6) & 1;
    vm->west         = (entry >> 7) & 1;
    vm->south        = (entry >> 8) & 1;
    vm->north        = (entry >> 9) & 1;
    // pas de diagonales en N/VONN
    vm->neast = vm->nwest = vm->seast = vm->swest = 0;
}

// Décodage Margolus complet : CENTER/CW/CCW/OPP depuis le bloc du plan 0,
// CENTER'/CW'/CCW'/OPP' depuis celui du plan 1, plus les pseudo-voisins.
// HORZ/VERT : la cellule (x,y) d'un bloc à la phase p a x = 2bx + p + cx,
// donc x&1 = p ^ cx (idem verticalement) — les parités spatiales se
// déduisent du coin et de la phase.
void forth_decode_margolus_full(ForthVM *vm, uint8_t p0_nibble,
                                 uint8_t p1_nibble, uint8_t phase,
                                 uint8_t phase_prime, uint8_t probe_p1_nibble,
                                 ForthCorner corner) {
    uint8_t nw0 = (p0_nibble >> 0) & 1, ne0 = (p0_nibble >> 1) & 1;
    uint8_t sw0 = (p0_nibble >> 2) & 1, se0 = (p0_nibble >> 3) & 1;
    uint8_t nw1 = (p1_nibble >> 0) & 1, ne1 = (p1_nibble >> 1) & 1;
    uint8_t sw1 = (p1_nibble >> 2) & 1, se1 = (p1_nibble >> 3) & 1;

    // &CENTER' est UNIFORME PAR BLOC (le bit NW du nibble de sonde, tout
    // le temps), PAS par coin. Sans ca, chaque coin verrait une valeur
    // DIFFERENTE de la sonde (position absolue differente sur le plan
    // croise), et les 4 coins choisiraient CW/CCW independamment plutot
    // qu'en rotation coordonnee du bloc entier -- ce qui brise la
    // conservation du nombre de particules (une rotation est une
    // permutation, garantie conservative ; 4 choix independants ne le
    // sont pas). Meme principe que RAND, deja partage par tout le bloc.
    // Decouvert le 6/7 : DENDRITE-NOISE fuyait des particules du plan 0
    // meme SANS aucun mur peint, exactement a cause de cette variation
    // par coin.
    uint8_t probe = (probe_p1_nibble >> 0) & 1;

    uint8_t cx, cy;
    switch (corner) {
        case FORTH_CORNER_NW:
            vm->center = nw0; vm->cw = ne0; vm->ccw = sw0; vm->opp = se0;
            vm->center_prime = nw1; vm->cw_p = ne1; vm->ccw_p = sw1; vm->opp_p = se1;
            cx = 0; cy = 0;
            break;
        case FORTH_CORNER_NE:
            vm->center = ne0; vm->cw = se0; vm->ccw = nw0; vm->opp = sw0;
            vm->center_prime = ne1; vm->cw_p = se1; vm->ccw_p = nw1; vm->opp_p = sw1;
            cx = 1; cy = 0;
            break;
        case FORTH_CORNER_SE:
            vm->center = se0; vm->cw = sw0; vm->ccw = ne0; vm->opp = nw0;
            vm->center_prime = se1; vm->cw_p = sw1; vm->ccw_p = ne1; vm->opp_p = nw1;
            cx = 1; cy = 1;
            break;
        case FORTH_CORNER_SW:
        default:
            vm->center = sw0; vm->cw = nw0; vm->ccw = se0; vm->opp = ne0;
            vm->center_prime = sw1; vm->cw_p = nw1; vm->ccw_p = se1; vm->opp_p = ne1;
            cx = 0; cy = 1;
            break;
    }
    vm->amp_center_prime = probe; // identique pour les 4 coins, voir ci-dessus

    vm->phase       = phase & 1;
    vm->phase_prime = phase_prime & 1;
    // NB : HORZ/VERT supposent que l'origine de la partition suit PHASE
    // (comme dans ALT-GRID-PH) ; un cycle qui les découple fausserait
    // ces deux pseudo-voisins.
    vm->horz = (uint8_t)((phase ^ cx) & 1);
    vm->vert = (uint8_t)((phase ^ cy) & 1);
}

// Assigne CENTER/CW/CCW/OPP selon le coin occupé par la cellule courante
// dans son bloc 2x2, en parcourant les coins dans le sens horaire
// NW -> NE -> SE -> SW -> NW (cf. livre, §12.5) :
//   CW  = voisin suivant dans ce sens
//   CCW = voisin précédent
//   OPP = coin diagonalement opposé
void forth_decode_margolus_corner(ForthVM *vm, uint8_t nw, uint8_t ne,
                                   uint8_t sw, uint8_t se, ForthCorner corner) {
    switch (corner) {
        case FORTH_CORNER_NW:
            vm->center = nw; vm->cw = ne; vm->ccw = sw; vm->opp = se;
            break;
        case FORTH_CORNER_NE:
            vm->center = ne; vm->cw = se; vm->ccw = nw; vm->opp = sw;
            break;
        case FORTH_CORNER_SE:
            vm->center = se; vm->cw = sw; vm->ccw = ne; vm->opp = nw;
            break;
        case FORTH_CORNER_SW:
        default:
            vm->center = sw; vm->cw = nw; vm->ccw = se; vm->opp = ne;
            break;
    }
}

void forth_decode_entry(ForthVM *vm, uint32_t entry) {
    vm->center        = (entry >> 0) & 1;
    vm->center_prime  = (entry >> 1) & 1;
    vm->north         = (entry >> 2) & 1;
    vm->south         = (entry >> 3) & 1;
    vm->east          = (entry >> 4) & 1;
    vm->west          = (entry >> 5) & 1;
    vm->neast         = (entry >> 6) & 1;
    vm->nwest         = (entry >> 7) & 1;
    vm->seast         = (entry >> 8) & 1;
    vm->swest         = (entry >> 9) & 1;
}

static int str_eq(const char *a, const char *b) {
    return strcmp(a, b) == 0;
}

static int tokenize(const char *rule, char tokens[FORTH_MAX_TOKENS][32]) {
    char buf[4096];
    strncpy(buf, rule, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    // Commentaire-ligne du Forth standard : un '\' ISOLE (entoure de
    // blancs) neutralise tout jusqu'a la fin de la ligne. Efface avant
    // la tokenisation, qui perd les fins de ligne (strtok_r coupe sur
    // '\n'). Le test "isole" est indispensable : sans lui, un futur mot
    // contenant une contre-oblique disparaitrait avec sa ligne.
    // Les commentaires ( ... ) restent geres en aval, inchanges.
    for (char *p = buf; *p; p++) {
        if (*p != '\\') continue;
        int at_start = (p == buf) || (p[-1] == ' ' || p[-1] == '\t' || p[-1] == '\n');
        int isolated = (p[1] == '\0' || p[1] == ' ' || p[1] == '\t' || p[1] == '\n');
        if (!at_start || !isolated) continue;
        while (*p && *p != '\n') *p++ = ' ';
        if (!*p) break;
    }

    int count = 0;
    char *saveptr = NULL;
    char *token = strtok_r(buf, " \t\n", &saveptr);
    while (token != NULL && count < FORTH_MAX_TOKENS) {
        strncpy(tokens[count], token, 31);
        tokens[count][31] = '\0';
        count++;
        token = strtok_r(NULL, " \t\n", &saveptr);
    }
    return count;
}

static int find_matching_then(char tokens[FORTH_MAX_TOKENS][32], int count, int from) {
    int depth = 1;
    for (int i = from; i < count; i++) {
        if (str_eq(tokens[i], "IF")) depth++;
        else if (str_eq(tokens[i], "THEN")) {
            depth--;
            if (depth == 0) return i;
        }
    }
    return -1;
}

static int find_matching_else(char tokens[FORTH_MAX_TOKENS][32], int count, int from, int then_idx) {
    int depth = 1;
    for (int i = from; i < then_idx; i++) {
        if (str_eq(tokens[i], "IF")) depth++;
        else if (str_eq(tokens[i], "THEN")) depth--;
        else if (str_eq(tokens[i], "ELSE") && depth == 1) return i;
    }
    return -1;
}

static const char *dict_lookup(ForthVM *vm, const char *name) {
    for (int i = 0; i < vm->dict_size; i++) {
        if (str_eq(vm->dict[i].name, name)) {
            return vm->dict[i].body;
        }
    }
    return NULL;
}

// Comme dict_lookup, mais donne acces au ForthWord entier -- en
// particulier a ses tokens deja mis en cache (voir ForthWord dans
// cam_forth.h), pour eviter de retokeniser le corps a chaque appel.
static ForthWord *dict_lookup_word(ForthVM *vm, const char *name) {
    for (int i = 0; i < vm->dict_size; i++) {
        if (str_eq(vm->dict[i].name, name)) {
            return &vm->dict[i];
        }
    }
    return NULL;
}

// déclarations forward
static void exec_tokens(ForthVM *vm, char tokens[FORTH_MAX_TOKENS][32], int start, int end);
static int32_t forth_eval_inner(ForthVM *vm, const char *rule);

static void exec_tokens(ForthVM *vm, char tokens[FORTH_MAX_TOKENS][32], int start, int end) {
    int i = start;
    while (i < end) {
        const char *token = tokens[i];

        // --- commentaire Forth ( ... ) — ignoré, comme dans le livre
        // (ex: les commentaires de pile "( -- flt)") ---
        if (token[0] == '(') {
            while (i < end) {
                size_t len = strlen(tokens[i]);
                int closes = (len > 0 && tokens[i][len - 1] == ')');
                i++;
                if (closes) break;
            }
            continue;
        }

        // --- contrôle de flux IF/ELSE/THEN ---
        if (str_eq(token, "IF")) {
            int then_idx = find_matching_then(tokens, end, i + 1);
            int else_idx = find_matching_else(tokens, end, i + 1, then_idx);

            int32_t cond = forth_pop(vm);

            if (cond != 0) {
                int block_end = (else_idx != -1) ? else_idx : then_idx;
                exec_tokens(vm, tokens, i + 1, block_end);
            } else if (else_idx != -1) {
                exec_tokens(vm, tokens, else_idx + 1, then_idx);
            }
            i = then_idx + 1;
            continue;
        }

        // --- table inline { e0 e1 ... eN } ---
        // Conforme au Forth du bouquin : les entrées peuvent être des
        // littéraux OU des mots (U, OPP, 2PART, ...). L'index est dépilé,
        // puis SEULE l'entrée sélectionnée est évaluée — c'est elle qui
        // laisse le résultat sur la pile. Permet d'écrire, tel quel :
        //   CENTER CW CCW OPP + + +  { U OPP 2PART U U }
        if (str_eq(token, "{")) {
            int table_start = i + 1;
            int table_end = table_start;
            while (table_end < end && !str_eq(tokens[table_end], "}")) {
                table_end++;
            }
            int table_size = table_end - table_start;

            int32_t idx = forth_pop(vm);
            if (idx >= 0 && idx < table_size) {
                exec_tokens(vm, tokens, table_start + idx, table_start + idx + 1);
            } else {
                forth_push(vm, 0);
            }
            i = table_end + 1; // saute le '}'
            continue;
        }

        // --- mot utilisateur (dictionnaire) ---
        {
            ForthWord *w = dict_lookup_word(vm, token);
            if (w) {
                exec_tokens(vm, w->tokens, 0, w->token_count);
                i++;
                continue;
            }
        }

        // --- neighbor words ---
        if      (str_eq(token, "CENTER"))   forth_push(vm, vm->center);
        else if (str_eq(token, "CENTER'"))  forth_push(vm, vm->center_prime);
        else if (str_eq(token, "U")) {
            forth_push(vm, vm->center);
        }
        else if (str_eq(token, "NORTH"))    forth_push(vm, vm->north);
        else if (str_eq(token, "SOUTH"))    forth_push(vm, vm->south);
        else if (str_eq(token, "EAST"))     forth_push(vm, vm->east);
        else if (str_eq(token, "WEST"))     forth_push(vm, vm->west);
        else if (str_eq(token, "N.EAST"))   forth_push(vm, vm->neast);
        else if (str_eq(token, "N.WEST"))   forth_push(vm, vm->nwest);
        else if (str_eq(token, "S.EAST"))   forth_push(vm, vm->seast);
        else if (str_eq(token, "S.WEST"))   forth_push(vm, vm->swest);

        // --- voisins primés de N/VONN (plan 1, tableau 7.2) ---
        else if (str_eq(token, "NORTH'"))   forth_push(vm, vm->north_p);
        else if (str_eq(token, "SOUTH'"))   forth_push(vm, vm->south_p);
        else if (str_eq(token, "EAST'"))    forth_push(vm, vm->east_p);
        else if (str_eq(token, "WEST'"))    forth_push(vm, vm->west_p);

        // --- voisins hexagonaux (N/HEX, chapitre 16) ---
        else if (str_eq(token, "HEX-E"))    forth_push(vm, vm->hex_e);
        else if (str_eq(token, "HEX-NE"))   forth_push(vm, vm->hex_ne);
        else if (str_eq(token, "HEX-NW"))   forth_push(vm, vm->hex_nw);
        else if (str_eq(token, "HEX-W"))    forth_push(vm, vm->hex_w);
        else if (str_eq(token, "HEX-SW"))   forth_push(vm, vm->hex_sw);
        else if (str_eq(token, "HEX-SE"))   forth_push(vm, vm->hex_se);

        // --- voisins declares par N/CUSTOM : NBR0..NBR11, dans l'ordre
        // de declaration. Un mot utilisateur peut evidemment les
        // baptiser ( : NORD NBR1 ; ) -- le dictionnaire existant s'en
        // charge sans rien de plus.
        else if (strncmp(token, "NBR", 3) == 0 && token[3] != '\0') {
            char *endp;
            long n = strtol(token + 3, &endp, 10);
            if (*endp == '\0' && n >= 0 && n < FORTH_CUSTOM_MAX) {
                if ((int)n >= vm->custom_count) {
                    fprintf(stderr, "CAM-FORTH: %s — seulement %d voisin(s) "
                            "déclaré(s) par N/CUSTOM ; vaut 0.\n",
                            token, vm->custom_count);
                }
                forth_push(vm, ((int)n < vm->custom_count) ? vm->custom[n] : 0);
            } else {
                fprintf(stderr, "CAM-FORTH: mot inconnu '%s' (NBR0..NBR%d)\n",
                        token, FORTH_CUSTOM_MAX - 1);
                forth_push(vm, 0);
            }
        }

        // --- voisins de Margolus (bloc 2x2, §12.5) ---
        else if (str_eq(token, "CW"))       forth_push(vm, vm->cw);
        else if (str_eq(token, "CCW"))      forth_push(vm, vm->ccw);
        else if (str_eq(token, "OPP"))      forth_push(vm, vm->opp);

        // --- voisins primés de Margolus (bloc du plan 1) ---
        else if (str_eq(token, "CW'"))      forth_push(vm, vm->cw_p);
        else if (str_eq(token, "CCW'"))     forth_push(vm, vm->ccw_p);
        else if (str_eq(token, "OPP'"))     forth_push(vm, vm->opp_p);

        // --- pseudo-voisin bruyant (chapitre 8) ---
        else if (str_eq(token, "RAND"))     forth_push(vm, vm->rnd);

        // --- pseudo-voisins de phase (N/MARG-PH, N/MARG-HV) ---
        else if (str_eq(token, "PHASE"))    forth_push(vm, vm->phase);
        else if (str_eq(token, "HORZ"))     forth_push(vm, vm->horz);
        else if (str_eq(token, "VERT"))     forth_push(vm, vm->vert);
        else if (str_eq(token, "HV")) {
            forth_push(vm, vm->horz | (vm->vert << 1));
        }
        else if (str_eq(token, "PHASE'")) forth_push(vm, vm->phase_prime);
        else if (str_eq(token, "PHASES")) {
            forth_push(vm, vm->phase | (vm->phase_prime << 1));
        }

        // --- collectifs primé/non-primé (§7.4) ---
        else if (str_eq(token, "CENTERS")) {
            forth_push(vm, vm->center | (vm->center_prime << 1));
        }
        else if (str_eq(token, "CWS")) {
            forth_push(vm, vm->cw | (vm->cw_p << 1));
        }
        else if (str_eq(token, "CCWS")) {
            forth_push(vm, vm->ccw | (vm->ccw_p << 1));
        }
        else if (str_eq(token, "OPPS")) {
            forth_push(vm, vm->opp | (vm->opp_p << 1));
        }

        // --- sondes vers l'autre demi-machine (chapitre 9) ---
        else if (str_eq(token, "&CENTER"))   forth_push(vm, vm->amp_center);
        else if (str_eq(token, "&CENTER'"))  forth_push(vm, vm->amp_center_prime);
        else if (str_eq(token, "&CENTERS")) {
            forth_push(vm, vm->amp_center | (vm->amp_center_prime << 1));
        }
        // alias de lecture selon la source déclarée (§7.3.2) : mêmes fils
        else if (str_eq(token, "&PHASE"))   forth_push(vm, vm->amp_center);
        else if (str_eq(token, "&PHASE'"))  forth_push(vm, vm->amp_center_prime);
        else if (str_eq(token, "&PHASES")) {
            forth_push(vm, vm->amp_center | (vm->amp_center_prime << 1));
        }
        else if (str_eq(token, "&HORZ"))    forth_push(vm, vm->amp_center);
        else if (str_eq(token, "&VERT"))    forth_push(vm, vm->amp_center_prime);
        else if (str_eq(token, "&HV")) {
            forth_push(vm, vm->amp_center | (vm->amp_center_prime << 1));
        }

        // --- autres voisinages mineurs &... non câblés (ex. &PHASE) ---
        else if (token[0] == '&') {
            static int warned_camb_probe = 0;
            if (!warned_camb_probe) {
                fprintf(stderr, "CAM-FORTH: %s — sonde mineure non câblée, "
                                "vaut toujours 0.\n", token);
                warned_camb_probe = 1;
            }
            forth_push(vm, 0);
        }

        // --- opérateurs logiques ---
        else if (str_eq(token, "XOR")) {
            int32_t b = forth_pop(vm), a = forth_pop(vm);
            forth_push(vm, a ^ b);
        }
        else if (str_eq(token, "AND")) {
            int32_t b = forth_pop(vm), a = forth_pop(vm);
            forth_push(vm, a & b);
        }
        else if (str_eq(token, "OR")) {
            int32_t b = forth_pop(vm), a = forth_pop(vm);
            forth_push(vm, a | b);
        }
        else if (str_eq(token, "NOT")) {
            int32_t a = forth_pop(vm);
            forth_push(vm, (a == 0) ? 1 : 0);
        }

        // --- arithmétique ---
        else if (str_eq(token, "2*")) {
            forth_push(vm, forth_pop(vm) << 1);
        }
        else if (str_eq(token, "2/")) {
            forth_push(vm, forth_pop(vm) >> 1);
        }
        else if (str_eq(token, "+")) {
            int32_t b = forth_pop(vm), a = forth_pop(vm);
            forth_push(vm, a + b);
        }
        else if (str_eq(token, "-")) {
            int32_t b = forth_pop(vm), a = forth_pop(vm);
            forth_push(vm, a - b);
        }

        // --- stack manipulation ---
        else if (str_eq(token, "DUP")) {
            int32_t a = forth_pop(vm);
            forth_push(vm, a);
            forth_push(vm, a);
        }
        else if (str_eq(token, "DROP")) {
            forth_pop(vm);
        }
        else if (str_eq(token, "SWAP")) {
            int32_t b = forth_pop(vm), a = forth_pop(vm);
            forth_push(vm, b);
            forth_push(vm, a);
        }
        else if (str_eq(token, "OVER")) {
            int32_t b = forth_pop(vm), a = forth_pop(vm);
            forth_push(vm, a);
            forth_push(vm, b);
            forth_push(vm, a);
        }

        // --- comparaisons ---
        else if (str_eq(token, "=")) {
            int32_t b = forth_pop(vm), a = forth_pop(vm);
            forth_push(vm, (a == b) ? 1 : 0);
        }
        else if (str_eq(token, "<>")) {
            int32_t b = forth_pop(vm), a = forth_pop(vm);
            forth_push(vm, (a != b) ? 1 : 0);
        }
        else if (str_eq(token, ">")) {
            int32_t b = forth_pop(vm), a = forth_pop(vm);
            forth_push(vm, (a > b) ? 1 : 0);
        }
        else if (str_eq(token, "<")) {
            int32_t b = forth_pop(vm), a = forth_pop(vm);
            forth_push(vm, (a < b) ? 1 : 0);
        }

        // --- expédition vers les plans (§5.4) : >PLNn dépile la valeur
        // et la destine au plan n. >PLNO (lettre O) = graphie OCR de >PLN0.
        else if (str_eq(token, ">PLN0") || str_eq(token, ">PLNO")) {
            vm->out_val[0] = (forth_pop(vm) != 0) ? 1 : 0;
            vm->out_mask |= 0x1;
        }
        else if (str_eq(token, ">PLN1")) {
            vm->out_val[1] = (forth_pop(vm) != 0) ? 1 : 0;
            vm->out_mask |= 0x2;
        }
        else if (str_eq(token, ">PLN2")) {
            vm->out_val[2] = (forth_pop(vm) != 0) ? 1 : 0;
            vm->out_mask |= 0x4;
        }
        else if (str_eq(token, ">PLN3")) {
            vm->out_val[3] = (forth_pop(vm) != 0) ? 1 : 0;
            vm->out_mask |= 0x8;
        }
        // >PLNA / >PLNB : expédie une valeur 2 bits vers la PAIRE de plans
        // de la demi-machine (A: plans 0-1, B: plans 2-3). Utilisé par les
        // compteurs du chapitre 9 (ex. le chrono des TUBE-WORMS).
        else if (str_eq(token, ">PLNA")) {
            int32_t v = forth_pop(vm);
            vm->out_val[0] = v & 1;
            vm->out_val[1] = (v >> 1) & 1;
            vm->out_mask |= 0x3;
        }
        else if (str_eq(token, ">PLNB")) {
            int32_t v = forth_pop(vm);
            vm->out_val[2] = v & 1;
            vm->out_val[3] = (v >> 1) & 1;
            vm->out_mask |= 0xC;
        }

        // --- nombre littéral ---
        else {
            char *endptr;
            long val = strtol(token, &endptr, 10);
            if (*endptr == '\0') {
                forth_push(vm, (int32_t)val);
            } else {
                fprintf(stderr, "CAM-FORTH: mot inconnu '%s'\n", token);
            }
        }

        i++;
    }
}

static int32_t forth_eval_inner(ForthVM *vm, const char *rule) {
    char tokens[FORTH_MAX_TOKENS][32];
    int count = tokenize(rule, tokens);
    exec_tokens(vm, tokens, 0, count);
    return forth_pop(vm);
}

int32_t forth_eval(ForthVM *vm, const char *rule) {
    vm->sp = 0;  // reset seulement au niveau top
    return forth_eval_inner(vm, rule);
}

// prototypes des builders (définis plus bas, utilisés par make_table)
static int build_lut_from_body(ForthVM *vm, uint8_t *lut, const char *body);

void make_table(uint8_t *lut, const char *rule) {
    ForthVM vm;
    forth_init(&vm);
    (void)build_lut_from_body(&vm, lut, rule);
}

// --- helpers de construction, partagés par MAKE-TABLE (§ dispatch) ---

// Avertissement unique si une règle expédie vers les plans 2/3 (CAM-B).
static void warn_camb_if_needed(uint8_t used_mask) {
    static int warned = 0;
    if ((used_mask & 0xC) && !warned) {
        fprintf(stderr, "CAM-FORTH: >PLN2/>PLN3 — ces plans appartiennent à "
                        "CAM-B ; en mode Margolus l'expédition est ignorée "
                        "(utilisez une section CAM-B en mode LUT).\n");
        warned = 1;
    }
}

// LUT Moore ou VONN, 2 bits par entrée : bit0 = plan 0, bit1 = plan 1.
// Plan 0 : valeur de >PLN0, ou à défaut le sommet de pile (compatibilité
// avec les règles qui se terminent par une expression nue).
// Plan 1 : valeur de >PLN1, ou à défaut l'ECHO (= CENTER), qui reproduit
// le comportement historique du moteur.
// Construit la LUT d'une demi-machine. Pour CAM-A, la sortie 2 bits est
// (plan 0, plan 1) — expédiée par >PLN0/>PLN1/>PLNA ; pour CAM-B c'est
// (plan 2, plan 3) — expédiée par >PLN2/>PLN3/>PLNB. Dans les deux cas,
// CENTER etc. désignent les plans PROPRES de la moitié, et &CENTER /
// &CENTER' les centres de l'autre moitié (bits 11-12 de l'entrée).
// Retourne le OU des out_mask rencontres pendant la construction --
// c'est ce qui permet a MAKE-TABLE de savoir si une regle N/CUSTOM a
// expedie >PLN1 (custom_p1_used), sans variable globale de liaison.
static int build_lut_from_body(ForthVM *vm, uint8_t *lut, const char *body) {
    int half = vm->half; // 0 = CAM-A, 1 = CAM-B
    int lo = half ? 2 : 0;             // slot du plan "bas" de la paire
    uint8_t lo_bit = half ? 0x4 : 0x1; // bits correspondants de out_mask
    uint8_t hi_bit = half ? 0x8 : 0x2;
    int vonn   = (vm->neighborhood == FORTH_NEIGHBORHOOD_VONN);
    int hex    = (vm->neighborhood == FORTH_NEIGHBORHOOD_HEX);
    int custom = (vm->neighborhood == FORTH_NEIGHBORHOOD_CUSTOM);
    int used_mask = 0;

    // Tokenise UNE SEULE FOIS ici : "body" ne change jamais a travers
    // les 256/1024/8192 iterations qui suivent. Avant cette correction,
    // tokenize() etait rappele a CHAQUE entree -- un gaspillage pur,
    // mesure a ~20ms par compilation sur une regle a mots imbriques ;
    // le tokeniseur re-decoupait le meme texte des milliers de fois.
    char word_tokens[FORTH_MAX_TOKENS][32];
    int word_count = tokenize(body, word_tokens);

    // N/CUSTOM (chapitre 7) : LUT de 2^(count+1) entrees -- un bit par
    // voisin declare, dans l'ordre de declaration, plus RAND sur le bit
    // count. Le reste du buffer (jusqu'a FORTH_LUT_SIZE) est remis a
    // zero pour ne pas trainer de residus d'une compilation precedente.
    // 2 bits par entree comme en Moore : bit0 = plan 0 (>PLN0 ou sommet
    // de pile), bit1 = plan 1 (>PLN1 UNIQUEMENT -- pas d'ECHO par
    // defaut ici, car CENTER n'existe que si (0,0,0) est declare ; si
    // rien n'est expedie, le moteur fait traverser le plan 1 inchange,
    // exactement comme en Margolus).
    if (custom) {
        if (vm->custom_count <= 0) {
            fprintf(stderr, "CAM-FORTH: MAKE-TABLE en N/CUSTOM sans aucun "
                            "voisin déclaré — table non construite.\n");
            return 0;
        }
        memset(lut, 0, FORTH_LUT_SIZE);
        uint32_t nentries = 1u << (vm->custom_count + 1); // + RAND
        for (uint32_t entry = 0; entry < nentries; entry++) {
            forth_decode_entry_custom(vm, entry);
            vm->sp = 0;
            vm->out_mask = 0;

            exec_tokens(vm, word_tokens, 0, word_count);

            uint8_t plo = (vm->out_mask & lo_bit) ? vm->out_val[lo]
                                                  : ((forth_pop(vm) != 0) ? 1 : 0);
            uint8_t phi = (vm->out_mask & hi_bit) ? vm->out_val[lo + 1] : 0;
            used_mask |= vm->out_mask;

            lut[entry] = plo | (phi << 1);
        }
        return used_mask;
    }

    // N/HEX (chapitre 16) : table dediee, plus petite (256 entrees,
    // pas de sondes ni de plan 1 -- le gaz FHP est mono-plan). On
    // n'ecrit que le bit0 (plan 0) ; le reste du buffer lut n'est pas
    // touche par cette branche.
    if (hex) {
        for (uint32_t entry = 0; entry < FORTH_HEX_LUT_SIZE; entry++) {
            forth_decode_entry_hex(vm, entry, 0);
            vm->sp = 0;
            vm->out_mask = 0;

            exec_tokens(vm, word_tokens, 0, word_count);

            uint8_t p0 = (vm->out_mask & 0x1) ? vm->out_val[0]
                                              : ((forth_pop(vm) != 0) ? 1 : 0);
            used_mask |= vm->out_mask;
            lut[entry] = p0;
        }
        return used_mask;
    }

    for (uint32_t entry = 0; entry < FORTH_LUT_SIZE; entry++) {
        if (vonn) forth_decode_entry_vonn(vm, entry & 0x3FF);
        else      forth_decode_entry(vm, entry & 0x3FF);
        vm->rnd              = (entry >> 10) & 1; // RAND (chapitre 8)
        vm->amp_center       = (entry >> 11) & 1; // &CENTER  (chapitre 9)
        vm->amp_center_prime = (entry >> 12) & 1; // &CENTER'
        vm->sp = 0;
        vm->out_mask = 0;

        exec_tokens(vm, word_tokens, 0, word_count);

        uint8_t plo = (vm->out_mask & lo_bit) ? vm->out_val[lo]
                                              : ((forth_pop(vm) != 0) ? 1 : 0);
        uint8_t phi = (vm->out_mask & hi_bit) ? vm->out_val[lo + 1]
                                              : vm->center; // ECHO par défaut
        used_mask |= vm->out_mask;

        lut[entry] = plo | (phi << 1);
    }
    return used_mask;
}

// Tables de bloc Margolus complètes (FORTH_MARG_TABLE_SIZE entrées) :
// index = nibble plan 0 | nibble plan 1 << 4 | PHASE << 8. Sorties :
//  - p0_table : nouveau bloc du plan 0 ;
//  - p1_table : nouveau bloc du plan 1 si la règle expédie >PLN1
//    (sinon *p1_used reste 0 et le moteur fait traverser le plan 1
//    inchangé). Si >PLN1 n'est expédié que sur certains chemins IF/ELSE,
//    les cellules non expédiées reçoivent 0.
static void build_margolus_from_body(ForthVM *vm,
                                     uint8_t *p0_table,
                                     uint8_t *p1_table,
                                     int *p1_used,
                                     const char *body) {
    static const ForthCorner corners[4] = {
        FORTH_CORNER_NW, FORTH_CORNER_NE, FORTH_CORNER_SW, FORTH_CORNER_SE
    };
    char word_tokens[FORTH_MAX_TOKENS][32];
    int word_count = tokenize(body, word_tokens);
    uint8_t used_mask = 0;

    // Sensible a la moitie, comme build_lut_from_body : en CAM-B, la
    // sortie "propre" de cette demi-machine est PLN2/PLN3 (bits 0x4/0x8
    // de out_mask), pas PLN0/PLN1. Avant ce correctif, cette fonction
    // ne regardait JAMAIS out_val[2]/[3] -- >PLN2 a l'interieur d'une
    // regle Margolus CAM-B etait donc silencieusement ignore, la table
    // retombant a zero partout (voir dendrite/CAM-B, session du 6/7).
    int half = vm->half;
    int lo = half ? 2 : 0;
    uint8_t lo_bit = half ? 0x4 : 0x1;
    uint8_t hi_bit = half ? 0x8 : 0x2;

    *p1_used = 0;

    for (uint32_t idx = 0; idx < FORTH_MARG_TABLE_SIZE; idx++) {
        uint8_t p0_nibble = idx & 0xF;
        uint8_t p1_nibble = (idx >> 4) & 0xF;
        uint8_t phase     = (idx >> 8) & 1;
        uint8_t rnd       = (idx >> 9) & 1;
        uint8_t phase_p   = (idx >> 10) & 1;
        uint8_t probe_p1  = (idx >> 11) & 0xF;
        uint8_t out_p0 = 0;
        uint8_t out_p1 = 0;

        for (int c = 0; c < 4; c++) {
            forth_decode_margolus_full(vm, p0_nibble, p1_nibble, phase,
                                        phase_p, probe_p1, corners[c]);
            vm->rnd = rnd; // bit RAND commun au bloc (chapitre 8)
            vm->sp = 0;
            vm->out_mask = 0;
            exec_tokens(vm, word_tokens, 0, word_count);

            uint8_t p0 = (vm->out_mask & lo_bit) ? vm->out_val[lo]
                                                 : ((forth_pop(vm) != 0) ? 1 : 0);
            out_p0 |= (p0 << c);

            if (vm->out_mask & hi_bit) {
                out_p1 |= (vm->out_val[lo + 1] << c);
                *p1_used = 1;
            }
            used_mask |= vm->out_mask;
        }

        p0_table[idx] = out_p0;
        p1_table[idx] = out_p1;
    }

    // L'avertissement ne concerne plus que CAM-A ecrivant a tort sur
    // PLN2/PLN3 (toujours un no-op la) -- desormais sans objet pour une
    // regle CAM-B qui ecrit legitimement sur ses propres plans.
    if (half == 0) warn_camb_if_needed(used_mask);
}

// Baptise un voisin custom : cree (ou remplace) un mot du dictionnaire
// dont le corps est "NBRn" -- le nom declare devient un mot Forth comme
// un autre, resolu par la machinerie existante. Un nom peut donc
// legitimement MASQUER un mot cable (CENTER, NORTH...) : le dictionnaire
// est consulte avant les mots internes, et c'est exactement l'effet
// recherche ("0 0 0 CENTER" redonne au CENTER habituel son sens).
// Refuse les noms de la forme NBRn : le mot aurait pour corps son
// propre nom, recursion infinie garantie a la premiere evaluation.
static void custom_register_name(ForthVM *vm, const char *name, int idx) {
    if (strncmp(name, "NBR", 3) == 0) {
        char *endp;
        (void)strtol(name + 3, &endp, 10);
        if (name[3] != '\0' && *endp == '\0') {
            fprintf(stderr, "CAM-FORTH: N/CUSTOM — '%s' est réservé "
                    "(indexation des voisins), nom ignoré.\n", name);
            return;
        }
    }

    char body[16];
    snprintf(body, sizeof(body), "NBR%d", idx);

    for (int d = 0; d < vm->dict_size; d++) {
        if (str_eq(vm->dict[d].name, name)) {
            strncpy(vm->dict[d].body, body, FORTH_BODY_MAXLEN - 1);
            vm->dict[d].token_count = tokenize(body, vm->dict[d].tokens);
            return;
        }
    }
    if (vm->dict_size < FORTH_DICT_SIZE) {
        strncpy(vm->dict[vm->dict_size].name, name, FORTH_WORD_MAXLEN - 1);
        vm->dict[vm->dict_size].name[FORTH_WORD_MAXLEN - 1] = '\0';
        strncpy(vm->dict[vm->dict_size].body, body, FORTH_BODY_MAXLEN - 1);
        vm->dict[vm->dict_size].token_count =
            tokenize(body, vm->dict[vm->dict_size].tokens);
        vm->dict_size++;
    } else {
        fprintf(stderr, "CAM-FORTH: N/CUSTOM — dictionnaire plein, "
                "nom '%s' ignoré (utilisez NBR%d).\n", name, idx);
    }
}

int forth_compile(ForthVM *vm, ForthTables *tables, const char *source) {    char tokens[FORTH_MAX_TOKENS][32];
    char buf[4096];
    strncpy(buf, source, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    int count = 0;
    char *tok = strtok(buf, " \t\n");
    while (tok != NULL && count < FORTH_MAX_TOKENS) {
        strncpy(tokens[count], tok, 31);
        tokens[count][31] = '\0';
        count++;
        tok = strtok(NULL, " \t\n");
    }

    int built = 0;
    int i = 0;

    while (i < count) {

        // --- commentaire Forth ( ... ) — ignoré partout, y compris
        // avant/entre les définitions ---
        if (tokens[i][0] == '(') {
            while (i < count) {
                size_t len = strlen(tokens[i]);
                int closes = (len > 0 && tokens[i][len - 1] == ')');
                i++;
                if (closes) break;
            }
            continue;
        }

        // --- définition : NOM ... ; ---
        if (str_eq(tokens[i], ":")) {
            i++;
            if (i >= count) break;

            char word_name[FORTH_WORD_MAXLEN];
            strncpy(word_name, tokens[i], FORTH_WORD_MAXLEN - 1);
            word_name[FORTH_WORD_MAXLEN - 1] = '\0';
            i++;

            char body[FORTH_BODY_MAXLEN];
            body[0] = '\0';
            while (i < count && !str_eq(tokens[i], ";")) {
                if (strlen(body) + strlen(tokens[i]) + 2 < FORTH_BODY_MAXLEN) {
                    if (body[0] != '\0') strcat(body, " ");
                    strcat(body, tokens[i]);
                }
                i++;
            }
            i++; // saute le ;

            // ajoute ou remplace dans le dictionnaire
            int found = 0;
            for (int d = 0; d < vm->dict_size; d++) {
                if (str_eq(vm->dict[d].name, word_name)) {
                    strncpy(vm->dict[d].body, body, FORTH_BODY_MAXLEN - 1);
                    vm->dict[d].token_count = tokenize(body, vm->dict[d].tokens);
                    found = 1;
                    break;
                }
            }
            if (!found && vm->dict_size < FORTH_DICT_SIZE) {
                strncpy(vm->dict[vm->dict_size].name, word_name, FORTH_WORD_MAXLEN - 1);
                strncpy(vm->dict[vm->dict_size].body, body, FORTH_BODY_MAXLEN - 1);
                vm->dict[vm->dict_size].token_count =
                    tokenize(body, vm->dict[vm->dict_size].tokens);
                vm->dict_size++;
            }
            continue;
        }

        // --- run-cycle (§11.5) : CYCLE ... END-CYCLE ---
        // Lignes "valeur IS <registre>", un STEP clôt chaque pas.
        // Registres : <ORG-HV> (0 = grille paire, 3 = impaire),
        // <PHASE>, <PHASE'>, <PHASES> (réglage joint 2 bits).
        if (str_eq(tokens[i], "CYCLE")) {
            i++;
            int32_t pending = 0;
            uint8_t org = 0, ph = 0, php = 0;
            int len = 0;
            while (i < count && !str_eq(tokens[i], "END-CYCLE")) {
                if (str_eq(tokens[i], "IS")) {
                    i++;
                    if (i >= count) break;
                    const char *reg = tokens[i];
                    if      (str_eq(reg, "<PHASE>"))  ph  = pending & 1;
                    else if (str_eq(reg, "<PHASE'>")) php = pending & 1;
                    else if (str_eq(reg, "<PHASES>")) {
                        ph = pending & 1; php = (pending >> 1) & 1;
                    }
                    else if (str_eq(reg, "<ORG-HV>")) {
                        if (pending == 0)      org = 0;
                        else if (pending == 3) org = 1;
                        else {
                            fprintf(stderr, "CAM-FORTH: <ORG-HV> %d — seuls "
                                    "0 et 3 (décalage diagonal) sont câblés ; "
                                    "traité comme %d.\n",
                                    pending, pending ? 1 : 0);
                            org = pending ? 1 : 0;
                        }
                    }
                    else {
                        fprintf(stderr, "CAM-FORTH: registre %s non câblé "
                                "dans le run-cycle.\n", reg);
                    }
                } else if (str_eq(tokens[i], "STEP")) {
                    if (tables && len < FORTH_CYCLE_MAX) {
                        tables->cycle[len].org         = org;
                        tables->cycle[len].phase       = ph;
                        tables->cycle[len].phase_prime = php;
                        len++;
                    }
                } else {
                    char *endp;
                    long v = strtol(tokens[i], &endp, 10);
                    if (*endp == '\0') pending = (int32_t)v;
                }
                i++;
            }
            if (tables) tables->cycle_len = len;
            i++; // saute END-CYCLE
            continue;
        }

        // --- sections de demi-machine (chapitre 9) ---
        if (str_eq(tokens[i], "CAM-A")) { vm->half = 0; i++; continue; }
        if (str_eq(tokens[i], "CAM-B")) { vm->half = 1; i++; continue; }
        // Déclarations du voisinage mineur (§7.3.2) : choisissent ce que
        // transportent les deux fils de sonde de la moitié COURANTE.
        if (str_eq(tokens[i], "&/CENTERS") || str_eq(tokens[i], "&/CENTER")) {
            if (tables) {
                if (vm->half) tables->probe_source_b = FORTH_PROBE_CENTERS;
                else          tables->probe_source_a = FORTH_PROBE_CENTERS;
            }
            i++; continue;
        }
        if (str_eq(tokens[i], "&/PHASES")) {
            if (tables) {
                if (vm->half) tables->probe_source_b = FORTH_PROBE_PHASES;
                else          tables->probe_source_a = FORTH_PROBE_PHASES;
            }
            i++; continue;
        }
        if (str_eq(tokens[i], "&/HV")) {
            if (tables) {
                if (vm->half) tables->probe_source_b = FORTH_PROBE_HV;
                else          tables->probe_source_a = FORTH_PROBE_HV;
            }
            i++; continue;
        }

        // --- déclarations de voisinage (§7.3 du livre) ---
        // N/MOORE : Moore 3x3 (défaut). N/VONN : von Neumann (voisins
        // primés du plan 1). N/MARG : Margolus, blocs 2x2. Les variantes
        // N/MARG-PH et N/MARG-HV sont acceptées : PHASE, HORZ et VERT
        // sont de toute façon disponibles en mode Margolus.
        if (str_eq(tokens[i], "N/MOORE")) {
            vm->neighborhood = FORTH_NEIGHBORHOOD_MOORE;
            i++;
            continue;
        }
        if (str_eq(tokens[i], "N/VONN")) {
            vm->neighborhood = FORTH_NEIGHBORHOOD_VONN;
            i++;
            continue;
        }
        if (str_eq(tokens[i], "N/MARG") || str_eq(tokens[i], "N/MARGOLUS") ||
            str_eq(tokens[i], "N/MARG-PH") || str_eq(tokens[i], "N/MARG-HV")) {
            vm->neighborhood = FORTH_NEIGHBORHOOD_MARGOLUS;
            i++;
            continue;
        }
        if (str_eq(tokens[i], "N/HEX")) {
            vm->neighborhood = FORTH_NEIGHBORHOOD_HEX;
            i++;
            continue;
        }

        // --- N/CUSTOM ... END-CUSTOM (chapitre 7, voisinages libres) ---
        // Le corps est une suite de "dx dy plane [NOM]" : trois nombres,
        // puis un nom OPTIONNEL qui baptise ce voisin (il devient un mot
        // du dictionnaire equivalant a NBRn -- les NBRn restent toujours
        // disponibles). Commentaires ( ... ) admis n'importe ou.
        // Convention BOUSSOLE : dx > 0 = est, dy > 0 = nord (haut de
        // l'ecran). plane 0 ou 1 : les plans propres de la demi-machine
        // courante. Jusqu'a FORTH_CUSTOM_MAX (12) voisins, comme le
        // vrai CAM-8.
        if (str_eq(tokens[i], "N/CUSTOM")) {
            i++;
            int32_t trip[3];
            int t = 0, n = 0, dropped = 0;
            int awaiting_name = 0; // un triplet valide vient d'etre clos
            while (i < count && !str_eq(tokens[i], "END-CUSTOM")) {
                if (tokens[i][0] == '(') { // commentaire au fil des triplets
                    while (i < count) {
                        size_t len = strlen(tokens[i]);
                        int closes = (len > 0 && tokens[i][len - 1] == ')');
                        i++;
                        if (closes) break;
                    }
                    continue;
                }
                char *endp;
                long v = strtol(tokens[i], &endp, 10);
                if (*endp == '\0') {
                    awaiting_name = 0; // le triplet suivant commence sans nom
                    trip[t++] = (int32_t)v;
                    if (t == 3) {
                        t = 0;
                        if (n >= FORTH_CUSTOM_MAX) { dropped++; i++; continue; }
                        if (trip[0] < -127 || trip[0] > 127 ||
                            trip[1] < -127 || trip[1] > 127) {
                            fprintf(stderr, "CAM-FORTH: N/CUSTOM — déplacement "
                                    "(%d,%d) hors limites (±127), voisin ignoré.\n",
                                    trip[0], trip[1]);
                            i++; continue;
                        }
                        if (trip[2] != 0 && trip[2] != 1) {
                            fprintf(stderr, "CAM-FORTH: N/CUSTOM — plane %d "
                                    "invalide (0 ou 1), voisin ignoré.\n", trip[2]);
                            i++; continue;
                        }
                        vm->custom_def[n].dx    = (int8_t)trip[0];
                        vm->custom_def[n].dy    = (int8_t)trip[1];
                        vm->custom_def[n].plane = (uint8_t)trip[2];
                        n++;
                        awaiting_name = 1; // un nom PEUT suivre
                    }
                } else if (awaiting_name && t == 0) {
                    // le jeton qui suit un triplet complet est son nom
                    custom_register_name(vm, tokens[i], n - 1);
                    awaiting_name = 0;
                } else {
                    fprintf(stderr, "CAM-FORTH: N/CUSTOM — jeton '%s' "
                            "inattendu %s, ignoré.\n", tokens[i],
                            t ? "au milieu d'un triplet" : "(nombres attendus)");
                }
                i++;
            }
            i++; // saute END-CUSTOM
            if (t != 0)
                fprintf(stderr, "CAM-FORTH: N/CUSTOM — triplet incomplet "
                        "en fin de déclaration, ignoré.\n");
            if (dropped)
                fprintf(stderr, "CAM-FORTH: N/CUSTOM — %d voisin(s) au-delà "
                        "de la limite de %d, ignoré(s).\n", dropped, FORTH_CUSTOM_MAX);
            vm->custom_count = n;
            vm->neighborhood = FORTH_NEIGHBORHOOD_CUSTOM;
            continue;
        }

        // --- MAKE-TABLE NOM ---
        // Construit la table du voisinage COURANT, comme dans le vrai
        // CAM Forth : N/MOORE → LUT 10 bits, N/MARG → table de bloc 2x2.
        // MAKE-TABLE-MARGOLUS reste accepté comme raccourci historique
        // (équivaut à N/MARG puis MAKE-TABLE, sans changer la déclaration
        // courante pour la suite de la source).
        if (str_eq(tokens[i], "MAKE-TABLE") || str_eq(tokens[i], "MAKE-TABLE-MARGOLUS")) {
            int force_margolus = str_eq(tokens[i], "MAKE-TABLE-MARGOLUS");
            i++;
            if (i >= count) break;

            const char *body = dict_lookup(vm, tokens[i]);
            if (!body) {
                fprintf(stderr, "CAM-FORTH: MAKE-TABLE - mot '%s' inconnu\n", tokens[i]);
                i++;
                continue;
            }

            int margolus = force_margolus ||
                           (vm->neighborhood == FORTH_NEIGHBORHOOD_MARGOLUS);

            if (margolus && tables) {
                if (vm->half == 1) {
                    // CAM-B a sa PROPRE table Margolus, symetrique de
                    // celle de CAM-A -- fidele a la vraie machine des
                    // annees 80, ou les deux modules etaient identiques.
                    build_margolus_from_body(vm, tables->margolus_p0_b,
                                              tables->margolus_p1_b,
                                              &tables->margolus_p1_used_b, body);
                    built |= FORTH_BUILT_MARGOLUS_B;
                } else {
                    build_margolus_from_body(vm, tables->margolus_p0,
                                              tables->margolus_p1,
                                              &tables->margolus_p1_used, body);
                    built |= FORTH_BUILT_MARGOLUS;
                }
            } else if (!margolus && tables) {
                if (vm->half == 1) {
                    if (vm->neighborhood == FORTH_NEIGHBORHOOD_CUSTOM) {
                        // Le moteur ne sait pas (encore) echantillonner
                        // des offsets libres sur les plans 2-3 : plutot
                        // qu'une table silencieusement fausse, on refuse
                        // franchement -- meme politique que >PLN2/>PLN3
                        // en Margolus a l'epoque.
                        fprintf(stderr, "CAM-FORTH: N/CUSTOM n'est pas câblé "
                                "pour CAM-B pour l'instant — table ignorée.\n");
                    } else {
                        (void)build_lut_from_body(vm, tables->lut_b, body);
                        tables->lut_b_neighborhood = vm->neighborhood;
                        built |= FORTH_BUILT_LUT_B;
                    }
                } else {
                    int used = build_lut_from_body(vm, tables->lut, body);
                    tables->lut_neighborhood = vm->neighborhood;
                    if (vm->neighborhood == FORTH_NEIGHBORHOOD_CUSTOM) {
                        if (vm->custom_count <= 0) { i++; continue; } // rien construit
                        memcpy(tables->custom_nbrs, vm->custom_def,
                               sizeof(vm->custom_def));
                        tables->custom_count   = vm->custom_count;
                        tables->custom_p1_used = (used & 0x2) ? 1 : 0;
                    }
                    built |= FORTH_BUILT_LUT;
                }
            }
            i++;
            continue;
        }
        i++;
    }

    return built;
}
