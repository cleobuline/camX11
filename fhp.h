//
//  fhp.h
//  cam-8 — chapitre 16 : gaz de Frisch-Hasslacher-Pomeau
//
//  Sous-systeme ENTIEREMENT ISOLE du reste du moteur (cam_core.c,
//  cam_forth.c). Aucun fichier existant n'est modifie pour l'integrer :
//  FHPState est une structure a part, avec son propre create/destroy/step.
//  L'integration dans l'app (si souhaitee) se fait en ADDITION pure,
//  jamais en modification des chemins CAM-A/CAM-B/Margolus/HEX-mono-plan
//  qui fonctionnent deja.
//
//  Pourquoi un sous-systeme a part et pas une extension de CAMState ?
//  Le vrai gaz FHP a besoin de 6 bits par cellule (un par direction a
//  60 degres) pour encoder une particule EN MOUVEMENT — un CA a un seul
//  bit par cellule (comme N/HEX ou CAM-A/CAM-B) ne peut representer que
//  de la densite/diffusion, jamais du transport avec quantite de
//  mouvement (voir le test de conservation qui a fait echouer la
//  premiere tentative de regle FHP-GAS en Forth).
//

#ifndef CAM8_FHP_H
#define CAM8_FHP_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// Les 6 directions a 60 degres, memes conventions que N/HEX dans
// cam_forth.h (encodage even-r : E, NE, NW, W, SW, SE).
typedef enum {
    FHP_DIR_E  = 0,
    FHP_DIR_NE = 1,
    FHP_DIR_NW = 2,
    FHP_DIR_W  = 3,
    FHP_DIR_SW = 4,
    FHP_DIR_SE = 5
} FHPDirection;

// direction opposee : E<->W, NE<->SW, NW<->SE
static inline FHPDirection fhp_opposite(FHPDirection d) {
    return (FHPDirection)((d + 3) % 6);
}

typedef struct {
    uint32_t width;
    uint32_t height;

    // 6 plans booleens, un par direction, double-tamponnes.
    uint8_t *dir_a[6];
    uint8_t *dir_b[6];

    // Obstacle (mur, cylindre...) : UN plan, non double-tamponne — le
    // decor ne bouge pas. Les particules qui l'atteignent rebondissent
    // (bounce-back complet, la condition aux limites classique pour
    // simuler un solide immobile dans le gaz).
    uint8_t *obstacle;

    // Graine du generateur de nombres pseudo-aleatoires DEDIEE a ce
    // sous-systeme (pas celle de cam_core.c) : deux mondes independants.
    uint32_t rng_state;

    // Bords ouverts (chapitre 16, soufflerie) : quand actif, tout ce qui
    // atteint le bord DROIT (x = width-1) disparaît a la transport au
    // lieu de reapparaitre a gauche par le tore. Indispensable pour une
    // vraie scene "vent + obstacle" : sans ca, le gaz appauvri par
    // l'obstacle finit par recirculer et boucler sur lui-meme, ce qui
    // ressemble a une bande stagnante bien plus qu'a un sillage.
    // Desactive par defaut : le comportement torique d'origine (utile
    // pour les bancs d'essai d'isotropie) n'est pas remis en cause.
    int open_right_edge;

    // FHP-II (particules au repos, viscosite reglable) : un 7e canal,
    // PAS booleen -- il compte 0, 1 ou 2 particules au repos par
    // cellule. Deux, pas une seule : c'est le minimum qui permette
    // d'echanger une PAIRE frontale (quantite de mouvement nette nulle,
    // E+W ou NE+SW ou NW+SE) contre des particules au repos SANS jamais
    // violer la conservation du nombre de particules ni de la quantite
    // de mouvement. Double-tamponne comme les 6 canaux directionnels,
    // meme si le repos ne "transporte" jamais (vitesse nulle) -- la
    // phase de transport se contente de le recopier tel quel.
    uint8_t *rest_a;
    uint8_t *rest_b;

    // p_rest_convert : probabilite (0-100) qu'une collision frontale a
    // 2 corps soit ABSORBEE en 2 particules au repos plutot que devier
    // vers l'axe transverse habituel (et reciproquement, qu'une paire
    // au repos soit EMISE vers un axe choisi au hasard). C'est LE bouton
    // de viscosite qui manquait a FHP-I : plus haut, plus le gaz
    // "colle", plus la traversee d'un obstacle laisse un vrai sillage.
    // A 0 (defaut), le comportement est identique bit-pour-bit a FHP-I
    // -- cette extension ne change RIEN a ce qui a deja ete valide ce
    // soir tant qu'on n'y touche pas explicitement.
    int p_rest_convert;
} FHPState;

FHPState *fhp_create(uint32_t width, uint32_t height);
void      fhp_destroy(FHPState *fhp);

// Vide tout (gaz + obstacles).
void fhp_clear(FHPState *fhp);

// Remplit une direction avec une densite aleatoire (0-100).
void fhp_seed_random(FHPState *fhp, FHPDirection dir, int density_percent);

// Pose/retire un obstacle ponctuel (mur, cylindre construit cellule
// par cellule depuis l'app ou un test).
void fhp_set_obstacle(FHPState *fhp, uint32_t x, uint32_t y, int present);

// Un pas complet : collision puis transport. Void inline.
void fhp_step(FHPState *fhp);

// Accesseurs pratiques pour l'affichage / les tests :
//  - densite totale (nombre de bits allumes, toutes directions confondues)
//  - densite locale a une cellule (0-6)
int      fhp_total_population(const FHPState *fhp);
uint8_t  fhp_local_density(const FHPState *fhp, uint32_t x, uint32_t y);
uint8_t  fhp_local_rest(const FHPState *fhp, uint32_t x, uint32_t y); // 0, 1 ou 2


// --- Vue hydrodynamique (chapitre 16) ---------------------------------
// Le gaz FHP est BRUYANT : a l'equilibre, la densite d'une cellule
// fluctue de +/-1 particule autour de sa moyenne, alors qu'une onde
// acoustique ne la fait varier que de quelques dixiemes. Affichee
// cellule par cellule, l'onde est litteralement noyee (rapport
// signal/bruit mesure : 0.38). L'hydrodynamique du FHP n'existe qu'APRES
// moyennage spatial -- c'est le coarse-graining, et ce n'est pas un
// artifice d'affichage mais le passage de la mecanique statistique a la
// mecanique des fluides.
//
// fhp_coarse_field remplit `out` (width*height flottants) avec la densite
// moyenne sur un carre (2r+1)x(2r+1) centre sur chaque cellule. Calcul en
// O(N) par table de sommes cumulees (summed-area table), independamment
// de r : un rayon de 6 ne coute pas plus cher qu'un rayon de 1, ce qui
// permet de le regler en continu pendant que la simulation tourne.
//
// Mesure : r=0 -> S/B 0.38 (invisible) ; r=2 -> 1.39 ; r=4 -> 2.54 ;
//          r=6 -> 3.32 (le front se detache nettement).
//
// Retourne la densite moyenne globale (la "densite d'equilibre" rho0),
// dont l'appelant a besoin comme point neutre de la palette divergente.
double fhp_coarse_field(const FHPState *fhp, float *out, int radius);

#endif
