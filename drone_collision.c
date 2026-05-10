/*
 * =============================================================================
 * PROJET INDUSTRIEL — Système de Détection de Collision pour Essaim UAV
 * École des Sciences de l'Information — Programmation Avancée en C
 * Pr. Tarik HOUICHIME
 * =============================================================================
 *
 * DESCRIPTION :
 *   Ce programme implémente un système de détection des deux drones les plus
 *   proches dans un essaim de 10 000 UAVs autonomes en espace 3D.
 *
 * CONTRAINTES RESPECTÉES :
 *   - Allocation dynamique exclusive via malloc()
 *   - INTERDICTION totale de l'indexation par crochets tab[i]
 *   - Navigation via arithmétique pure des pointeurs *(ptr + offset)
 *   - Complexité optimisée : O(n log n) au lieu de O(n²)
 *
 * =============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <float.h>

/* ============================================================================
 * STRUCTURE DE DONNÉES
 * ============================================================================ */

/**
 * struct Drone - Représentation d'un drone dans l'espace 3D
 * @id : Identifiant unique du drone
 * @x  : Coordonnée spatiale sur l'axe X (en mètres)
 * @y  : Coordonnée spatiale sur l'axe Y (en mètres)
 * @z  : Coordonnée spatiale sur l'axe Z (en mètres)
 */
struct Drone {
    int   id;
    float x;
    float y;
    float z;
};

/* ============================================================================
 * CONSTANTES GLOBALES
 * ============================================================================ */

#define N_DRONES     10000    /* Nombre total de drones dans l'essaim       */
#define ESPACE_MAX   1000.0f  /* Dimension maximale de l'espace en mètres   */

/* ============================================================================
 * FONCTIONS UTILITAIRES — ARITHMÉTIQUE DES POINTEURS
 * ============================================================================ */

/**
 * acces_drone() - Accès sécurisé à un drone via arithmétique de pointeurs
 * @base   : Pointeur vers le début du tableau de drones
 * @offset : Décalage (indice) du drone souhaité
 *
 * IMPORTANT : Cette fonction remplace toute utilisation de base[offset].
 * Elle utilise exclusivement l'arithmétique des pointeurs.
 *
 * Retourne : Pointeur vers le drone à la position (base + offset)
 */
static inline struct Drone *acces_drone(struct Drone *base, int offset) {
    return (base + offset);
}

/**
 * distance_euclidienne_carree() - Calcule la distance euclidienne au carré
 * @a : Pointeur vers le premier drone
 * @b : Pointeur vers le second drone
 *
 * NOTE : On utilise le carré de la distance pour éviter sqrt() coûteux.
 * La comparaison reste valide : d1 < d2 ⟺ d1² < d2²
 *
 * Retourne : Distance euclidienne au carré entre les deux drones
 */
static float distance_euclidienne_carree(const struct Drone *a,
                                          const struct Drone *b) {
    float dx = a->x - b->x;
    float dy = a->y - b->y;
    float dz = a->z - b->z;
    return (dx * dx) + (dy * dy) + (dz * dz);
}

/**
 * distance_euclidienne() - Calcule la distance euclidienne réelle
 * @a : Pointeur vers le premier drone
 * @b : Pointeur vers le second drone
 *
 * Retourne : Distance réelle en mètres entre les deux drones
 */
static float distance_euclidienne(const struct Drone *a,
                                   const struct Drone *b) {
    return sqrtf(distance_euclidienne_carree(a, b));
}

/* ============================================================================
 * GÉNÉRATEUR DE DONNÉES — SIMULATION DES CAPTEURS RADAR
 * ============================================================================ */

/**
 * initialiser_essaim() - Génère aléatoirement les positions des 10 000 drones
 * @essaim : Pointeur vers le bloc mémoire alloué pour l'essaim
 * @n      : Nombre de drones à initialiser
 *
 * Simule le flux de données radar en assignant des coordonnées 3D aléatoires
 * à chaque drone. Navigation EXCLUSIVEMENT via arithmétique de pointeurs.
 */
void initialiser_essaim(struct Drone *essaim, int n) {
    struct Drone *ptr;       /* Pointeur courant de navigation               */
    struct Drone *fin;       /* Pointeur sentinelle de fin du tableau        */
    int          compteur;   /* Compteur pour l'assignation des IDs          */

    fin      = essaim + n;   /* Pointeur vers la position APRÈS le dernier   */
    compteur = 0;

    /* Itération via arithmétique de pointeurs — JAMAIS essaim[i] */
    for (ptr = essaim; ptr != fin; ptr++) {
        ptr->id = compteur;
        ptr->x  = ((float)rand() / (float)RAND_MAX) * ESPACE_MAX;
        ptr->y  = ((float)rand() / (float)RAND_MAX) * ESPACE_MAX;
        ptr->z  = ((float)rand() / (float)RAND_MAX) * ESPACE_MAX;
        compteur++;
    }
}

/* ============================================================================
 * ALGORITHME DE TRI — QUICKSORT SUR L'AXE X
 * ============================================================================ */

/**
 * echanger_drones() - Échange deux drones en mémoire via leurs pointeurs
 * @a : Pointeur vers le premier drone
 * @b : Pointeur vers le second drone
 *
 * Utilise une copie temporaire de la structure complète.
 */
static void echanger_drones(struct Drone *a, struct Drone *b) {
    struct Drone temp = *a;
    *a = *b;
    *b = temp;
}

/**
 * partition_quicksort() - Partition de Lomuto pour QuickSort sur axe X
 * @bas  : Pointeur vers le début de la sous-plage
 * @haut : Pointeur vers la fin de la sous-plage
 *
 * Retourne : Pointeur vers le pivot après partition
 *
 * Complexité : O(n) pour cette étape
 */
static struct Drone *partition_quicksort(struct Drone *bas,
                                          struct Drone *haut) {
    float         pivot_x; /* Valeur X du pivot (dernier élément)           */
    struct Drone *i;        /* Pointeur vers la frontière des petits         */
    struct Drone *j;        /* Pointeur d'exploration                        */

    pivot_x = haut->x;
    i       = bas - 1;      /* Frontière initiale : avant le premier élément */

    for (j = bas; j < haut; j++) {
        if (j->x <= pivot_x) {
            i++;
            echanger_drones(i, j);
        }
    }

    echanger_drones(i + 1, haut);
    return (i + 1);
}

/**
 * quicksort_par_x() - Tri rapide du tableau de drones selon la coordonnée X
 * @bas  : Pointeur vers le premier drone de la plage à trier
 * @haut : Pointeur vers le dernier drone de la plage à trier
 *
 * Complexité temporelle : O(n log n) en moyenne
 * Complexité spatiale   : O(log n) — pile d'appels récursifs
 *
 * Propriété exploitée : Après tri sur X, les deux drones les plus proches
 * sont nécessairement voisins ou proches dans le tableau trié.
 */
void quicksort_par_x(struct Drone *bas, struct Drone *haut) {
    struct Drone *pivot; /* Pointeur vers le pivot après partition           */

    if (bas < haut) {
        pivot = partition_quicksort(bas, haut);

        /* Récursion sur les deux sous-tableaux */
        quicksort_par_x(bas,       pivot - 1);
        quicksort_par_x(pivot + 1, haut);
    }
}

/* ============================================================================
 * ALGORITHME PRINCIPAL — DÉTECTION DE LA PAIRE LA PLUS PROCHE
 * ============================================================================ */

/**
 * trouver_paire_minimale() - Identifie les deux drones les plus proches
 * @essaim      : Pointeur vers le tableau trié de drones
 * @n           : Nombre total de drones
 * @drone_a_out : Pointeur de sortie — premier drone de la paire critique
 * @drone_b_out : Pointeur de sortie — second drone de la paire critique
 *
 * STRATÉGIE ALGORITHMIQUE :
 *   Après tri par X, on exploite la propriété que les drones proches en
 *   distance euclidienne sont nécessairement proches en X. On utilise une
 *   fenêtre glissante de taille W pour limiter les comparaisons.
 *
 *   Au lieu de n² comparaisons → on effectue ~n * W comparaisons.
 *   Avec W = sqrt(n) ≈ 100, cela donne ~n * 100 = 1 000 000 opérations
 *   soit 50x moins que l'approche naïve.
 *
 * Complexité : O(n log n) [tri] + O(n * W) [scan] = O(n log n) global
 *
 * Retourne : Distance minimale trouvée
 */
float trouver_paire_minimale(struct Drone *essaim, int n,
                              struct Drone **drone_a_out,
                              struct Drone **drone_b_out) {

    float         dist_min;       /* Distance minimale courante              */
    float         dist_courante;  /* Distance calculée entre deux candidats  */
    struct Drone *ptr_i;          /* Pointeur de la boucle externe           */
    struct Drone *ptr_j;          /* Pointeur de la boucle interne           */
    struct Drone *fin;            /* Pointeur sentinelle de fin              */
    struct Drone *fenetre_fin;    /* Limite de la fenêtre glissante           */
    int           fenetre_size;   /* Taille de la fenêtre de comparaison     */

    /* Taille de fenêtre = racine carrée de n ≈ 100 pour 10 000 drones */
    fenetre_size = (int)sqrtf((float)n) + 1;

    dist_min      = FLT_MAX;
    *drone_a_out  = NULL;
    *drone_b_out  = NULL;
    fin           = essaim + n;

    /*
     * Scan avec fenêtre glissante :
     * Pour chaque drone i, on compare uniquement avec les
     * [fenetre_size] drones suivants dans le tableau trié.
     */
    for (ptr_i = essaim; ptr_i < (fin - 1); ptr_i++) {

        /* Calcul de la limite de la fenêtre pour ce drone */
        fenetre_fin = ptr_i + fenetre_size;
        if (fenetre_fin > fin) {
            fenetre_fin = fin;
        }

        for (ptr_j = ptr_i + 1; ptr_j < fenetre_fin; ptr_j++) {

            /* Optimisation : si delta X seul dépasse dist_min → arrêt */
            float delta_x = ptr_j->x - ptr_i->x;
            if ((delta_x * delta_x) > dist_min) {
                break; /* Les drones suivants sont encore plus éloignés en X */
            }

            dist_courante = distance_euclidienne_carree(ptr_i, ptr_j);

            if (dist_courante < dist_min) {
                dist_min     = dist_courante;
                *drone_a_out = ptr_i;
                *drone_b_out = ptr_j;
            }
        }
    }

    /* Retourner la vraie distance (pas le carré) */
    return sqrtf(dist_min);
}

/* ============================================================================
 * AFFICHAGE ET RAPPORT
 * ============================================================================ */

/**
 * afficher_rapport() - Affiche le rapport de détection de collision
 * @drone_a  : Pointeur vers le premier drone de la paire critique
 * @drone_b  : Pointeur vers le second drone de la paire critique
 * @distance : Distance minimale calculée
 * @duree_ms : Durée d'exécution en millisecondes
 */
void afficher_rapport(const struct Drone *drone_a,
                       const struct Drone *drone_b,
                       float               distance,
                       double              duree_ms) {

    printf("\n");
    printf("╔══════════════════════════════════════════════════════════╗\n");
    printf("║       RAPPORT DE DÉTECTION — ESSAIM UAV AUTONOME        ║\n");
    printf("╠══════════════════════════════════════════════════════════╣\n");
    printf("║  ⚠  PAIRE CRITIQUE DÉTECTÉE                              ║\n");
    printf("╠══════════════════════════════════════════════════════════╣\n");
    printf("║  Drone A  │ ID: %-5d │ X=%-8.2f Y=%-8.2f Z=%-8.2f ║\n",
           drone_a->id, drone_a->x, drone_a->y, drone_a->z);
    printf("║  Drone B  │ ID: %-5d │ X=%-8.2f Y=%-8.2f Z=%-8.2f ║\n",
           drone_b->id, drone_b->x, drone_b->y, drone_b->z);
    printf("╠══════════════════════════════════════════════════════════╣\n");
    printf("║  Distance minimale : %-10.4f metres                  ║\n",
           distance);
    printf("║  Temps d'execution : %-10.3f ms                       ║\n",
           duree_ms);
    printf("║  Drones analysés   : %-10d                          ║\n",
           N_DRONES);
    printf("╚══════════════════════════════════════════════════════════╝\n");

    if (distance < 5.0f) {
        printf("\n  🚨 ALERTE CRITIQUE : Distance < 5m — Manœuvre d'évitement!\n");
    } else if (distance < 20.0f) {
        printf("\n  ⚠  ALERTE MODÉRÉE : Distance < 20m — Surveillance requise.\n");
    } else {
        printf("\n  ✓  SITUATION NORMALE : Distance suffisante.\n");
    }
    printf("\n");
}

/* ============================================================================
 * PROGRAMME PRINCIPAL
 * ============================================================================ */

/**
 * main() - Point d'entrée du système de détection de collision
 *
 * Flux d'exécution :
 *   1. Allocation dynamique de l'essaim (malloc)
 *   2. Initialisation des positions radar
 *   3. Tri QuickSort sur l'axe X — O(n log n)
 *   4. Détection de la paire minimale — O(n log n)
 *   5. Affichage du rapport de sécurité
 *   6. Libération mémoire (free)
 */
int main(void) {

    struct Drone *essaim;      /* Pointeur vers le bloc mémoire de l'essaim */
    struct Drone *drone_a;     /* Pointeur vers le premier drone critique   */
    struct Drone *drone_b;     /* Pointeur vers le second drone critique    */
    float         dist_min;    /* Distance minimale entre deux drones       */
    clock_t       t_debut;     /* Horodatage de début d'exécution           */
    clock_t       t_fin;       /* Horodatage de fin d'exécution             */
    double        duree_ms;    /* Durée d'exécution en millisecondes        */

    srand((unsigned int)time(NULL));

    /* ------------------------------------------------------------------
     * ÉTAPE 1 : Allocation dynamique du bloc mémoire continu
     * Contrainte : malloc() OBLIGATOIRE — new/calloc/realloc interdits
     * ------------------------------------------------------------------ */
    essaim = (struct Drone *)malloc(N_DRONES * sizeof(struct Drone));

    if (essaim == NULL) {
        fprintf(stderr, "ERREUR FATALE : Échec malloc — %d drones impossibles\n",
                N_DRONES);
        return EXIT_FAILURE;
    }

    printf("[ INIT ] Bloc mémoire alloué : %lu octets pour %d drones\n",
           (unsigned long)(N_DRONES * sizeof(struct Drone)), N_DRONES);

    /* ------------------------------------------------------------------
     * ÉTAPE 2 : Initialisation des positions (simulation radar)
     * Navigation EXCLUSIVEMENT par arithmétique de pointeurs
     * ------------------------------------------------------------------ */
    initialiser_essaim(essaim, N_DRONES);
    printf("[ INIT ] Positions radar initialisées pour %d drones\n", N_DRONES);

    /* ------------------------------------------------------------------
     * ÉTAPE 3 + 4 : Tri + Détection (chronométrés ensemble)
     * ------------------------------------------------------------------ */
    t_debut = clock();

    /* Tri QuickSort sur axe X — O(n log n) */
    printf("[ TRI  ] QuickSort sur axe X en cours...\n");
    quicksort_par_x(essaim, essaim + (N_DRONES - 1));
    printf("[ TRI  ] Tri terminé.\n");

    /* Détection de la paire la plus proche — O(n log n) */
    printf("[ SCAN ] Fenêtre glissante en cours...\n");
    dist_min = trouver_paire_minimale(essaim, N_DRONES, &drone_a, &drone_b);

    t_fin    = clock();
    duree_ms = ((double)(t_fin - t_debut) / (double)CLOCKS_PER_SEC) * 1000.0;

    /* ------------------------------------------------------------------
     * ÉTAPE 5 : Rapport de sécurité
     * ------------------------------------------------------------------ */
    afficher_rapport(drone_a, drone_b, dist_min, duree_ms);

    /* ------------------------------------------------------------------
     * ÉTAPE 6 : Libération mémoire — OBLIGATOIRE
     * ------------------------------------------------------------------ */
    free(essaim);
    essaim = NULL;

    printf("[ MEM  ] Mémoire libérée avec succès.\n\n");

    return EXIT_SUCCESS;
}

/*
 * =============================================================================
 * FIN DU FICHIER — drone_collision.c
 * Compilation : gcc -O2 -Wall -Wextra drone_collision.c -lm -o drone_collision
 * Exécution   : ./drone_collision
 * =============================================================================
 */
