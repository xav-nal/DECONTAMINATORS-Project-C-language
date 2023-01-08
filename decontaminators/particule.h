/**
 * \file		simulation.h 
 * \version		Rendu2
 * \date		2018-04-14
 * \author		Nal et 288275
 * \brief		Programme pour le projet du cours CS-111(c)
 *            
 *
 */
 
#ifndef PARTICULE_H
#define PARTICULE_H

#include <stdio.h>
#include "error.h"
#include "utilitaire.h"

typedef struct PARTICULE PARTICULE;


int particule_decodage(char *tab,int *p_ligne, int mode, int *p_etat);

void particule_afficher_liste_particule(FILE *file);

void particule_display();

void particule_vider_liste();

void particule_save(FILE *file);

void particule_draw_particule(double x, double y, double rayon);

void particule_get_info(PARTICULE *part,int *n, double *e, C2D *p, 
					    PARTICULE ***p_next);

PARTICULE* particule_get_liste_part();

void particule_decomposition_aleatoire();

C2D particule_position(int num_part);

int particule_nb_particule();

void particule_nettoyage(S2D part_asupprimer);

double particule_rate();
#endif
