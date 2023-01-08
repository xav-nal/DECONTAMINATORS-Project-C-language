/**
 * \file		288275.c 
 * \version		rendu2
 * \date		2018-04-22
 * \author		Nal et 288275
 * \brief		Implémentation du module responsable des particules
 *            
 *
 */

// *******************************************************************
// 		inclusion de fichiers en-tête avec la directives include
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h> 
#include <time.h>
#include "particule.h"

#define FIN_LISTE_PARTICULE  4
#define NB_INFO				 4
#define VRAI 				 1

enum Etat_particule{NBPARTICULE,PARTICULES,FIN_PARTICULE};
struct PARTICULE
{
    int nombre;
    double energie;
	C2D particule;
    PARTICULE *suivant;
};

//*** Déclaration des fonctions propres a ce module
int particule_lecture(char *tab, int nb_particule,int *p_ligne, int mode);

void particule_insertion(double e, double r, double x, double y);

int particule_particule_collision(double x, double y, double r,int num, int mode);

void particule_suppression();

void particule_separation(int num_part);


//*** Variables globales (du module)
static int nb_particule = VALEUR_INITIALE;

static int compteur_particule = VALEUR_INITIALE;

static int numero_particule = CORRECTION;

static PARTICULE *liste_particule = NULL;

static double sum_ini = VALEUR_INITIALE;

static double sum_sup = VALEUR_INITIALE;


//*** Fonctions exportées
int particule_decodage(char *tab,int *p_ligne, int mode, int *p_etat)
{
	int erreur = VALEUR_INITIALE;
	
	switch(*p_etat)
	{
		case NBPARTICULE:
			sscanf(tab,"%d", &nb_particule);
			if(nb_particule <= VALEUR_INITIALE)
			{
				error_invalid_nb_particules();
				erreur = ERROR;
				if(mode == ERROR) exit(EXIT_FAILURE);
				else return erreur;
			}
			*p_etat = *p_etat + CORRECTION;
			break;
				
		case PARTICULES:
			erreur = particule_lecture(tab, nb_particule,p_ligne,mode);
			if(erreur == ERROR) return erreur;
			if(nb_particule == VALEUR_INITIALE) *p_etat = *p_etat + CORRECTION;
			break;
		
		case FIN_PARTICULE:
			*p_etat = FIN_LISTE_PARTICULE;
			if(compteur_particule < nb_particule)
			{
				error_fin_liste_particules(*p_ligne);
				erreur = ERROR;
				if(mode == ERROR) exit(EXIT_FAILURE);
				else return erreur;
			}
			break;
	}
	return erreur;
}

void particule_afficher_liste_particule(FILE *file)
{
	PARTICULE *actuel = liste_particule;
    while (actuel != NULL)
    {
		fprintf(file,"#particule %d : \n",actuel->nombre);
		fprintf(file, " %lf %lf %lf %lf\n", actuel->energie, 
										actuel->particule.rayon,
										actuel->particule.centre.x,
										actuel->particule.centre.y);
        actuel = actuel->suivant;
    }
}

void particule_display()
{
	PARTICULE *actuel = liste_particule;
	int i = VALEUR_INITIALE;
	
    while (actuel != NULL)
    {						 
		particule_draw_particule(actuel->particule.centre.x,
								 actuel->particule.centre.y,
								 actuel->particule.rayon);
        actuel = actuel->suivant;
        i++;
    }
}

void particule_vider_liste()
{
	while(liste_particule != NULL)
		particule_suppression();	

	//initialisation des parametres
	nb_particule = VALEUR_INITIALE;
    compteur_particule = VALEUR_INITIALE;
    numero_particule = 1;
    sum_ini = VALEUR_INITIALE;
    sum_sup = VALEUR_INITIALE;
}

void particule_save(FILE * file)
{
	fprintf(file,"%d\n",nb_particule);
	particule_afficher_liste_particule(file);
	fprintf(file,"FIN_LISTE\n");
}

void particule_draw_particule (double x, double y, double rayon)
{
	graphic_set_color3f(0.12,0.88,0.545);
	graphic_draw_circle(x,y,rayon,FILLED);
}
 
void particule_get_info(PARTICULE *part,int *n, double *e, C2D *p, PARTICULE ***p_next)
{
    *n = part->nombre;
    *e = part->energie;
    *p = part->particule;
    **p_next = part->suivant;
}

PARTICULE* particule_get_liste_part()	
{
	return liste_particule;
}

void particule_decomposition_aleatoire()
{
	PARTICULE *actuel = liste_particule;
	double p;
	srand(time(NULL));
	
	while (actuel != NULL)
	{
		p = ( rand()/(double)RAND_MAX );
		 
		if(p <= DECOMPOSITION_RATE) particule_separation(actuel->nombre);
			
		actuel = actuel->suivant;
	}
}

C2D particule_position(int num_part)
{
	PARTICULE *actuel = liste_particule;
	int i = VALEUR_INITIALE;
	
    while (i != num_part)
    {
		actuel = actuel->suivant;
		i++;
	}
	
	return actuel->particule; 
}

int particule_nb_particule()	
{
	return nb_particule;
}

void particule_nettoyage(S2D part_asupprimer)
{
	int i = VALEUR_INITIALE;
	PARTICULE *aSupprimer = NULL;
	PARTICULE *part_avant = NULL;
	
	aSupprimer = liste_particule;
	part_avant = aSupprimer;
	
	while(aSupprimer->particule.centre.x != part_asupprimer.x || 
		  aSupprimer->particule.centre.y != part_asupprimer.y)
	{	
		part_avant = aSupprimer;
		aSupprimer = aSupprimer->suivant;	
		i++;
	}
	
	if(i != VALEUR_INITIALE)
	{
		part_avant->suivant = aSupprimer->suivant;
		sum_sup += aSupprimer->energie;
		free(aSupprimer);
		nb_particule --;
	}
	
	if(i == VALEUR_INITIALE)
	{
        liste_particule = liste_particule->suivant;
        sum_sup += aSupprimer->energie;
        free(aSupprimer);
        nb_particule--;
	}
}

double particule_rate()
{
	double r = VALEUR_INITIALE;
	//r = 100*(double)(sum_sup/sum_ini);
	r = SIM_FIN*(sum_sup/sum_ini);
	return r;
}

//Fonctions propres à ce module
int particule_lecture(char *tab, int nb_particule,int *p_ligne, int mode)
{
	int total = VALEUR_INITIALE;
	int count = VALEUR_INITIALE;
	int i = VALEUR_INITIALE;
	int erreur = VALEUR_INITIALE;
	double e, r, x, y;
	
	while(sscanf(tab+total, "%lf %lf %lf %lf%n", &e, &r, &x, &y, &count) == NB_INFO )
	{
		if( e > E_PARTICULE_MAX || r > R_PARTICULE_MAX || r < R_PARTICULE_MIN 
			|| x < -DMAX || x > DMAX || y < -DMAX || y > DMAX)
		{
			error_invalid_particule_value(e, r, x, y);
			erreur = ERROR;
			if(mode == ERROR) exit(EXIT_FAILURE);
			else return erreur;
		}
		
		compteur_particule++;
		
		if(compteur_particule > nb_particule)
		{
			error_missing_fin_liste_particules(*p_ligne);
			erreur = ERROR;
			if(mode == ERROR) exit(EXIT_FAILURE);
			else return erreur;
		}
		
		erreur = particule_particule_collision(x, y, r, numero_particule, mode);
		
		if(erreur == ERROR)
		{
			return erreur;
		}
		
		particule_insertion( e, r, x, y);
		
		sum_ini += e;
		total += count;
		numero_particule++;
		i++;
	}
	return erreur;
}

void particule_insertion(double e, double r, double x, double y)
{
    int j = VALEUR_INITIALE;
    
    PARTICULE *actuel = liste_particule;
    PARTICULE *precedent = liste_particule;
    PARTICULE *derniereParticule = liste_particule;
    PARTICULE * nouveau = (PARTICULE *)malloc(sizeof(PARTICULE));
    
    if (nouveau == NULL )
    {
        printf("Problème d'allocation dans insertion particule\n");
        exit(EXIT_FAILURE);
    }
    while (actuel != NULL)
    {
        j++;
        precedent = actuel;
        actuel = actuel->suivant;

        if(actuel != NULL) derniereParticule = actuel;
    }

	if(liste_particule != NULL)	j =  precedent->nombre;

    nouveau->suivant = NULL;

    nouveau->nombre = j + CORRECTION;
    nouveau->energie = e;
    nouveau->particule.rayon = r;
    nouveau->particule.centre.x = x;
    nouveau->particule.centre.y = y;
    
    if(j > VALEUR_INITIALE) derniereParticule->suivant = nouveau;	
    else liste_particule = nouveau; 
}

int particule_particule_collision(double x, double y, double r, int num_pt1, 
								   int mode)
{
	PARTICULE *actuel = liste_particule;
	int collision = VALEUR_INITIALE;
	int erreur = VALEUR_INITIALE;
	int num_pt2 = CORRECTION;
	double distance = VALEUR_INITIALE;
	C2D part;
	
	part.rayon = r;
	part.centre.x = x;
	part.centre.y = y;

    while (actuel != NULL)
    {
		distance = util_distance(part.centre,actuel->particule.centre);
		collision = util_collision_cercle(part, actuel->particule, &distance); 
        
        if(collision == VRAI)
		{
			error_collision(PARTICULE_PARTICULE,num_pt2 ,num_pt1);
			erreur = ERROR;
			if(mode == ERROR) exit(EXIT_FAILURE);
			else return erreur;
		}
		actuel = actuel->suivant;
		num_pt2++;
    }
    return erreur;
}

void particule_suppression()
{
	PARTICULE *aSupprimer = NULL;
	
    if (liste_particule == NULL) exit(EXIT_FAILURE);

    if (liste_particule != NULL)
    {
        aSupprimer = liste_particule;
        liste_particule = liste_particule->suivant;
        free(aSupprimer);
    }
}

void particule_separation(int num_part)
{
	int j = VALEUR_INITIALE;
	PARTICULE *actuel = liste_particule;
	PARTICULE *precedent = actuel;
	double e_part = VALEUR_INITIALE, r_part = VALEUR_INITIALE;
	double x = VALEUR_INITIALE, y = VALEUR_INITIALE;
	
	while(actuel->nombre != num_part && actuel != NULL)
	{
		precedent = actuel;
		actuel = actuel->suivant;
		j++;
	}
	
	e_part = actuel->energie * E_PARTICULE_FACTOR;
	r_part = actuel->particule.rayon * R_PARTICULE_FACTOR;
	
	if (r_part >= R_PARTICULE_MIN)
	{
		x = actuel->particule.centre.x;
		y = actuel->particule.centre.y;
		
		particule_insertion( e_part, r_part, x - r_part, y + r_part);
		particule_insertion( e_part, r_part, x + r_part, y + r_part);
		particule_insertion( e_part, r_part, x - r_part, y - r_part);
		particule_insertion( e_part, r_part, x + r_part, y - r_part);
		
		if(j > 0)
		{
			precedent->suivant = actuel->suivant;
			free(actuel);
			nb_particule += 3;
		}
		if(j == 0)
		{
			liste_particule = liste_particule->suivant;
			free(actuel);
			nb_particule += 3;
		}
	}
}

