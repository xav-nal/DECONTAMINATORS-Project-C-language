/**
 * \file		simulation.c 
 * \version		Rendu2
 * \date		2018-04-22
 * \author		Nal et Leafe
 * \brief		Implémentation du module responsable de la simulation
 */
 
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "simulation.h"

#define FIN_LISTE_ROBOT      3
#define FIN_LISTE_PARTICULE  4
#define TAB_MAX				 80
#define SIZE_CARRE			 2*DMAX
#define IDENTIQUE            0

enum Type{ROBOTS_LECTURE, PARTICULES_LECTURE};
static int s_etat = VALEUR_INITIALE;
static int type = ROBOTS_LECTURE;
static int  erreur = VALEUR_INITIALE;

//declarations des fonctions propre à ce module
void simulation_decodage_ligne(char * tab, int *p_ligne, int mode);

void simulation_world_display();

//******Fonctions exportées
int  simulation_lecture(const char * nom_fichier, int mode)
{
	char tab[TAB_MAX] ;
	char chaine[] = "FIN_LISTE\n";
	char *position_diese = NULL; 
	
	s_etat = VALEUR_INITIALE;
	
	FILE *fichier = NULL;
	int ligne = VALEUR_INITIALE;
	int i = VALEUR_INITIALE;
	
	for(i = VALEUR_INITIALE; i < TAB_MAX; i++) tab[i] = VALEUR_INITIALE;
	
	if((fichier = fopen (nom_fichier, "r")) != NULL) 
	{   
		while(fgets(tab, TAB_MAX, fichier))
		{
			ligne++;
			if((tab[VALEUR_INITIALE] =='\n')||(tab[VALEUR_INITIALE] =='\r')) 
				continue;	
			
			position_diese = strpbrk(tab, "0123456789#F");
			
			if(position_diese == NULL) continue;
			
			if(*position_diese != '#') position_diese = NULL;
				
			if (position_diese != NULL) continue;

			if (strcmp(tab, chaine) == IDENTIQUE) s_etat++;
				
			simulation_decodage_ligne(tab, &ligne, mode);
			
			for(i = VALEUR_INITIALE; i < TAB_MAX; i++)tab[i] = VALEUR_INITIALE;
			
			if(erreur == ERROR) return erreur;
		}
		if (erreur == NOT_ERROR)
			error_no_error_in_this_file();
	}
	else
	{
		error_file_missing(nom_fichier);
		erreur = ERROR;
		if(mode==ERROR) exit(EXIT_FAILURE);
	}
	return erreur;
}

void simulation_display()
{
	simulation_world_display();
	robot_display();
	particule_display();
}

void simulation_liberation_memoire()
{
	robot_liberation_memoire();
	particule_vider_liste();
	erreur = VALEUR_INITIALE;
	s_etat = VALEUR_INITIALE;
	type = ROBOTS_LECTURE;
}

void simulation_save(FILE* file)
{
	robot_save(file);
	particule_save(file);
}

void simulation_update()
{
	particule_decomposition_aleatoire(); 

	robot_update();
}	

int simulation_mouse_robot(double x, double y,int nrb, int rob_man)
{
	int c = VALEUR_INITIALE;
	c = robot_mouse_robot(x, y, nrb, rob_man);
	return c;
}

double simulation_set_rob_vit(int nrb, int val,double delta, int init)
{
	double v = VALEUR_INITIALE;
	v = robot_set_vit(nrb, val, delta, init);
	return v;
}

void simulation_put_rob_mode(int nrb,int mode)
{
	robot_put_rob_mode(nrb,mode);
}

double simulation_rate()
{
	double r = VALEUR_INITIALE;
	r = particule_rate();
	return r;
}


//fonctions propre à simulation
void simulation_decodage_ligne(char *tab, int *p_ligne, int mode)
{
	switch(type)
	{
		case ROBOTS_LECTURE:
			erreur = robot_decodage(tab,p_ligne, mode, &s_etat);
			break;
		
		case PARTICULES_LECTURE:
			erreur = particule_decodage(tab,p_ligne, mode, &s_etat);
			break;
	}
	
	if(s_etat == FIN_LISTE_ROBOT)
	{
		type = PARTICULES_LECTURE;
		s_etat = VALEUR_INITIALE;
	}
	
	if(s_etat == FIN_LISTE_PARTICULE)
	{
		erreur = robot_particule_collision(mode);
		type = ROBOTS_LECTURE;
		s_etat = VALEUR_INITIALE;
	}
}

void simulation_world_display()
{
	graphic_set_color3f(VALEUR_INITIALE,VALEUR_INITIALE,VALEUR_INITIALE);
	graphic_draw_rectangle(VALEUR_INITIALE,VALEUR_INITIALE,SIZE_CARRE,EMPTY);
}

