/**
 * \file		robot.h 
 * \version		Rendu2
 * \date		2018-04-22
 * \author		Nal et 288275
 * \brief		Header du module responsable des robots          
 *
 */

#ifndef ROBOT_H
#define ROBOT_H

#include <stdio.h>
#include "error.h"
#include "utilitaire.h"
#include "particule.h"

int robot_decodage(char *tab,int *p_ligne, int mode, int *p_etat);

int robot_particule_collision(int mode);

void robot_liberation_memoire();


//Fonctions pour la simulation
int robot_update();

//Donne les coordonnées d'une particule comme objectif à un robot
void robot_coordination(int i);  

void robot_deplacement_update(int num_rob);


// Fonctions pour le callback graphique
int robot_mouse_robot(double x, double y, int nrb, int rob_man);

void robot_put_rob_mode(int nrb, int mode);

double robot_set_vit(int nrb, int val,double delta, int init);

void robot_display();

void robot_save(FILE *file);

#endif 
