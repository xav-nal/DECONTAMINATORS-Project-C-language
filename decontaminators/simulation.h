/**
 * \file		simulation.h 
 * \version		Rendu2
 * \date		2018-04-22
 * \author		Nal et 288275
 * \brief		Header du module responsable de la simulation          
 *
 */

#ifndef SIMULATION_H
#define SIMULATION_H

#include "error.h"
#include "utilitaire.h"
#include "particule.h"
#include "robot.h"
int simulation_lecture(const char * nom_fichier, int mode);

void simulation_display(); 

void simulation_liberation_memoire();

void simulation_save(FILE * file);

void simulation_update();

//Fonction pour tester si on a cliqué sur un robot renvoyant vrai ou faux
int simulation_mouse_robot(double x, double y,int nrb,int rob_man);

double simulation_set_rob_vit(int nrb, int val, double delta, int init);

void simulation_put_rob_mode(int nrb, int mode);

double simulation_rate();

#endif
