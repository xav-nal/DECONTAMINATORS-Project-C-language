/**
 * \file		robot.h 
 * \version		Rendu2
 * \date		2018-04-22
 * \author		Nal et 288275
 * \brief		Interface de définition des constantes globales au projet         
 *
 */
 
#ifndef CONSTANTES_H
#define CONSTANTES_H

#include "tolerance.h"

#define DELTA_T					0.25
#define VTRAN_MAX				0.75
#define VROT_MAX				0.5
#define DELTA_VROT				0.125
#define DELTA_VTRAN				0.25
#define DMAX 					20
#define R_ROBOT					0.5
#define R_PARTICULE_MAX			4
#define R_PARTICULE_MIN			0.3
#define R_PARTICULE_FACTOR		0.4142
#define E_PARTICULE_MAX 		1
#define E_PARTICULE_FACTOR		0.25
#define DECOMPOSITION_RATE		0.025
#define MAX_LINE 				120

#define SUPPRESSION_ROBOTS 	    3
#define SUPPRESSION_PARTICULES  3
#define VALEUR_INITIALE         0
#define ERROR              	    1
#define NOT_ERROR               0
#define DRAW				    0
#define FAUX                    0
#define VRAI 				    1
#define ACTIF				    1
#define INACTIF				    0
#define MANUEL				    0
#define AUTO				    1
#define	VROT				    1		
#define VTRAN 					0
#define CORRECTION				1
#define HALF				    2
#define SIM_FIN			 		100.000000

//valeur du temps au maximum mis par un robot pour aller à une particule dans le monde
#define T_MAX 				   	342
#endif


