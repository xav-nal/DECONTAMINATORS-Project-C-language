/**
 * \file		robot.h 
 * \version		Rendu2
 * \date		2018-04-22
 * \author		Nal et 288275
 * \brief		Header du module responsable du graphique
 */

#ifndef GRAPHIC_H
#define GRAPHIC_H

#include "constantes.h"

#define FILLED   1
#define EMPTY 	 0

void 	   graphic_draw_rectangle (float xc, float yc, float side, int filled);

void       graphic_draw_segment (double x1, double y1, double x2, double y2);

void       graphic_draw_circle (double xc, double yc, double rayon, int filled);

void       graphic_set_color3f(float r, float g, float b);


#endif

