/**
 * \file		graphic.c 
 * \version		rendu2
 * \date		2018-04-22
 * \author		Nal et Leafe
 * \brief		Implémentation responsable du modle graphique        
 *
 */
 
// *******************************************************************
// 		inclusion de fichiers en-tête avec la directives include

#include <stdio.h>
#include <math.h>
#include <GL/glu.h>
#include "graphic.h"

#define SIZES 		500
#define VAL_CERCLE 	2

void graphic_draw_rectangle (float xc, float yc, float side, int filled)
{  
	if (filled == FILLED)
		glBegin (GL_POLYGON);
	else
		glBegin (GL_LINE_LOOP);

	glVertex2f (xc+side/HALF, yc+side/HALF);
	glVertex2f (xc-side/HALF, yc+side/HALF);
	glVertex2f (xc-side/HALF, yc-side/HALF);
	glVertex2f (xc+side/HALF, yc-side/HALF);

	glEnd ();	
}

void graphic_draw_segment (double x1, double y1, double x2, double y2 )
{ 
	glLineWidth(1.);
    glBegin(GL_LINE_STRIP);
	
    glVertex2d (x1, y1);
    glVertex2d (x2, y2);
	
    glEnd ();
}

void graphic_draw_circle (double xc, double yc,double rayon,int filled)
{ 
	int i; 
	double alpha,x,y;
	const int SIDES = SIZES;
	
	glLineWidth (2.);
	
	if(filled)
		glBegin (GL_POLYGON);
	
	else 
		glBegin (GL_LINE_LOOP);
	
	for (i=0; i < SIDES; i++)
    {
		alpha = i * VAL_CERCLE * M_PI / SIDES;
		x = xc + rayon * cos (alpha), 
		y = yc + rayon * sin (alpha); 
		
		glVertex2d (x, y); 
	}
	
	glEnd ();
}

void graphic_set_color3f(float r,float g,float b)
{
	glColor3f(r, g, b);
}
