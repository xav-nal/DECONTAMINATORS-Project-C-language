#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "utilitaire.h"

#define P_CARRE		2
#define TOUR		2

// **Fonctions exportées 

// renvoie la distance entre les points a et b
double 	util_distance(S2D a, S2D b)
{
	double d = VALEUR_INITIALE;
	
	//d = sqrt(pow(b.x-a.x,P_CARRE) + pow(b.y-a.y,P_CARRE));
	
	d = sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
	
	return d;
}

// renvoie l'angle que fait le bipoint ab et l'axe X du monde. 
// L'angle doit être en radians et compris dans l'intervalle ]-pi, +pi]
double 	util_angle(S2D a, S2D b)
{
	double angle = VALEUR_INITIALE;
	
	angle = atan2((b.y - a.y), (b.x - a.x));

	util_range_angle(&angle);

	return angle;
}

// modifie si nécessaire l'angle pointé par p_angle 
// pour qu'il soit compris dans l'intervalle ]-pi, +pi]
void util_range_angle(double * p_angle)
{	
	if(p_angle)
	{	
		while(*p_angle > M_PI) 
			*p_angle -= TOUR * M_PI;
			
		while(*p_angle <= -M_PI)
			*p_angle += TOUR * M_PI;
	}
}

// renvoie VRAI si le point est en dehors du domaine [-max, max]  		      
bool 	util_point_dehors(S2D a, double max)
{
	if( a.x > max || a.x < -max) 
		return VRAI;
	else 
		return FAUX;
		
	if( a.y > max || a.y < -max) 
		return VRAI;
	else 
		return FAUX;
}

// renvoie VRAI si l'angle alpha est en dehors de l'intervalle [-pi, pi]	   XAVIER
bool 	util_alpha_dehors(double alpha)
{
	if( alpha > M_PI || alpha < -M_PI) 
		return VRAI;
		
	else
		return FAUX;
}

// renvoie VRAI si le point a est dans le cercle c 
// plus précisément: si la distance de a au centre de c  < rayon - EPSIL_ZERO  XAV
bool 	util_point_dans_cercle(S2D a, C2D c)
{
	double distance = VALEUR_INITIALE;
	distance = util_distance(a,c.centre);
	
	if( distance < c.rayon - EPSIL_ZERO	)
		return VRAI;
		
	else
		return FAUX;
}

// renvoie VRAI en cas de collision des cercles a et b selon l'Equ. 4          XAV
// le paramètre de sortie p_dist est la distance entre les centres de a et b
bool 	util_collision_cercle(C2D a, C2D b, double * p_dist) 
{	
	double d = util_distance(a.centre, b.centre);
	
	if(p_dist)
		*p_dist = d;
	
	if( d < (a.rayon + b.rayon) - EPSIL_ZERO)
		return VRAI;

	else
		return FAUX;
		
}

// renvoie la position obtenue après déplacement du point p d'une distance dist
// dans la direction définie par l'angle alpha
S2D 	util_deplacement(S2D p, double alpha, double dist)
{
	S2D p2;
	p2.x = VALEUR_INITIALE;
	p2.y = VALEUR_INITIALE;
	
	p2.x = p.x + dist*cos(alpha);
	p2.y = p.y + dist*sin(alpha);
	
	return p2;
}


// renvoie VRAI si la distance de a à b > EPSIL_ZERO et renvoie FAUX sinon. 	XAV
// DE PLUS, dans le cas VRAI on utilise p_ecart_angle (qui doit être défini) 
// pour récupérer l'écart angulaire entre le bipoint ab et un vecteur d'angle alpha. 
// La valeur de l'écart angulaire doit être dans l'intervalle [-pi, pi].
bool	util_ecart_angle(S2D a, double alpha, S2D b, double *p_ecart_angle)
{
	double distance = VALEUR_INITIALE;
	distance = util_distance(a,b);
	if( distance > EPSIL_ZERO)
	{ 
		double beta = VALEUR_INITIALE;
		beta = util_angle(a,b);
		
		*p_ecart_angle = beta - alpha;
		
		util_range_angle(p_ecart_angle);
		
		return VRAI;
	}
	else
		return FAUX;
}

// renvoie VRAI si un vecteur d'angle alpha est aligné avec le vecteur ab. Pour
// déterminer cela on obtient l'écart angulaire avec la fonction util_ecart_angulaire
// renvoie VRAI si la valeur absolue de
// cet écart angulaire < EPSIL_ALIGNEMENT. Renvoie FAUX pour tous les autres cas.
bool 	util_alignement(S2D a, double alpha, S2D b)
{
	double delta_angle = VALEUR_INITIALE;
	
	util_ecart_angle(a, alpha, b, &delta_angle);
	
	if (fabs(delta_angle) < EPSIL_ALIGNEMENT)
		return VRAI;
		
	else 
		return FAUX;
		
}

// renvoie VRAI si on peut calculer la nouvelle longueur du coté a lorsqu'on change
// la longueur du coté b, la longueur du coté c restant constante. Les longueurs des 
// cotés a,b,c sont notées la, lb, lc. La nouvelle longueur du coté b est lb_new. 
// le paramètre de sortie p_la_new doit être défini. Renvoie VRAI si:
// les 3 longueurs la, lb et lc > EPSIL_ZERO et lb_new se trouve dans l'intervalle
// autorisé [lb, lc]. Le calcul de la_new est donné par l'Equ.5 qui résoud le cas 
// particulier de la Fig 5c avec:
//     la     = delta_d, lb = D, lc = L, lb_new = r1+r2
//     la_new = delta_d'
//
bool 	util_inner_triangle(double la, double lb, double lc, double lb_new,
						    double * p_la_new)
{
	//~ double cBeta = VALEUR_INITIALE;
	//~ double delta = VALEUR_INITIALE;
	
	//~ cBeta = ((pow(la,P_CARRE) + pow(lc,P_CARRE) - pow(lb,P_CARRE)) / (2*la*lc) );
	
	//~ delta = ( 4*pow(lc,P_CARRE)*pow(cBeta,P_CARRE) )-
			//~ ( 4*( pow(lc,P_CARRE)- pow(lb_new,P_CARRE) ));
	
	//~ if (delta < VALEUR_INITIALE)
		//~ return FAUX;
	
	//~ if (delta > VALEUR_INITIALE)
	//~ {
		//~ double x1 = VALEUR_INITIALE;
		//~ double x2 = VALEUR_INITIALE;
		
		//~ x1 = ((2*lc*cBeta) - sqrt(delta))/HALF;
		//~ x2 = ((2*lc*cBeta) + sqrt(delta))/HALF;
	
		//~ if(x1 < x2)*p_la_new = x1;
		//~ else
			//~ *p_la_new = x2;
			
		//~ return VRAI;
	//~ }
	
	//~ else
	//~ {
		//~ *p_la_new = (2*lc*cBeta) / HALF;
		//~ return VRAI;
	//~ }
	
	if (la > EPSIL_ZERO && lb >= 0. && lc > EPSIL_ZERO && // lb peut être nul
	    lb_new >= lb    && lb_new <= lc)
	{
		double l_cosb = (la*la + lc*lc - lb*lb)/(2*la);   // L*cosb

		// par construction l'angle beta est inférieur à pi/2
		// donc lcosb doit être positif 
		
		// coef a,b,c de l'equ du second degré: ax² + bx + c = 0
		//double a = 1.;
		double b = -2*l_cosb ; 
		double c = lc*lc - lb_new*lb_new;
		
		double delta = b*b -4*c ;
	
		if(delta < 0.)			
			return false;
			
		delta = sqrt(delta) / 2;
	
		// solutions: 0.5*( -b +/- sqrt(delta)) doivent être positifs
		// notre solution est la plus petite qui doit être positive
		if( (l_cosb - delta ) < 0.) // erreur numérique: on renvoie 0.
		{
			* p_la_new = 0. ;
			return false;
		}

		* p_la_new = l_cosb - delta ;
	
		return true;
	}
	return false;
	
}


