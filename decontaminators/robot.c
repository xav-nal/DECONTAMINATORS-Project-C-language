/**
 * \file		288275.c 
 * \version		rendu2
 * \date		2018-04-22
 * \author		Nal et 288275
 * \brief		Implémentation du nmodule responsable des robots
 *            
 *
 */

// *******************************************************************
// 		inclusion de fichiers en-tête avec la directives include
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "robot.h"

#define NBINFO_ROB		3
#define DEB_ROB			1
#define SCORE			10
#define PROCHE			0.9
#define MOYEN			0.7
#define ELOIGNE			0.5
#define LOIN		    0.4
#define D_VITAL			1.5*R_ROBOT
#define D_PROCHE_MOY    5
#define D_MOY_ELOI      15
#define D_ELOI_LOIN     20
#define COEFF_TOT		4
#define COEFF_PART		3
#define ALPHA_LIM	    M_PI/4
#define DELTA	 		0.1

enum Etat_robot{NBROBOT,ROBOT,FIN_ROBOT};

typedef struct ROBOT_POSITION ROBOT_POSITION;
struct ROBOT_POSITION
{
	S2D centre;
	double alpha;
	C2D objectif;
	int col_part;
	int mode;
	double vrot;
	double vtran;
};

//Déclaration des fonctions propres à ce module

int robot_lecture(char *tab,int nb_robot, int *p_ligne, int mode);
void robot_inilialisation_valeurs(int num_rob);
void robot_position(double x, double y, double alpha);

void robot_deplacement(int num_rob, C2D but);
void robot_rotation(int numrob,S2D particule,double ecart_angle);
void robot_translation(int numrob,S2D particule,double ecart_angle);

void robot_sim_col_rr_update(int n_rob, S2D robot_coord);
bool robot_robot_simulation_collision(int num_rob, int *rob_col);
void robot_deplacement_apres_collision_rr(int num_rob,int rob_col, S2D robot_coord);

bool robot_sim_col_rp_update(int n_rob, S2D robot_coord, C2D *part);
bool robot_particule_simulation_collision(int num_rob, C2D *part);
void robot_deplacement_apres_collision_rp(int num_rob,S2D robot_coord, C2D particule);

void robot_decontamination_update(int n_rob, C2D part);
bool robot_particule_test_decontamination(int num_rob, C2D part);

void robot_draw_robot (double x, double y, double alpha, int mode);

void robot_manual_update(int nrb);

double robot_particule_score(int i, PARTICULE *part, PARTICULE **suite, 
							 C2D* particule_cible);

int robot_particule_cible_existance(int num_rob);

void robot_initialisation_S2D(S2D *pnt);
void robot_initialisation_C2D(C2D *cercle);


static int nb_robot = VALEUR_INITIALE;
static int compteur_robot = VALEUR_INITIALE;
static int numero_robot = VALEUR_INITIALE;
static double vrot = VALEUR_INITIALE;
static double vtran = VALEUR_INITIALE;

ROBOT_POSITION *robot = NULL;

//Fonctions exportées
int robot_decodage(char *tab,int *p_ligne,int mode, int *p_etat)
{
	int i = VALEUR_INITIALE;
	int erreur = FAUX;
	switch(*p_etat)
	{
		case NBROBOT:
			sscanf(tab,"%d", &nb_robot);
			if(nb_robot <= VALEUR_INITIALE)
			{
				error_invalid_nb_robots();
				erreur = ERROR;
				if(mode == ERROR) exit(EXIT_FAILURE);
				else return erreur;
			}
			
			if(nb_robot > VALEUR_INITIALE)
			{
				robot = (ROBOT_POSITION*)malloc(nb_robot*sizeof(ROBOT_POSITION));
				if (robot == NULL) exit(EXIT_FAILURE);
			}
			
			if(robot)
			{
				for(i = VALEUR_INITIALE; i < nb_robot; i++)
				{
					robot_inilialisation_valeurs(i);
				}
			}
			*p_etat = *p_etat + CORRECTION;
			break;
			
		case ROBOT:
			erreur = robot_lecture(tab, nb_robot, p_ligne, mode);
			if (erreur)
				return erreur;
				
			if(nb_robot == VALEUR_INITIALE)
				*p_etat = *p_etat +CORRECTION;
			break;
		
		case FIN_ROBOT:
			if(compteur_robot < nb_robot)
			{
				error_fin_liste_robots(*p_ligne);
				erreur = ERROR;
				if(mode == ERROR) exit(EXIT_FAILURE);
				else return erreur;
			}
			*p_etat = *p_etat + CORRECTION;
			break;	
	}
	return erreur;			
}

int robot_particule_collision(int mode)
{
	int collision = VALEUR_INITIALE;
	int i = VALEUR_INITIALE, j = VALEUR_INITIALE;
	int num_rb = CORRECTION,num_part = CORRECTION;
	int nb_part = VALEUR_INITIALE;
	int erreur = VALEUR_INITIALE;
	double distance = VALEUR_INITIALE;
	
	nb_part = particule_nb_particule();
	
	C2D part, rob;
	rob.centre.x = robot[i].centre.x;
	rob.centre.y = robot[i].centre.y;
	rob.rayon = R_ROBOT; 
	
	for(j = 0; j < nb_part; j++)
	{
		part = particule_position(j);
		
		for(i = 0; i < nb_robot; i++)
		{
			rob.centre.x = robot[i].centre.x;
			rob.centre.y = robot[i].centre.y;
			distance = util_distance(robot[i].centre,part.centre);
			
			collision = util_collision_cercle(rob, part, &distance); 
			
			if(collision == VRAI)
			{
				error_collision(ROBOT_PARTICULE, num_rb, num_part);
				erreur = ERROR;
				if(mode == ERROR) exit(EXIT_FAILURE);
				else return erreur;
			}
			num_rb++;
		}
		num_rb = CORRECTION;
		num_part++;
	}
    return erreur;	
}

void robot_liberation_memoire()
{
	//initialisation des parametres
	nb_robot = VALEUR_INITIALE;
	compteur_robot = VALEUR_INITIALE;
	numero_robot = VALEUR_INITIALE;
	
	if(robot != NULL)
	{
		free(robot);
		robot = NULL;
	}
}

int robot_update()
{
	int i = VALEUR_INITIALE;
	int collision = VALEUR_INITIALE;
	int update = VALEUR_INITIALE;
	S2D robot_coord;
	C2D part;
	
	PARTICULE* tete_de_liste = particule_get_liste_part();
	
	robot_initialisation_S2D(&robot_coord);
	robot_initialisation_C2D(&part);
	
	if(tete_de_liste == NULL)
				return update;
	
	while(i < nb_robot)
	{
		if(robot[i].mode == AUTO)
		{			
			robot_coord = robot[i].centre;
			
			if(!(robot_particule_cible_existance(i)))
				robot[i].col_part = FAUX;
				
			if(!(robot[i].col_part))
				robot_coordination(i);	
			
			robot_deplacement_update(i);
			
			robot_sim_col_rr_update(i,robot_coord);
			
			collision = robot_sim_col_rp_update(i, robot_coord, &part);
			
			if(collision)
			{	
				robot_decontamination_update(i, part);
				collision = VALEUR_INITIALE;
			}
		}
		
		if(robot[i].mode == MANUEL)
		{
			robot_coord = robot[i].centre;
			robot_manual_update(i);
			
			robot_sim_col_rr_update(i,robot_coord);
			collision = robot_sim_col_rp_update(i, robot_coord, &part);
			if(collision)
				collision = VALEUR_INITIALE;
		}	
		i++;
		robot_initialisation_S2D(&robot_coord);
		robot_initialisation_C2D(&part);
	}
	update++;
	return update;
}

void robot_coordination(int i)
{
    double s = VALEUR_INITIALE,tmp = VALEUR_INITIALE;
    PARTICULE *actuel = particule_get_liste_part();
    PARTICULE *next = actuel;
     
    C2D particule_cible;
    particule_cible.centre.x = VALEUR_INITIALE;
    particule_cible.centre.y = VALEUR_INITIALE;
    particule_cible.rayon = VALEUR_INITIALE;
             
    while(next != NULL)
    {
        tmp = robot_particule_score(i, actuel, &next, &particule_cible);
         
        if (tmp > s)
        {
            s = tmp;
            robot[i].objectif = particule_cible;
        }
        actuel = next;
    }
}

void robot_deplacement_update(int num_rob)
{
	robot_deplacement(num_rob,robot[num_rob].objectif);
}

int robot_mouse_robot(double x, double y, int nrb, int rob_man)
{
	int rob = VALEUR_INITIALE;
	S2D souris = {x,y};
	double dist = VALEUR_INITIALE;
	
	if(robot)
	{
		for (int i = 0; i < nb_robot; i++)
		{
			dist = util_distance(robot[i].centre, souris);
			
			if((dist <= (R_ROBOT + EPSIL_ZERO)) && (i == 0 ? i == nrb:(i != nrb)) 
				&& (rob_man == FAUX))
			{
				return rob = i + CORRECTION;
			}
		}
	}
	return FAUX;
}

void robot_put_rob_mode(int nrb,int mode)
{
	if(robot) robot[nrb].mode = mode;
}

double robot_set_vit(int nrb, int val, double delta, int init)
{
	double v = VALEUR_INITIALE;
	
	if(init)
	{
		robot[nrb].vrot = VALEUR_INITIALE;
		robot[nrb].vtran = VALEUR_INITIALE;
		return v;
	}
	
	else
	{
		if(val)
		{
			if(delta >= VALEUR_INITIALE)
			{
				if((robot[nrb].vrot < VROT_MAX))
				{
					robot[nrb].vrot += delta;
					v = robot[nrb].vrot;
				}
				else return v = VROT_MAX;
			}
			else if(robot[nrb].vrot > -VROT_MAX)
			{
				robot[nrb].vrot += delta;
				v = robot[nrb].vrot;
			}
			else return v = -VROT_MAX;	
			return v;
		}
		
		else if(delta >= VALEUR_INITIALE)
		{
			if(robot[nrb].vtran < VTRAN_MAX)
			{
				robot[nrb].vtran += delta;
					v = robot[nrb].vtran;
			}
			else return VTRAN_MAX;
		}
		else 
		{
			if(robot[nrb].vtran > -VTRAN_MAX)
			{
				robot[nrb].vtran += delta;
					v = robot[nrb].vtran;
			}
			else return -VTRAN_MAX;
		}
	}
	return v;
}

void robot_display()
{
	int i;
	if (robot != NULL)
		for(i = 0; i < nb_robot; i++)
		{
			robot_draw_robot(robot[i].centre.x,
							 robot[i].centre.y,robot[i].alpha,robot[i].mode);
		}
}

void robot_save(FILE* file)
{
	int i;
	fprintf(file,"%d\n",nb_robot);
	if (robot != NULL)
	{
		for(i = 0; i < nb_robot; i++)
			fprintf(file,"%lf %lf %lf\n",robot[i].centre.x,robot[i].centre.y,
				    robot[i].alpha);
	}
	fprintf(file,"FIN_LISTE\n\n");
}
	

//fonction propre a ce module

int robot_lecture(char *tab,int nb_robot,int *p_ligne, int mode)
{
	int j = VALEUR_INITIALE, total = VALEUR_INITIALE, count = VALEUR_INITIALE, 
	num_rb = VALEUR_INITIALE,erreur = VALEUR_INITIALE;
	double x = VALEUR_INITIALE, y = VALEUR_INITIALE, alpha = VALEUR_INITIALE,
	distance = VALEUR_INITIALE;
	
	while(sscanf(tab+total, "%lf %lf %lf%n", &x, &y, &alpha, &count) == NBINFO_ROB)
	{
		compteur_robot++;
		if( compteur_robot > nb_robot)
		{
			error_missing_fin_liste_robots(*p_ligne);
			erreur = ERROR;
			if (mode == ERROR) exit(EXIT_FAILURE);
			else return erreur;
		}
		robot_position(x, y, alpha);
		
		if(robot[numero_robot].alpha < -M_PI || robot[numero_robot].alpha > M_PI)
		{
			error_invalid_robot_angle(robot[numero_robot].alpha);
			erreur = ERROR;
			if(mode == ERROR) exit(EXIT_FAILURE);
			else return erreur;
		}
		
		num_rb = DEB_ROB;
		for(j = 0; j < numero_robot; j++)
		{
			distance = util_distance(robot[numero_robot].centre, 
									 robot[j].centre);
			
			if(distance < ( R_ROBOT + R_ROBOT) - EPSIL_ZERO)
			{
				error_collision(ROBOT_ROBOT,num_rb, compteur_robot);
				erreur = ERROR;
				if(mode == ERROR) exit(EXIT_FAILURE);
				else return erreur;
			}
			num_rb++;
		}
		total += count;
		numero_robot++;	
	}
	if(sscanf(tab+total, "%lf %lf %lf%n", &robot[numero_robot].centre.x, 
				&robot[numero_robot].centre.y, &robot[numero_robot].alpha, 
				&count)!=NBINFO_ROB && sscanf(tab+total, "%lf %lf %lf%n", 
				&robot[numero_robot].centre.x, &robot[numero_robot].centre.y, 
				&robot[numero_robot].alpha, &count) > VALEUR_INITIALE)
	{
		error_invalid_robot();
		erreur = ERROR;
		if(mode == ERROR) exit(EXIT_FAILURE);
		else return erreur;
	}
	return erreur;
}

void robot_inilialisation_valeurs(int num_rob)
{
	C2D objectif_zero;
	robot_initialisation_C2D(&objectif_zero);
	
	robot[num_rob].centre.x = VALEUR_INITIALE;
	robot[num_rob].centre.y = VALEUR_INITIALE;
	robot[num_rob].alpha = VALEUR_INITIALE;
	robot[num_rob].mode = AUTO;
	robot[num_rob].vrot = VALEUR_INITIALE;
	robot[num_rob].vtran = VALEUR_INITIALE;
	robot[numero_robot].objectif = objectif_zero;
	robot[numero_robot].col_part = VALEUR_INITIALE;
}

void robot_position(double x, double y, double alpha)
{
	robot[numero_robot].centre.x = x;
	robot[numero_robot].centre.y = y;
	robot[numero_robot].alpha = alpha;
}


void robot_deplacement(int num_rob, C2D but)
{
	double distance = VALEUR_INITIALE;
	double ecart_angle = VALEUR_INITIALE;
	
	if(robot != NULL)
	{
		util_ecart_angle(robot[num_rob].centre, robot[num_rob].alpha, but.centre, 
						 &ecart_angle);
		
		if(!(util_alignement(robot[num_rob].centre, robot[num_rob].alpha, but.centre)))
		{
			robot_rotation(num_rob,but.centre, ecart_angle);
		}
		
		util_ecart_angle(robot[num_rob].centre, robot[num_rob].alpha, but.centre, 
						 &ecart_angle);
		distance = util_distance(robot[num_rob].centre, but.centre);
		
		if(distance > (R_ROBOT + but.rayon) - EPSIL_ZERO)
		{
			robot_translation(num_rob, but.centre, ecart_angle);
		}
	}
}

void robot_rotation(int numrob,S2D particule, double ecart_angle)
{
	if(ecart_angle < VALEUR_INITIALE)
	{
		if(ecart_angle < -VROT_MAX*DELTA_T)
			vrot = -VROT_MAX;
		else
			vrot = ecart_angle/DELTA_T;
	}
	else if(ecart_angle > VALEUR_INITIALE)
	{
		if(ecart_angle > VROT_MAX*DELTA_T)
			vrot = VROT_MAX;
		else
			vrot = ecart_angle/DELTA_T;
	}
	
	//determination de l angle alpha
	robot[numrob].alpha += vrot*DELTA_T;
	
	util_range_angle(&robot[numrob].alpha);
}

void robot_translation(int numrob, S2D particule, double ecart_angle)
{
	double distance = VALEUR_INITIALE;
	
	if(ecart_angle < ALPHA_LIM && ecart_angle > -ALPHA_LIM)
	{		
		//calcul de la distance
		distance = util_distance(robot[numrob].centre, particule);
		
		//calcul de la vitesse de translation
		if(distance > VTRAN_MAX * DELTA_T )
			vtran = VTRAN_MAX;
		else
			vtran = distance/DELTA_T;
		
		//calcul de la distance parcouru
		distance = vtran * DELTA_T;
		
		robot[numrob].centre = util_deplacement(robot[numrob].centre, 
												robot[numrob].alpha, distance);
	}
}


void robot_sim_col_rr_update(int n_rob, S2D robot_coord)
{
	int num_rob = 0;

	int collision = VALEUR_INITIALE;
	
	collision = robot_robot_simulation_collision(n_rob, &num_rob);
			
	if(collision)
	{
		robot_deplacement_apres_collision_rr(n_rob, num_rob, robot_coord);
		collision = VALEUR_INITIALE;
	}
}

bool robot_robot_simulation_collision(int num_rob, int *rob_col)
{
	int i = VALEUR_INITIALE;
	double distance = VALEUR_INITIALE;
	
	for(i = 0; i < nb_robot; i++)
	{
		if(i != num_rob)
		{
			distance = util_distance(robot[num_rob].centre, 
									 robot[i].centre);
			if(distance < ( R_ROBOT + R_ROBOT) - EPSIL_ZERO)
			{
				*rob_col = i;
				return true;
			}
		}
	}
	return false;
}

void robot_deplacement_apres_collision_rr(int num_rob,int rob_col, S2D robot_coord)
{
	int rb_col = 0;
	int collision = 0;
	double deplacement = VALEUR_INITIALE;
	
	if(robot[num_rob].mode)
	{		
		//calcul de la distance
		deplacement = util_distance(robot_coord ,robot[num_rob].centre);
		
		//calcul de la vitesse de translation
		if(deplacement > VTRAN_MAX * DELTA_T )
			vtran = VTRAN_MAX;
		else
			vtran = deplacement / DELTA_T;
		
		deplacement = -VTRAN_MAX * DELTA_T;
		//printf("bloquage on recule %lf\n", deplacement);
	}
		
	robot[num_rob].centre = util_deplacement(robot_coord, robot[num_rob].alpha,
											 deplacement);
	
	collision = robot_robot_simulation_collision(num_rob, &rb_col);
	
	deplacement = VALEUR_INITIALE;
	
	if(collision)
		robot[num_rob].centre = util_deplacement(robot_coord, robot[num_rob].alpha,
												 deplacement);
}



bool robot_sim_col_rp_update(int n_rob, S2D robot_coord, C2D *part)
{
	int collision = VALEUR_INITIALE;
	
	collision = robot_particule_simulation_collision(n_rob, part);
	
	if(collision)
	{
		robot_deplacement_apres_collision_rp(n_rob,robot_coord,*part);
		return true;
	}
	
	return false;
}

bool robot_particule_simulation_collision(int num_rob, C2D *part)
{
	int collision = 0;
	int j = 0;
	int nb_part = 0;
	double distance = 0;
	nb_part = particule_nb_particule();
	
	C2D rob;
	rob.centre.x = robot[num_rob].centre.x;
	rob.centre.y = robot[num_rob].centre.y;
	rob.rayon = R_ROBOT; 
	
	for(j = 0; j < nb_part; j++)
	{
		*part = particule_position(j);
		
		collision = util_collision_cercle(rob, *part, &distance);
		
		if(collision)
			return true;
	}
	return false;
}

void robot_deplacement_apres_collision_rp(int num_rob,S2D robot_coord, C2D particule)
{
	int deplacement = 0;
	double la = 0, lb = 0, lc = 0;
	double lb_new = 0, la_new = 0;
	
	if(robot[num_rob].mode)
	{
		la = util_distance(robot_coord,particule.centre);
		lb = util_distance(robot[num_rob].centre, particule.centre);
		lc = util_distance(robot_coord,particule.centre);
		lb_new = R_ROBOT + particule.rayon;
		
		deplacement = util_inner_triangle(la, lb, lc, lb_new, &la_new);	
		
		if(!(deplacement))
		{
			//~ printf("Le deplacement apres collision entre robot-particule" 
					//~ "est impossible\n");
			la_new = 0;
		}
	}
	robot[num_rob].centre = util_deplacement(robot_coord, robot[num_rob].alpha,
											 la_new);
	robot[num_rob].objectif = particule;
	robot[num_rob].col_part = VRAI;
}


void robot_decontamination_update(int n_rob, C2D part)
{
	int test_decontamination = VALEUR_INITIALE;
	
	test_decontamination = robot_particule_test_decontamination(n_rob, part);
				
	if(test_decontamination)
	{
		particule_nettoyage(part.centre);
		robot[n_rob].col_part = FAUX;
	}
}

bool robot_particule_test_decontamination(int num_rob, C2D part)
{
	if(util_alignement(robot[num_rob].centre,robot[num_rob].alpha,part.centre))
		return true;
	else
		return false;
}

void robot_draw_robot (double x, double y, double alpha, int mode)
{
	double x2,y2;
	if(mode == MANUEL) graphic_set_color3f(1.0,0.0,0.0);
	else graphic_set_color3f(0.0,0.5,1.0);
	
	graphic_draw_circle (x, y, R_ROBOT,EMPTY);
	x2 = x + R_ROBOT * cos(alpha);
	y2 = y + R_ROBOT * sin(alpha);
	
	graphic_set_color3f(1.0,0.0,0.0);
	graphic_draw_segment (x, y, x2, y2);
}

void robot_manual_update(int nrb)
{
	robot[nrb].alpha += robot[nrb].vrot*DELTA_T;
	robot[nrb].centre = util_deplacement(robot[nrb].centre,robot[nrb].alpha,
									     (robot[nrb].vtran)*DELTA_T);
}

//Renvoie le score d'une particule, calculé à partir du temps mis 
//par un robot à l'atteindre, et son rayon
double robot_particule_score(int nrb, PARTICULE *part, PARTICULE **suite,
							 C2D* particule_cible)
{     
    int nombre;
    double energie; 
    double t,d,s,gamma,beta;
    C2D particule;
     
    particule_get_info(part, &nombre, &energie, &particule, &suite);
     
    *particule_cible =  particule;
     
	d = util_distance(robot[nrb].centre,particule.centre);

	d = d - (particule.rayon+R_ROBOT);
	
	beta = util_angle(robot[nrb].centre,particule.centre);
	
	gamma = fabs(robot[nrb].alpha - beta);
	
	t = (d / VTRAN_MAX * DELTA_T)+(gamma/ VROT_MAX * DELTA_T);
	
	if (d <= D_VITAL)
	{
		s = SCORE - (t * SCORE / T_MAX) + DELTA * particule.rayon;
		return s;
	}
			
	 if ( d > D_VITAL && d <= D_PROCHE_MOY)
	{
		s = PROCHE*((COEFF_PART * (SCORE -(t * SCORE / T_MAX)) + 
					(particule.rayon * SCORE / R_PARTICULE_MAX))/COEFF_TOT);
		return s;
	}
	
	else if (d > D_PROCHE_MOY  && d <= D_MOY_ELOI )
	{
		s = MOYEN * ((SCORE-(t * SCORE / T_MAX)) + 
				   (particule.rayon * SCORE / R_PARTICULE_MAX))/HALF ;
		return s;
	}
	
	else if (d > D_MOY_ELOI  && d <= D_ELOI_LOIN)
	{
		s = ELOIGNE*((SCORE -(t * SCORE / T_MAX)) + 
					 COEFF_PART * (particule.rayon*SCORE / R_PARTICULE_MAX))/COEFF_TOT;
		return s;
	}
		
	else
	{
		s = LOIN * (particule.rayon*SCORE / R_PARTICULE_MAX) ;
		return s;
	}
}

int robot_particule_cible_existance(int num_rob)
{
	int j = 0;
	int nb_part = 0;
	
	nb_part = particule_nb_particule();
	
	C2D part;
	robot_initialisation_C2D(&part);
	
	for(j = 0; j < nb_part; j++)
	{
		part = particule_position(j);
		
		if(robot[num_rob].objectif.centre.x == part.centre.x && 
		   robot[num_rob].objectif.centre.y == part.centre.y)
			return true;
	}
	return false;
}


void robot_initialisation_S2D(S2D *pnt)
{
	pnt->x = VALEUR_INITIALE;
	pnt->y = VALEUR_INITIALE;
}
void robot_initialisation_C2D(C2D *cercle)
{
	cercle->centre.x = VALEUR_INITIALE;
	cercle->centre.y = VALEUR_INITIALE;
	cercle->rayon = VALEUR_INITIALE;
}







