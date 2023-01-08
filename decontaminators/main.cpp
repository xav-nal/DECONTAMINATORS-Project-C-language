/**
 * \file		main.cpp 
 * \version		Rendu2
 * \date		2018-04-10
 * \author		Nal et Leafe
 * \brief		Main du projet Decontaminators
 *            
 *
 */
 
// *******************************************************************
// 		inclusion de fichiers en-tête avec la directives include

#include <GL/glu.h>
#include <GL/glui.h>


extern "C"
{
	#include <math.h>
	#include <stdlib.h>
	#include "simulation.h"
	#include "constantes.h"
	#include "utilitaire.h"
	#include "graphic.h"
}

 
// *******************************************************************
// 		

void open();

void save();

void start();

void rec();

namespace
{ 
	#define ASPECT_RATIO	 1
	#define NEARVAL			-1
	#define FARVAL		 	 1
	#define WDOW_GLUI_X		 900
	#define WDOW_GLUI_Y		 600
	#define WDOW_GLUT_X		 50
	#define WDOW_GLUT_Y		 100
	#define WDOW_GLUT_SIZE	 700
	#define RECT_MONDE		 2
	#define TAB				 20
	#define VAL				 1.0
	
	enum STATES{START,STOP,OPEN,SAVE,STEP,REC,CONTROL,EXIT};
	
	int main_window;
	int mode = INACTIF;
	int erreur = FAUX;
	int doc = FAUX;
	int num_rob = VALEUR_INITIALE;
	int width,height;
	int lv_start, lv_rec,lv_step;
	int lv_cont_mode = VALEUR_INITIALE;
	
	double taux = VALEUR_INITIALE;
	int cycle = VALEUR_INITIALE;
	
	int mouse_in = VALEUR_INITIALE;
	int rob_man = VALEUR_INITIALE;
	int prog_on = INACTIF;
	
	//Variables graphique
	
	GLUI *glui;
	
	GLfloat aspect_ratio ;
	
	GLUI_Panel *p_open, *p_saving, *p_simul, *p_rec, *p_cont_mod, *p_rob_cont;
	
	GLUI_EditText *ed_open, *ed_saving;
	
	GLUI_Button *but_open, *but_save, *but_start, *but_step, *but_exit;
	
	GLUI_Checkbox *check_record;
	
	GLUI_StaticText *rate, *turn, *trans, *rot;
	
	GLUI_RadioGroup *rd_group_cont;
	
	GLUI_RadioButton *rd_but_auto, *rd_but_manu;
	
	FILE *file_rec = NULL;
}

void display(void)
{
	glClearColor (VAL,VAL,VAL,VAL);
	glClear(GL_COLOR_BUFFER_BIT);
	glLoadIdentity();
	
	if (aspect_ratio <= ASPECT_RATIO)
	  glOrtho(-DMAX, DMAX, -DMAX/aspect_ratio, DMAX/aspect_ratio, NEARVAL, FARVAL);
	else 
	  glOrtho(-DMAX*aspect_ratio, DMAX*aspect_ratio, -DMAX, DMAX, NEARVAL, FARVAL);
	  
	if(mode == DRAW) simulation_display();
	
	glutSwapBuffers();
}  

void reshape(int w, int h)
{
	glViewport(VALEUR_INITIALE, VALEUR_INITIALE, w, h); 
	width = w;
	height = h;
	aspect_ratio = (GLfloat)w / (GLfloat)h ; 
}

void position_souris_monde(int x, int y, double *xm, double *ym)
{
	if (aspect_ratio <= ASPECT_RATIO)
	{
		*xm =((double)(x - (double)(width/HALF))/width)*RECT_MONDE*DMAX;
		*ym =-((double)(y -(double)(height/HALF))/height)*RECT_MONDE*DMAX/aspect_ratio;
	}
	else 
	{
		*xm = ((double)(x - (double)(width/HALF)) /width)*RECT_MONDE*DMAX*aspect_ratio;
		*ym = -((double)(y - (double)(height/HALF)) /height)*RECT_MONDE*DMAX;
	}
}

void file_cb(int control) 
{ 
	switch (control)
	{ 
		case OPEN:
			open();
			glutPostRedisplay();
			break;
			
		case SAVE:
			save();
			break;
			
		case START:
			start();
			break;
			
		case STEP:
			lv_step = ACTIF;
			break;
			
		case REC:
			rec();
			break;
		
		case CONTROL:
			switch(lv_cont_mode)
			{
				case INACTIF:
					lv_cont_mode = ACTIF;
					break;
				
				case ACTIF:
					lv_cont_mode = INACTIF;
					rob_man = VALEUR_INITIALE;
					simulation_put_rob_mode(num_rob,AUTO);
					trans -> set_text("Translation: 0.0000");
					rot -> set_text("Rotation: 0.0000");
					glutPostRedisplay();
					break;				
			}
			break;
	}
} 

void mouse_cb(int button, int state, int x, int y)
{
	if((button == GLUT_LEFT_BUTTON) && (state == GLUT_DOWN) && (lv_cont_mode) 
		&& (mouse_in))	
	{
		int c = VALEUR_INITIALE;
		double xm, ym;
		
		position_souris_monde(x,y,&xm,&ym);
		c = simulation_mouse_robot(xm,ym,num_rob,rob_man);
				
		switch(rob_man)
		{
			case INACTIF:
				if(c)
				{
					rob_man = VRAI;
					num_rob = c-CORRECTION;
					simulation_put_rob_mode(num_rob,MANUEL);
					glutPostRedisplay();
				}
				break;
			
			case ACTIF:
				if(c == FAUX)
				{
					rob_man = FAUX;
					simulation_put_rob_mode(num_rob,AUTO);
					simulation_set_rob_vit(num_rob,FAUX,FAUX,VRAI);
					num_rob = FAUX;
					trans -> set_text("Translation: 0.0000");
					rot -> set_text("Rotation: 0.0000");
					glutPostRedisplay();
				}
				break;	
		}
	}
}

void special_cb(int key, int x, int y)
{
	if((lv_cont_mode) && (rob_man))
	{
		char tab[TAB];
		double tmp;
		switch (key)
		{
			case GLUT_KEY_UP:
				tmp = simulation_set_rob_vit(num_rob,VTRAN,DELTA_VTRAN,FAUX);
				snprintf(tab,TAB,"Translation: %.4f",tmp);
				trans -> set_text(tab);				
				break;
			
			case GLUT_KEY_DOWN:
				tmp = simulation_set_rob_vit(num_rob,VTRAN,-DELTA_VTRAN,FAUX);
				snprintf(tab,TAB,"Translation: %.4f",tmp);
				trans -> set_text(tab);
				break;
			
			case GLUT_KEY_RIGHT:
				tmp = simulation_set_rob_vit(num_rob,VROT,DELTA_VROT,FAUX);
				snprintf(tab,TAB,"Rotation: %.4f",tmp);
				rot -> set_text(tab);
				break;
			
			case GLUT_KEY_LEFT:
				tmp = simulation_set_rob_vit(num_rob,VROT,-DELTA_VROT,FAUX);
				snprintf(tab,TAB,"Rotation: %.4f",tmp);
				rot -> set_text(tab);
				break;
		}
	}
}

void entry_cb(int state)
{
	switch(state)
	{
		case GLUT_ENTERED:
			mouse_in = VRAI;
			break;
			
		case GLUT_LEFT:
			mouse_in = FAUX;
			break;
	}
}

void idle_cb(void)
{
	if(glutGetWindow() != main_window)
		glutSetWindow(main_window);
	
	if(prog_on && (lv_start || lv_step))
	{
		char tab[TAB];
		
		simulation_update();
		
		taux = simulation_rate();
		if(fabs(taux - SIM_FIN) < EPSIL_ZERO) prog_on = INACTIF;
		else cycle++;
		
		snprintf(tab,TAB,"Rate: %.4f",taux);
		rate->set_text(tab);
		
		snprintf(tab,TAB,"Turn: %d",cycle);
		turn->set_text(tab);
		
		if(lv_rec)
		{
			fprintf(file_rec,"%d %lf\n", cycle, taux);
		}
		if(lv_step) lv_step = INACTIF;
		glutPostRedisplay();
	}
	
	if((fabs(taux - SIM_FIN) < EPSIL_ZERO) && (!prog_on))
	{		
		if(lv_rec)
			fclose(file_rec);
			
		lv_start = INACTIF;
		but_start ->set_name("START");	
		lv_cont_mode = INACTIF;
		lv_rec = INACTIF;
		check_record ->set_int_val(INACTIF);
		rd_group_cont -> set_int_val(INACTIF);
		trans -> set_text("Translation: 0.0000");
		rot -> set_text("Rotation: 0.0000");
		taux = VALEUR_INITIALE;	
	}
}

void glui_open(void)
{
	glui = GLUI_Master.create_glui("Decontaminators - Controle", VALEUR_INITIALE, 
								   WDOW_GLUI_X, WDOW_GLUI_Y); 
	glui->set_main_gfx_window( main_window );
	
	p_open = glui->add_panel( "Opening");
	ed_open = glui-> add_edittext_to_panel(p_open,"File Name:");
	ed_open->set_text(".txt");
	but_open = glui-> add_button_to_panel(p_open,"Open", OPEN, file_cb);
	
	p_saving= glui-> add_panel( "Saving");
	ed_saving = glui-> add_edittext_to_panel(p_saving,"File Name:");
	ed_saving->set_text("save.txt");
	but_save = glui-> add_button_to_panel(p_saving,"Save",SAVE, file_cb);
	
	glui->add_column(true);
	
	p_simul = glui-> add_panel( "Simulation");
	but_start = glui-> add_button_to_panel(p_simul, "START", START, file_cb);
	but_step = glui -> add_button_to_panel(p_simul, "STEP", STEP, file_cb);
	
	p_rec = glui-> add_panel( "Recording");
	check_record = glui-> add_checkbox_to_panel(p_rec,"Record",NULL,REC,file_cb);
	rate = glui-> add_statictext_to_panel(p_rec,"Rate : 0.0000");
	turn = glui-> add_statictext_to_panel(p_rec,"Turn : 0");
	
	glui-> add_column(true);
	
	p_cont_mod = glui-> add_panel("Control Mode");
	rd_group_cont = glui-> add_radiogroup_to_panel(p_cont_mod,NULL, CONTROL,file_cb);
	rd_but_auto = glui -> add_radiobutton_to_group(rd_group_cont, "Automatic");
	rd_but_manu = glui -> add_radiobutton_to_group(rd_group_cont, "Manual");
	p_rob_cont = glui -> add_panel("Robot Control");
	trans = glui -> add_statictext_to_panel (p_rob_cont, "Translation : 0.0000");
	rot = glui -> add_statictext_to_panel (p_rob_cont, "Rotation : 0.0000");
	
	but_exit = glui->add_button( "Exit",VALEUR_INITIALE,(GLUI_Update_CB)exit ); 	
	
	GLUI_Master.set_glutIdleFunc(idle_cb);
}

int main(int argc, char **argv)
{
	if(( argv[1] && strcmp("Error", argv[1]) == VALEUR_INITIALE) && argc > 2)
	{
		
		simulation_lecture(argv[2], ERROR);
	
		simulation_liberation_memoire();
	}
	
	 else if((argv[1] && (strcmp("Draw",argv[1]) == VALEUR_INITIALE)) || argc == 1 )
	{
		if (argc > 2)
			simulation_lecture(argv[2], DRAW);
		
		prog_on = ACTIF;
		
		glutInit(&argc, argv);
		glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
		glutInitWindowPosition(WDOW_GLUT_X, WDOW_GLUT_Y);
		glutInitWindowSize(WDOW_GLUT_SIZE, WDOW_GLUT_SIZE);
		
		main_window = glutCreateWindow("Decontaminators");
		
		glClearColor(VAL,VAL,VAL,(double)VALEUR_INITIALE);
		
		width = WDOW_GLUT_SIZE;
		height = WDOW_GLUT_SIZE;
		
		GLUI_Master.set_glutDisplayFunc(display);
		GLUI_Master.set_glutReshapeFunc(reshape);
		GLUI_Master.set_glutSpecialFunc(special_cb);
		glutEntryFunc(entry_cb);
		GLUI_Master.set_glutMouseFunc(mouse_cb);
		GLUI_Master.set_glutIdleFunc(idle_cb);
		
		glutPostRedisplay();
		
		glui_open();
		
		glutSwapBuffers();
		
		glutMainLoop();
	}
	 
	 else printf("Invalid use of this program !\n Syntax: ./projet.x "
				 "[Error|Draw, nom_fichier]\n");
	
	 return EXIT_SUCCESS;
}

void open()
{
	simulation_liberation_memoire();
	erreur = simulation_lecture(ed_open->get_text(), DRAW);
	
	if(erreur == ERROR)
		simulation_liberation_memoire();
		
	cycle = VALEUR_INITIALE;
	doc = ACTIF;
	
	if(lv_start)
		but_start ->set_name("START");
	
	if(erreur == NOT_ERROR) prog_on = ACTIF;
	lv_start = INACTIF;
	lv_cont_mode = INACTIF;
	lv_rec = INACTIF;
	check_record ->set_int_val(INACTIF);
	rd_group_cont -> set_int_val(INACTIF);
	trans -> set_text("Translation: 0.0000");
	rot -> set_text("Rotation: 0.0000");
	rate -> set_text("Rate: 0.0000");
	turn -> set_text("Turn: 0");
}

void save()
{
	if(doc)
	{
		FILE *file_save;
		file_save = fopen(ed_saving->get_text(),"w");
		fprintf(file_save,"%s\n","#SAVING OF SIMULATION");
		simulation_save(file_save);
		fclose(file_save);
		
	}
}

void start()
{
	if(doc)
	{	
		switch(lv_start)
		{  
			case INACTIF:
				lv_start = ACTIF;
				but_start -> set_name("STOP");
				break;
			
			case ACTIF:
				lv_start = INACTIF;
				but_start -> set_name("START");
				if(lv_rec)
				{
					lv_rec = INACTIF;
					check_record->set_int_val(VALEUR_INITIALE);
					fclose(file_rec);
				}
				break;
		}
	}
}

void rec()
{
	if (doc)
	{
		switch(lv_rec)
		{					
			case INACTIF:
				lv_rec = ACTIF;
				file_rec = fopen("out.dat","w");
				fprintf(file_rec,"#REC OPEN \n");
				break;
									
			case ACTIF:
				lv_rec = INACTIF;
				fclose(file_rec);
				break;
		}
	}
}

