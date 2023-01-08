# Compilateur a utiliser :
##########################

CC     = gcc

# Options de compilation : (-g = pour deverminer)
#################################################

CFLAGS = -g


# Librairies a utiliser :
# LIBS: toutes les librairies qu'il faut intégrer
#########################

LIBS   = -lstdc++ -lglut -lGL -lGLU -lm -Llib -lglui -L/usr/X11R6/lib -lX11 -lXext -lXmu -lXi

# Liste de fichiers source (.c) a compiler :
############################################

CFILES = main.cpp error.c graphic.c simulation.c  particule.c robot.c utilitaire.c

# Liste de modules objets (generee a partir de CFILES) :
# Les fichiers objets (.o) ont le même nom que les fichiers source (.c)
# Seulement le suffixe change.
########################################################

OFILES = $(CFILES:.c=.o)

# Nom du fichier executable :
#############################

CIBLE  = projet.x

# edition de liens (rassembler tous les objets -> executable)
#############################################################

glui.x: $(OFILES) 
	$(CC) $(OFILES) -o $(CIBLE) ${LIBS}


# compilation separee (production du module objet seulement)

exglui.o: main.cpp 
	$(CC) -c exglui.cpp $(CFLAGS)


# Definitions de cibles particulieres :
#
# "make depend" : genere la liste des dependances
# "make clean"  : efface les fichiers .o et .x
#################################################

depend:
	@echo " *** MISE A JOUR DES DEPENDANCES ***"
	@(sed '/^# DO NOT DELETE THIS LINE/q' Makefile && \
	  $(CC) -MM $(CFLAGS) $(CFILES) | \
	  egrep -v "/usr/include" \
	 ) >Makefile.new
	@mv Makefile.new Makefile

clean:
	@echo " *** EFFACE MODULES OBJET ET EXECUTABLE ***"
	@/bin/rm -f *.o *.x

#
# -- Regles de dependances generees par "make depend"
#####################################################
# DO NOT DELETE THIS LINE
main.o: main.cpp simulation.h error.h utilitaire.h constantes.h \
 tolerance.h graphic.h particule.h robot.h
error.o: error.c error.h constantes.h tolerance.h
graphic.o: graphic.c graphic.h
simulation.o: simulation.c simulation.h error.h utilitaire.h constantes.h \
 tolerance.h graphic.h particule.h robot.h
particule.o: particule.c particule.h error.h utilitaire.h constantes.h \
 tolerance.h graphic.h
robot.o: robot.c robot.h error.h utilitaire.h constantes.h tolerance.h \
 graphic.h particule.h
utilitaire.o: utilitaire.c utilitaire.h constantes.h tolerance.h \
 graphic.h
