// Copyright 2013 Vicenç Juan Tomàs Montserrat <vtomasr5@gmail.com>
// Copyright 2013 Eduardo Gasser <edugasser@gmail.com>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
// MA 02110-1301, USA.

#include <Aria.h>
#include <ariaUtil.h>
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <cfloat>

#include "INIReader.h"

#define NUM_SENSORS 16

using namespace std;

// Vector de punts
typedef struct {
	double x;
	double y;
} VPunts;

// valors de configuracio 
double DIST_MAX;
double DIST_MIN;
double MAX_VEL;
double PES_OBJECTIU;
double PES_SENSORS[NUM_SENSORS];
double Do; // distancia de marge entre l'objectiu i el robot
double Dr; // angle limit de rotacio
double ANGLE_SEGUIR_PARETS;
double HEADING;
double T; // temps d'espera entre moviments
int TASCA;
VPunts *PUNTS;
int NPUNTS;
int DEBUG;

const double NO_OBSTACLES = DBL_MAX; // valor maxim en format double

// mostra texte per pantalla si esta en mode DEBUG
void print(const string& message)
{
    if (DEBUG) cout << endl << message << endl;
}

// convert string to double
double s2d(const string& s)
{
    istringstream i(s);
    double x;
    if (!(i >> x))
        return 0;
    return x;
} 

// convert double to string
string d2s(const double& d)
{
    ostringstream ostream;
    ostream << d;
    return ostream.str();
} 

// mostra la posicio del robot per pantalla
void print_robot(ArRobot* robot3) 
{
    if (DEBUG) {
        double x = robot3->getX();
        double y = robot3->getY();
        double th = robot3->getTh();

        cout << "rX = " << x << endl;
        cout << "rY = " << y << endl;
        cout << "rTh = " << th << endl << endl;
    }
}

// obte el vector objectiu. Retorna l'angle per assolir aquest vector
double vector_objectiu(ArRobot* robot3, VPunts p) 
{
    // Posicio objectiu - posicio robot
	VPunts r;

	r.x = p.x - robot3->getX();
	r.y = p.y - robot3->getY();

	//return ArMath::atan2(r.y, r.x);
	return PES_OBJECTIU * ArMath::atan2(r.y, r.x); 
}

// calcula el vector de repulsio. Retorna l'angle
double vector_repulsio(ArRobot* robot3, bool &inminent, int &n_obstacles) 
{
    // (Posicio obstacle - posicio robot) + 180º
	ArSensorReading *lectura;
	int i;
	double vobs_i;
	double vx;
	double vy;
	double vobs_x = 0;
	double vobs_y = 0;
	bool obstacles = false;

	inminent = false;
	n_obstacles = 0;

    // per a cada lectura de sensor
	for (i = 0; i < robot3->getNumSonar(); i++) {
		lectura = robot3->getSonarReading(i);

        // comprovam si hi ha col·lisio inminent
		if (lectura->getRange() <= DIST_MIN) {
			inminent = true;
			obstacles = true;
		}

        // comprovam que el llindar del maxim rang de lectura dels sensors 
		if (lectura->getRange() <= DIST_MAX) {
			vobs_i = (DIST_MAX - double(lectura->getRange())) / DIST_MAX;
			n_obstacles++;
			obstacles = true;
		} else {
			vobs_i = 0;
		}

		vx = vobs_i * PES_SENSORS[i] * ArMath::cos(lectura->getSensorTh() + robot3->getTh());
		vy = vobs_i * PES_SENSORS[i] * ArMath::sin(lectura->getSensorTh() + robot3->getTh());

        // incrementam el valor del nou vector d'obstacles respecte a les lectures del sensors
		vobs_x += vx;
		vobs_y += vy;
	}

    // el vector de repulsio ha d'anar en sentit contrari a l'obstacle
	if (obstacles) return ArMath::atan2(vobs_y, vobs_x) + 180; 

	return NO_OBSTACLES;
}

// consisteix en calcular el vector objectiu i de repulsio per saber cap a on s'ha 
// d'orientar el robot. També té en compte els obstacles del seu voltant i adequa la 
// seva velocitat a la quanitat d'obstacles. Si veu una colisio inminent s'atura.
void moure_robot(ArRobot* robot3, VPunts punt) 
{
	VPunts p;
	double angle_actual;
	double angle_anterior;
	double angle_repulsio;
	double angle_objectiu;
	bool inminent;
	int n_obstacles;
	double vel;

    // calculam els vectors objectiu i repulsio
	angle_objectiu = vector_objectiu(robot3, punt);
	angle_repulsio = vector_repulsio(robot3, inminent, n_obstacles);

    print("angle_objectiu: " + d2s(angle_objectiu));
    print("angle_repulsio: " + d2s(angle_repulsio));

	if (inminent) robot3->setVel(0);

    // hi ha obstacles ja que l'angle de repulsio s'ha calculat i per tant es diferent a NO_OBSTACLES
	if (angle_repulsio != NO_OBSTACLES) {
		p.x = 0;
        print("OBSTACLES");
        // tindrem en compte l'objectiu unicament si no hi ha col·lisio inminent
		if (!inminent) p.x = ArMath::cos(angle_objectiu);

		p.x += ArMath::cos(angle_repulsio);
		p.y = 0;

        // tindrem en compte l'objectiu unicament si no hi ha col·lisio inminent
		if (!inminent) p.y = ArMath::sin(angle_objectiu);

    	p.y += ArMath::sin(angle_repulsio);
            
        // calculam l'angle
		angle_actual = ArMath::atan2(p.y, p.x);
	} else { // no hi ha obstacles
        print("NO OBSTACLES");
		angle_actual = angle_objectiu;
	}

	angle_anterior = robot3->getTh();

	if (angle_anterior < 0)	angle_anterior += 360;

    // orientam el robot
	robot3->setHeading(angle_actual); 

	// o be hi ha una col·lisio inminent o be esta massa desorientat respecte a l'objectiu. Esperam que acabi.
	if ((inminent) || (ArMath::fabs(angle_actual - angle_anterior) >= Dr)) {
		while(!robot3->isHeadingDone(HEADING));
	}

    // adaptam la velocitat en funcio del numero d'obstacles del seu voltant
	vel = MAX_VEL / (double)(n_obstacles + 1); 
	robot3->setVel(vel);
}

// comprova si el robot ha arribat al punt objectiu
bool ha_arribat(ArRobot* robot3, VPunts p) 
{
	VPunts r;

    // calculam la "distancia"
	r.x = p.x - robot3->getX();
	r.y = p.y - robot3->getY();

    // comprovam que la "distancia" sigui menor que la distancia de marge entre el robot i el punt objectiu
	if ((ArMath::fabs(r.x) < Do) && (ArMath::fabs(r.y) < Do)) 
        return true;

	return false;
}

// saber si el robot s'esta apropant a l'objectiu
bool mes_aprop_objectiu(ArRobot* robot3, VPunts desti, VPunts pos_anterior) 
{
	VPunts vobj_actual; // vector de punts amb objectiu actual
	VPunts vobj_antic; // vectro de punts amb objectiu antic
	double actual;
	double antic;
	
    vobj_actual.x = desti.x - pos_anterior.x;
	vobj_actual.y = desti.y - pos_anterior.y;
	vobj_antic.x = desti.x - robot3->getX();
	vobj_antic.y = desti.y - robot3->getY();

	antic = ArMath::fabs(vobj_actual.x) + ArMath::fabs(vobj_actual.y);
	actual = ArMath::fabs(vobj_antic.x) + ArMath::fabs(vobj_antic.y);
    
    // si estem mes aprop de l'objectiu
	if (actual < antic) 
		return true;

	return false;
}

// fa que el robot segueixi els obstacles parets
void seguir_parets(ArRobot* robot3, VPunts punt) 
{
	VPunts pos_anterior;
	double angle;
	bool inminent;
	bool atura_seguir = true;
	int n_obstacles;

	pos_anterior.x = robot3->getX();
	pos_anterior.y = robot3->getY();

    // si el robot no arribat i hi ha obstacles al seu voltant
	while (!mes_aprop_objectiu(robot3, punt, pos_anterior) && (n_obstacles != 0)) {
        // calcula el vector de repulsio
		angle = vector_repulsio(robot3, inminent, n_obstacles);

        // si hi ha obstacles, comença a seguir parets
		if (n_obstacles >= 1) {
			atura_seguir = false;
			angle += ANGLE_SEGUIR_PARETS;
		} else {  // sino deixa de seguir les parets
			if (!atura_seguir) {
				angle = robot3->getTh() + ANGLE_SEGUIR_PARETS;
				atura_seguir = true;
			}
		}

        print("angle_parets = " + d2s(angle));
		robot3->setHeading(angle);
		while(!robot3->isHeadingDone(HEADING));
		
        // assignam la velocitat adecuada per seguir parets
        robot3->setVel(100);
		ArUtil::sleep(T);
	}
}

// envia el robot a un punt determinat envitant els obstacles
void anar_a_punt(ArRobot* robot3, VPunts punt) 
{
	int passes = 0;
	bool encallat = false;
	VPunts pos_antiga;

	pos_antiga.x = 0.0;
	pos_antiga.y = 0.0;

    // mentre el robot no hagi arribata al punt de destí
	while (!ha_arribat(robot3, punt)) {
		passes++;

		if (passes == 20) { 
		    if (!mes_aprop_objectiu(robot3, punt, pos_antiga)) 
			    encallat = true;

		    pos_antiga.x = robot3->getX();
		    pos_antiga.y = robot3->getY();
		    passes = 0;
		}

		if (encallat) {
            print("Robot encallat --> bordeja paret");
   			seguir_parets(robot3, punt);
			
            print("Robot desencallat --> anar cap a l'objectiu");
			encallat = false;
		}

        print_robot(robot3);
		moure_robot(robot3, punt);
        
        // esperam el temps entre cridades del nivell tactic
		ArUtil::sleep(T); 
	}

    // aturam el robot, ja que ha arribat al destí
	robot3->setVel(0);
}

// calcula la distancia entre 2 punts
double distancia(VPunts p1, VPunts p2) 
{
	double x = p1.x - p2.x;
	double y = p1.y - p2.y;

	return sqrt(pow(x, 2) + pow(y, 2));
}

// obté el següent punt, de menor distancia, on ha d'anat el robot
VPunts obtenir_seguent_punt(VPunts pos_actual, bool *visitats, int &num_punt) 
{
	double menor_distancia = DBL_MAX;
	double dist;
		
	for (int i = 0; i < (int)sizeof(PUNTS); i++) {
		dist = distancia(pos_actual, PUNTS[i]);
		if ((!visitats[i]) && (dist < menor_distancia)) {
			num_punt = i;
			menor_distancia = dist;
		}
	}

	visitats[num_punt] = true;
	return PUNTS[num_punt];
}

// tasca alt nivell (viatjant de comerç)
void viatjant_comers(ArRobot* robot3) 
{
	VPunts pos_actual;
	VPunts seguent_punt;
	int num_punt;
	bool visitats[sizeof(PUNTS)];

    // inicialitzacio
	for (int i = 0; i < (int)sizeof(visitats); i++)
		visitats[i] = false;

	for (int i = 0; i < (int)sizeof(PUNTS); i++) {
		pos_actual.x = robot3->getX();
		pos_actual.y = robot3->getY();

		seguent_punt = obtenir_seguent_punt(pos_actual, visitats, num_punt);
        print("seguent.x: " + d2s(seguent_punt.x));
        print("seguent.y: " + d2s(seguent_punt.y));

		anar_a_punt(robot3, seguent_punt);
        print("Punt: " + d2s(num_punt));
	}	
}

// carrega la configuracio des del fitxer config.ini
void carrega_configuracio()
{
    INIReader reader("config.ini");

    if (reader.ParseError() < 0) {
        print("ERROR: Can't load 'config.ini'\n");
        exit(1);
    }
    // read all values from config file
    DEBUG = reader.GetInteger("parametres", "debug", 0);
    MAX_VEL = s2d(reader.Get("parametres", "vel", "0"));
    DIST_MIN = s2d(reader.Get("parametres", "dist_min", "0"));
    DIST_MAX = s2d(reader.Get("parametres", "dist_max", "0"));
    PES_OBJECTIU = s2d(reader.Get("parametres", "pes_objectiu", "0"));
    HEADING = s2d(reader.Get("parametres", "heading", "0"));
    T = s2d(reader.Get("parametres", "t", "0"));
    Do = s2d(reader.Get("parametres", "do", "0"));
    Dr = s2d(reader.Get("parametres", "dr", "0"));
    ANGLE_SEGUIR_PARETS = s2d(reader.Get("parametres", "angle_seguir_parets", "0"));
    TASCA = reader.GetInteger("parametres", "tasca", 0);

    string s = "pes_sensor";
    string s1, s2;
    int i;
    for (i = 0; i < NUM_SENSORS; i++) {
        stringstream ss;
        ss << i;
        s1 = s + ss.str();
        PES_SENSORS[i] = s2d(reader.Get("pesos", s1, "0"));
        s1 = "";
    }

    NPUNTS = reader.GetInteger("punts", "npunts", 0);
    PUNTS = new VPunts[NPUNTS+1];

    int j;
    s = "p";
    for (i = 1; i <= NPUNTS; i++) {
        stringstream ss;
        ss << i;
        s1 = s + ss.str();
        PUNTS[0].x = 0;
        PUNTS[0].y = 0;
        for (j = 1; j < 3; j++) {
            if (j == 1) {
                s2 = s1 + "x";
                PUNTS[i].x = s2d(reader.Get("punts", s2, "0"));               
           } else {           
                s2 = s1 + "y";
                PUNTS[i].y = s2d(reader.Get("punts", s2, "0"));
           }
        }
    }

    if (DEBUG) {
        // print values of config file
        print("Valors del fitxer de configuracio:");
        cout << "debug = " << DEBUG << endl;
        cout << "vel = " << MAX_VEL << endl;
        cout << "dist_min = " << DIST_MIN << endl;
        cout << "dist_max = " << DIST_MAX << endl;
        cout << "pes_objectiu = " << PES_OBJECTIU << endl;
        cout << "temps_espera = " << T << endl;
        cout << "Do = " << Do << endl;
        cout << "Dr = " << Dr << endl;
        cout << "angle_seguir_parets = " << ANGLE_SEGUIR_PARETS << endl;
        cout << "tasca = " << TASCA << endl;

        cout << "\nPesos sensors" << endl;
        for (i = 0; i < NUM_SENSORS; i++) {
            cout << "  Pes sensor " << i << " = " << PES_SENSORS[i] << endl;
        }   

        cout << "\nNumero de punts: " << NPUNTS << endl;
        cout << "Punts definits: " << endl;
        for (i = 1; i <= NPUNTS; i++) {
            cout << "  Punt " << i << endl;
            cout << "    x = " << PUNTS[i].x << endl;
            cout << "    y = " << PUNTS[i].y << endl;
        }
    }
    
    print("Fitxer carregat correctament.");
}

int main(int argc, char **argv) 
{
    ArRobot robot3;

	Aria::init();

	ArSimpleConnector connector(&argc, argv);

	if (!connector.parseArgs() || argc > 1) {
		connector.logOptions();
		exit(1);
	}

	if (!connector.connectRobot(&robot3)) {
		print("Could not connect to robot... exiting");
		Aria::shutdown();
		return 1;
	}

	robot3.comInt(ArCommands::SOUNDTOG, 1);
	robot3.comInt(ArCommands::ENABLE, 1);
	robot3.runAsync(false);	

	carrega_configuracio();

	while (robot3.isRunning()) {
		if (TASCA == 1) { // alt nivell (1)
            viatjant_comers(&robot3); 
		} else { // baix nivell (0)
			anar_a_punt(&robot3, PUNTS[1]); // comença per 1
			print("Ha arribat al punt");
		}
	}

    delete[] PUNTS;

	robot3.stopRunning();
	Aria::shutdown();
	
    return 0;
}
