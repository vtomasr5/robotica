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
#include "INIReader.h"
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <cfloat>

#define NUM_SENSORS 16
#define DEBUG 1  // 0 (disabled) or 1 (enabled)

using namespace std;

// Vector de punts
typedef struct {
	double x;
	double y;
} VPunts;

// valors de configuracio 
double TH;
double DIST_MAX;
double DIST_MIN;
double MAX_VEL;
double PES_OBJECTIU;
double PES_OBSTACLE;
double PES_SENSORS[NUM_SENSORS];
double Do; // distancia de marge entre l'objectiu i el robot
double Dr; // angle limit de rotacio
double ANGLE_SEGUIR_PARETS;
double HEADING;
double T; // temps d'espera entre moviments
int TASCA;
VPunts *PUNTS;
int NPUNTS;

const double NO_OBSTACLES = DBL_MAX; // valor maxim en format double

// mostra per pantalla si esta en mode DEBUG
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
    double x = robot3->getX();
    double y = robot3->getY();
    double th = robot3->getTh();

    if (DEBUG) {
        cout << "rX = " << x << endl;
        cout << "rY = " << y << endl;
        cout << "rTh = " << th << endl << endl;
    }
}

// obte el vector objectiu. Retorna l'angle per assolir aquest vector
double vector_objectiu(ArRobot* robot3, VPunts v) 
{
    // Posicio objectiu - posicio robot
	VPunts r;

	r.x = v.x - robot3->getX();
	r.y = v.y - robot3->getY();

	return ArMath::atan2(r.y, r.x); // va a l'objectiu en linia recta
//	return PES_OBJECTIU * ArMath::atan2(r.y, r.x); // va a l'objectiu fent una corba
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
	bool hiha_obstacles = false;

	inminent = false;
	n_obstacles = 0;

    // per a cada lectura de sensor
	for (i = 0; i < robot3->getNumSonar(); i++) {
		lectura = robot3->getSonarReading(i);

        // comprovam si hi ha col·lisio inminent
		if (lectura->getRange() <= DIST_MIN) {
			inminent = true;
			hiha_obstacles = true;
		}

        // comprovam que el llindar del maxim rang de lectura dels sensors 
		if (lectura->getRange() <= DIST_MAX) {
			vobs_i = (DIST_MAX - double(lectura->getRange())) / DIST_MAX;
			n_obstacles++;
			hiha_obstacles = true;
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
	if (hiha_obstacles) return ArMath::atan2(vobs_y, vobs_x) + 180; 

	return NO_OBSTACLES;
}

// consisteix en calcular el vector objectiu i de repulsio per saber cap a on s'ha 
// d'orientar el robot. També té en compte els obstacles del seu voltant i adequa la 
// seva velocitat a la quanitat d'obstacles. Si veu una colisio inminent s'atura.
void moure_robot(ArRobot* robot3, VPunts p) 
{
	VPunts d;
	double angle_actual;
	double angle_anterior;
	double angle_repulsio;
	double angle_objectiu;
	bool inminent;
	int n_obstacles;
	double vel;

    // calculam els vectors objectiu i repulsio
	angle_objectiu = vector_objectiu(robot3, p);
	angle_repulsio = vector_repulsio(robot3, inminent, n_obstacles);

    print("angle_objectiu: " + d2s(angle_objectiu));
    print("angle_repulsio: " + d2s(angle_repulsio));

	if (inminent) robot3->setVel(0);

    //Hi ha obstacles ja que l'angle de repulsio s'ha calculat i per tant es diferent a NO_OBSTACLES
	if (angle_repulsio != NO_OBSTACLES) {
		d.x = 0;
        print("OBSTACLES");
        //Tendrem en compte l'objectiu unicament si no hi ha col·lisio inminent
		if (!inminent) {
			d.x = ArMath::cos(angle_objectiu);
		}

		d.x = d.x + ArMath::cos(angle_repulsio);
		d.y = 0;

        //Tendrem en compte l'objectiu unicament si no hi ha col·lisio inminent
		if (!inminent) {
			d.y = ArMath::sin(angle_objectiu);
		}

		d.y = d.y + ArMath::sin(angle_repulsio);
		angle_actual = ArMath::atan2(d.y, d.x);
	} else { //No hi ha obstacles
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

    // adaptam la velocitat en funcio del numero d'obstacles del voltant
	vel = MAX_VEL / (double)(n_obstacles + 1); 
	robot3->setVel(vel);
}

// comprova si el robot ha arribat al punt objectiu
bool ha_arribat(ArRobot* robot3, VPunts v) 
{
	VPunts r;

    // calculam la "distancia"
	r.x = v.x - robot3->getX();
	r.y = v.y - robot3->getY();

    // comprovam que la "distancia" sigui menor que la distancia de marge entre el robot i el punt objectiu
	if ((ArMath::fabs(r.x) < Do) && (ArMath::fabs(r.y) < Do)) 
        return true;

	return false;
}

// saber si el robot s'esta apropant a l'objectiu
bool mes_aprop_objectiu(ArRobot* robot3, VPunts desti, VPunts posicio_anterior) 
{
	VPunts vobj_actual; //Vector de punts amb objectiu actual
	VPunts vobj_antic; //Vectro de punts amb objectiu antic
	double actual;
	double antic;
	
    vobj_actual.x = desti.x - posicio_anterior.x;
	vobj_actual.y = desti.y - posicio_anterior.y;
	vobj_antic.x = desti.x - robot3->getX();
	vobj_antic.y = desti.y - robot3->getY();

	antic = ArMath::fabs(vobj_actual.x) + ArMath::fabs(vobj_actual.y);
	actual = ArMath::fabs(vobj_antic.x) + ArMath::fabs(vobj_antic.y);
    
    //Si ara estem mes aprop de l'objectiu
	if (actual < antic) 
		return true;

	return false;
}

// fa que el robot segueixi els obstacles parets
// algorisme tipus BUG
void seguir_parets(ArRobot* robot3, VPunts punt) 
{
	VPunts posicio_anterior;
	double angle;
	bool inminent;
	bool atura_seguir = true;
	int n_obstacles;

	posicio_anterior.x = robot3->getX();
	posicio_anterior.y = robot3->getY();

    // si el robot no arribat i hi ha obstacles al seu voltant
	while (!mes_aprop_objectiu(robot3, punt, posicio_anterior) && (n_obstacles != 0)) {
        // calcula el vector de repulsio
		angle = vector_repulsio(robot3, inminent, n_obstacles);

        // si hi ha molts d'obstacles, comença a seguir parets
		if (n_obstacles >= 1) {
			atura_seguir = false;
			angle += ANGLE_SEGUIR_PARETS;
		} else {  // sino deixa de seguir les parets
			if (!atura_seguir) {
				angle = robot3->getTh() + ANGLE_SEGUIR_PARETS;
				atura_seguir = true;
			}
		}

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
	VPunts posicio_antiga;

	posicio_antiga.x = 0.0;
	posicio_antiga.y = 0.0;

	while (!ha_arribat(robot3, punt)) {
		passes++;

		if (passes == 10) { // si ho llevam el robot no fa res i el %cpu se dispara
		    if (!mes_aprop_objectiu(robot3, punt, posicio_antiga)) {
			    encallat = true;
		    }

		    posicio_antiga.x = robot3->getX();
		    posicio_antiga.y = robot3->getY();
		    passes = 0;
		}

		if (encallat) {
            print("Encallat -> bordeja parets");
   			seguir_parets(robot3, punt);
			
            print("Desencallat -> anar cap a l'objectiu");
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
	double menor_distancia = NO_OBSTACLES;
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
		cout << endl << "seguent.x: " << seguent_punt.x << endl << "seguent.y: " << seguent_punt.y << endl;

		anar_a_punt(robot3, seguent_punt);
		cout << "Punt: " << num_punt << endl;
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
    TH = s2d(reader.Get("parametres", "th", "0"));
    MAX_VEL = s2d(reader.Get("parametres", "vel", "0"));
    DIST_MIN = s2d(reader.Get("parametres", "dist_min", "0"));
    DIST_MAX = s2d(reader.Get("parametres", "dist_max", "0"));
    PES_OBJECTIU = s2d(reader.Get("parametres", "pes_objectiu", "0"));
    PES_OBSTACLE = s2d(reader.Get("parametres", "pes_obstacle", "0"));
    HEADING = s2d(reader.Get("parametres", "heading", "0"));
    T = s2d(reader.Get("parametres", "t_espera", "0"));
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
        cout << "th = " << TH << endl;
        cout << "vel = " << MAX_VEL << endl;
        cout << "dist_min = " << DIST_MIN << endl;
        cout << "dist_max = " << DIST_MAX << endl;
        cout << "pes_objectiu = " << PES_OBJECTIU << endl;
        cout << "pes_obstacle = " << PES_OBSTACLE << endl;
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
            break;
		} else { // baix nivell (0)
			anar_a_punt(&robot3, PUNTS[1]); // comença per 1
			print("Ha arribat al punt");
            break;
		}
	}

    delete[] PUNTS;

	robot3.stopRunning();
	Aria::shutdown();
	
    return 0;
}
