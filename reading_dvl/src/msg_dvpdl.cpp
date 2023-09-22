//###########################################################################################
#include "CJoystick.h"
#include "CJoystick.cpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Empty.h>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <stdio.h>
#include <ctime>
#include <vector>
#include <fstream>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/BatteryState.h"						//variable para bateria
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Bool.h"
#include "tf/tf.h"
#include "tf/tfMessage.h"
#define PI 3.14159265358979323846
#define g 9.80665   //Aceleracion de la gravedad

using namespace std;

string dvl1, dvl2, dvl3, dato_anterior;
int i = 0, pos4=0, pos1 = 0, pos2 = 0, pos3 =0, b1 = 0, b2 = 0, c = 0, init = 0, init1 = 0, posf, d1=0, d2=0;
float tiempo = 0, T = 0.05;
float tr[8];
float gc[9];  

//################# Variables Globales #################
double posx, posy, posz, posax, posay, posaz, posaw, posrollrad, pospitchrad, posyawrad, posroll, pospitch, posyaw;
double velx,vely,velz,veax,veay,veaz;
double bat, zbf=0.0,zbf_a=0.0, a_z = 0.0;
float zDes=21.0, xDes=1.0, yDes=1.0, yawDes=128.0, rollDes=20.0, pitchDes =10.0;  	//Referencias
float ez=0, ex=0,  ey=0, eyaw=0, eroll=0, epitch=0;									//Errores
float ex_ant=0,ey_ant=0, ez_ant=0, eyaw_ant=0, eroll_ant=0, epitch_ant=0;	        //Valores anteriores
float ex_p=0, ey_p=0, ez_p=0, eyaw_p=0, eroll_p=0, epitch_p=0;						//Derivadas
double ex_i=0, ey_i=0, ez_i=0, eyaw_i=0, eroll_i=0, epitch_i=0;						//Derivadas
float chVal=1500,b=100;

float kpz=2.50, kiz=0.00, kdz=1.0;  

//////Ganancias control robusto  z
float z1z=0.0, z2z=0.0, uz_rob=0.0;
double ctrl_z=0;
//float gN= 10.0, g_roll=1.0;   				//solo roll
float gNz= 1.0, g_z=0.30;						//roll y z

float kpz2=9.0, kiz2=0.50, kdz2=3.0;  											//Ganancias z PD Saturado
uint16_t uz=0, usz=0;
double zz=0;
bool fl1=0, fl2=0;

float kpx=100.0, kix=5.0, kdx=5.0;  											//Ganancias x
float kpx2=100.0, kix2=5.0, kdx2=5.0;  											//Ganancias x PD Saturado
uint16_t ux=0, xx=0, uxf=0;

float kpy=100.0, kiy=5.0, kdy=5.0; 												//Ganancias y
uint16_t uy=0, yy=0, uyf=0;

float kpyaw=1.0, kiyaw=1.0, kdyaw=1.0; 											//Ganancias yaw
float kpyaw2=1.0, kiyaw2=1.0, kdyaw2=1.0; 										//Ganancias yaw PD Saturado
uint16_t uyaw=0, yyaw=0, usyaw=0;

float kproll=0.80, kdroll=0.2;                           						//ganancias para PD robusto con  referencia fija con controlador en z

float kproll2=8.0, kiroll2=1.0, kdroll2=1.40; 									//Ganancias roll PD Saturado
uint16_t uroll=0, usroll=0;
double  rroll=0;

//float kppitch=1.0, kipitch=1.0, kdpitch=1.0; 									//Ganancias pitch
float kppitch=3.0, kipitch=1.0, kdpitch=1.0; 									//Ganancias pitch  PD

float kppitch2=1.0, kipitch2=1.0, kdpitch2=1.0; 								//Ganancias pitch PD Saturado
uint16_t upitch=0, ppitch=0, uspitch=0;

//ganancias saturaciones
float b_z = 1, b_yaw = 1;             											//Cota de saturacion
float mu_z=0.5, mu_yaw = 0.5;  													//Mu entre 0 y 1

int paro=0,  bajar=0,  armar=0,   control_pos= 0,   desarmar=0, controles=0;
int forwardmove=0, lateralmove=0;
int axis_frwd, axis_latr, axis_vrtl, axis_yaw, axis_roll, axis_pitch;
int satkpz=0, satkiz=0, satkdz=0;
int satkproll=0, satkiroll=0, satkdroll=0;
int satkppitch=0, satkipitch=0, satkdpitch=0;
int satkpyaw=0, satkiyaw=0, satkdyaw=0;

//Ganancias control robusto
float z1roll=0.0, z2roll=0.0, uroll_rob=0.0;
double ctrl_roll=0;
//float gN= 10.0, g_roll=1.0;   				//solo roll
float gN= 10.0, g_roll=1.8;						//roll y z

std_msgs::Bool armState;
std_msgs::String modo;
std::stringstream motxt;
std::stringstream filenameDate;

std_msgs::UInt16 vertical;
std_msgs::UInt16 lateral;
std_msgs::UInt16 fforward;
std_msgs::UInt16 yaw;
std_msgs::UInt16 roll;
std_msgs::UInt16 pitch;

std_msgs::UInt16 verticalc;
std_msgs::UInt16 lateralc;
std_msgs::UInt16 fforwardc;
std_msgs::UInt16 yawc;
std_msgs::UInt16 rollc;
std_msgs::UInt16 pitchc;

string dato;

//################# Funciones Callback #################
//Orientación vehiculo real (Roll, Pitch, Yaw)       				%%%%%%%%%%%
void posCallback(const sensor_msgs::Imu::ConstPtr & msg) 
{
	posax=msg->orientation.x; 
	posay=msg->orientation.y;
	posaz=msg->orientation.z;
	posaw=msg->orientation.w; 
	
  	veax=msg->angular_velocity.x;

	tf::Quaternion q(posax, posay, posaz, posaw);
  	tf::Matrix3x3 m(q);
  	m.getRPY(posrollrad,pospitchrad,posyawrad);
	posroll=posrollrad*(180/PI);
	pospitch=-1*(pospitchrad*(180/PI));

	//ROS_INFO( "posaw%.3f,", msg->orientation.x);
	
	posyaw=(posyawrad*(180/PI));
   	if (posyaw > 180)
	{
      posyaw = posyaw -360;
	  
   	};
}

//Datos DVL
void chatterCallback(const std_msgs::String::ConstPtr & msg)
{
  dato = msg->data.c_str();
  
  if (dato.compare(dato_anterior) != 0)
    {
		if(d2 == 1)
		  	{
      		//dvl1 = dvl1 + dato;
			dvl1 = dvl1.append(dato);
			d1 = 1;
			d2 = 0;			
      	}
		else
		{

      	pos1 = dato.find("$DVPDL");
      
      	if (pos1 > 0 )
      	{
        	dvl1 = dato;
			pos2 = dvl1.find("*");
			pos3 = dvl1.find("*", pos2 + 1);
			if(pos3 >0 )
			{
				d1 = 1; 
			}
			else
				d2 = 1;

      	}
		}
      	if(d1==1)
		{
			ROS_INFO_STREAM( "datoA: " << dvl1 << endl);
			pos4 = dvl1.find("DVPDL")+6;
			for (i=0; i < 8; i++)
            {
                posf = dvl1.find(',', pos4);
                tr[i] = stof(dvl1.substr(pos4, posf-pos4));
                pos4 = posf+1;   
        
            }
			d1=0;
			ROS_INFO_STREAM("Datos: " << tr[0] << "\t" << tr[1]  << "\t" << tr[2] << "\t" << tr[3] << "\t" 
     		<< tr[4] << "\t" << tr[5] << "\t" << tr[6] << "\t" << tr[7]  << "\t" << tiempo << endl ) ;
        
		}
		dato_anterior = dato;
         	              
    }     //FIN DEL DVL 
  
}

float satura(float a);  //Definicion de funcion
float saturayaw(float c);  //Definicion de funcion
float saturob(float c);
int satK(float h, float b, float k, float m);
int sgn(float in);


//################# CICLO PRINCIPAL #################
int main(int argc, char **argv)
{
	ros::init(argc, argv, "dvl_subscriber");
  	ros::NodeHandle n;
  	ros::NodeHandle nh;
  	ros::Subscriber sub = n.subscribe("dvl_out", 10, chatterCallback);

  	//Variables para trayectoria
	int t=0, us=0, signo=0, elem=0, r=0;
	float tiempo2=0, tiempo3=0, paso=0.05, valor=0;
	float tfinal=60,  z_filter_ant=0, t_eje=75;
	float roll_filter_ant=0, pitch_tray_ant=0, pitch_filter_ant=0;
	float t1=0.0;

	float valsin=0, intsin=0;
	int chValz=1480;
	time_t currentTime = time(0);
	tm* currentDate = localtime(&currentTime);

  	ros::init(argc, argv, "octosub_node");
	
	//ros::Subscriber subVel;
	//ros::Subscriber subPos;
	ros::Subscriber subPres;
	ros::Subscriber subBat;

	ros::Publisher pubArm;
	ros::Publisher pubMod;

	//Movimientos
	ros::Publisher pubVertical;
	ros::Publisher pubLateral;
	ros::Publisher pubForward;
	ros::Publisher pubYaw;
	ros::Publisher pubRoll;
	ros::Publisher pubPitch;


	CJoystick *H = new CJoystick();

  	//#################Generar archivo para guardar los datos del DVL##############
  	int archivo;
  	ifstream last_file;
	last_file.open ("/home/cesar/BlueROV2_ws/src/reading_dvl/src/Datos-DVL/lastlog.txt");
	last_file >> archivo;
	last_file.close();

	char str1[80];
	snprintf (str1,80,"/home/cesar/BlueROV2_ws/src/reading_dvl/src/Datos-DVL/data_dvl%d.txt",archivo);

	ofstream last_file1;
	last_file1.open ("/home/cesar/BlueROV2_ws/src/reading_dvl/src/Datos-DVL/lastlog.txt");
	archivo++;
	last_file1 << archivo;
	last_file1.close();

	ofstream myfile;
  	char str2[80];
	snprintf (str2,80,"/home/cesar/BlueROV2_ws/src/reading_dvl/src/Datos-DVL/data_dvl%d.txt",archivo-1);
	myfile.open (str2); // ARCHIVO DONDE SE GUARDAN LOS DATOS DEL UAV1

	//#################Generar archivo para guardar los datos de la IMU##############
  	int archivo2;
  	ifstream last_file2;
	last_file2.open ("/home/cesar/BlueROV2_ws/src/reading_dvl/src/Datos-IMU/lastlog.txt");
	last_file2 >> archivo2;
	last_file2.close();

	char str3[80];
	snprintf (str3,80,"/home/cesar/BlueROV2_ws/src/reading_dvl/src/Datos-IMU/data_imu%d.txt",archivo2);

	ofstream last_file3;
	last_file3.open ("/home/cesar/BlueROV2_ws/src/reading_dvl/src/Datos-IMU/lastlog.txt");
	archivo2++;
	last_file3 << archivo2;
	last_file3.close();

	ofstream myfile2;
  	char str4[80];
	snprintf (str4,80,"/home/cesar/BlueROV2_ws/src/reading_dvl/src/Datos-IMU/data_dvl%d.txt",archivo2-1);
	myfile2.open (str4); // ARCHIVO DONDE SE GUARDAN LOS DATOS DEL UAV1
  

  	//Nodo
  	ros::Subscriber subPos;
  	//################# Subscriptores #################
  	subPos  = nh.subscribe("/BlueRov2/imu/data",1, posCallback);  //Topico real
  
  	pubVertical=nh.advertise<std_msgs::UInt16>("BlueRov2/rc_channel3/set_pwm", 1);
  	pubLateral=nh.advertise<std_msgs::UInt16>("BlueRov2/rc_channel6/set_pwm", 1);
  	pubForward=nh.advertise<std_msgs::UInt16>("BlueRov2/rc_channel5/set_pwm", 1);
  	pubYaw=nh.advertise<std_msgs::UInt16>("BlueRov2/rc_channel4/set_pwm", 1);
  	pubRoll=nh.advertise<std_msgs::UInt16>("BlueRov2/rc_channel2/set_pwm", 1);
  	pubPitch=nh.advertise<std_msgs::UInt16>("BlueRov2/rc_channel1/set_pwm", 1);

  	ros::Subscriber Joystick = nh.subscribe("joy", 1000, &CJoystick::chatterCallback,H);

  	//################# Publicadores #################

  	//Armar
  	pubArm=nh.advertise<std_msgs::Bool>(nh.resolveName("/BlueRov2/arm"), 1);

  	//Modo deph hold
  	pubMod=nh.advertise<std_msgs::String>(nh.resolveName("/BlueRov2/mode/set"), 1);
  	motxt << "manual";
  	modo.data=motxt.str();
	int B1=0;
	int B2=0;

  	ros::Rate loop_rate(20);
  	while (ros::ok())
  	{ 
	    //ROS_INFO_STREAM("Datos: " << tr[0] << "\t" << tr[1]  << "\t" << tr[2] << "\t" << tr[3] << "\t" 
      	//<< tr[4] << "\t" << tr[5] << "\t" << tr[6] << "\t" << tr[7]  << "\t" << tiempo << endl ) ;
		

	    //ROS_INFO_STREAM(dato << endl);


	//Funciones de joystick
		armar= H->a;
    	control_pos= H->b;
    	desarmar=H->y;
    	paro=H->x;
		forwardmove=H->b5;
		lateralmove=H->b6;

    	//manipular roll y pitch
  		axis_latr =  1500 +(H->JYaw*200)*0.5 ;
  		axis_frwd = 1500+(H ->JGaz*200)*0.5;
  		axis_vrtl = 1500-(H ->JPitch*200)*0.5;
  		axis_yaw = 1500+(H ->JRoll*50);
		axis_roll =1500;
		axis_pitch = 1500;

		//Pasa valor de joystick a variables para publicar en topicos de ROS
		vertical.data=axis_vrtl;
		lateral.data=axis_latr;
		fforward.data=axis_frwd;
		yaw.data=axis_yaw;
		roll.data=axis_roll;
		pitch.data=axis_pitch;

		//Alarma bateria baja
		if(bat<=13.4)
    	{
			//printf("BATERIA BAJA\n");
		} 

		if(armar==true)  //boton a
    	{
			printf("Armado\n");
			armState.data=true;
	  	  	pubArm.publish(armState);
		}

    	if(desarmar==true)  //boton y
    	{
			printf("Desarmado\n");
			armState.data=false;
	    	pubArm.publish(armState);
			myfile <<"Fin \n";
    	}

		if(paro==true)  //boton x
    	{
			printf("Paro\n");
			controles=false;
			vertical.data=1500;
			lateral.data=1500;
			fforward.data=1500;
			yaw.data=1500;
			roll.data=1500;
			pitch.data=1500;

	    	pubVertical.publish(vertical);
	    	pubLateral.publish(lateral);
	    	pubForward.publish(fforward);
	    	pubYaw.publish(yaw);
			pubRoll.publish(roll);
			pubPitch.publish(pitch);

			armState.data=false;
	    	pubArm.publish(armState);

			return 0;
    	}

		if(forwardmove==true) //boton b5
		{
			//printf("Forward_move\n");
			controles=true;
			//for (tiempo3=0; i < 50; i++)
			fforward.data=1500+100;
	    	pubForward.publish(fforward);
			B1=1;
		}
		else
		{
			B1=0;
		}

		if(lateralmove==true) //boton b6
		{
			//printf("Lateral_move\n");
			controles=true;
			lateral.data=1500+100;
	    	pubLateral.publish(lateral);
			B2=1;
		}
		else
		{
			B2=0;
		}

    	//Boton de constrol activa una bandera para el ciclo de control que se ejecuta durante cierto tiempo
	  	if(control_pos==true)  //boton b
	  	{	
			printf("Control\n");
			myfile <<  " Control \n";
			controles=true;
			//actualiza referncia yaw
			yawDes=posyaw;
			
	  	}
		
		//################# CICLO DEL CONTROLADOR #################
	  	if(controles == true)
	  	{
			pubVertical.publish(vertical);
	    	pubLateral.publish(lateral);
	    	pubForward.publish(fforward);
	    	//pubYaw.publish(yaw);
		  	pubRoll.publish(roll);
		  	pubPitch.publish(pitch);

			tiempo2=tiempo2+0.05;   //tiempo
		  	
		  	//if(tiempo2<=t_eje)    ///if(tiempo<=tfinal)  cambiar a tfinak que es la duracion de la trayectoria
		  	//{

			//################# INCIA EL CONTROLADOR #################
			//#################################################################################//

			////////////////////////////////////////Control de Yaw
			eyaw=posyaw-yawDes;  												//Error
			eyaw_p=(eyaw-eyaw_ant)/0.05;										//Error p
			eyaw_i=eyaw_i+(eyaw*0.05);

			yyaw=kpyaw*eyaw+kdyaw*eyaw_p+eyaw_i*kiyaw;											//PID

			uyaw=(chVal-yyaw);													//Resta o suma al valor central de pwm 1500
			usyaw=saturayaw(uyaw);														//Satura la señal de pwm entre 1100 a 1900

			//#################################################################################//

			//COMENTAR SI SE REALIZAN PRUEBAS FISICAS CON EL VEHICULO FUERA DEL AGUA
			verticalc.data=usz;													//se envia valor saturado
			pubVertical.publish(verticalc);

			yawc.data=usyaw; 
			pubYaw.publish(yawc);
				
			//////////////////// GUARDA VARIABLES ANTERIORES ////////////////
				
			eyaw_ant = eyaw;

	  	}
	  	else
	  	{
			// MODO DE OPERACION MANUAL
	    	pubVertical.publish(vertical);
	    	pubLateral.publish(lateral);
	    	pubForward.publish(fforward);
	    	pubYaw.publish(yaw);
		  	pubRoll.publish(roll);
		  	pubPitch.publish(pitch);

	  	}
  
		myfile << std::setprecision(5) << tiempo << "\t"   << tr[1]  << "\t" << tr[5] 
		<< "\t" << tr[6] << "\t" << tr[7]  <<  "\n";
    
      	//##########################Lectura de datos para el mensaje tipo: $DVPDL###################
      	ros::spinOnce();
      	loop_rate.sleep();
      	tiempo = tiempo + T;
      	
  	} //FIN DEL WHILE
  
  delete H;
  return 0;
  //myfile << "\n";
  myfile.close();
  myfile2.close();
} //FIN DEL MAIN



float satura(float a)
{
  if (a >1650)
  {
     a = 1650;
	 return a;
  }
  else if (a < 1350)
  {
	 a = 1350;
     return a;
  }
  return a;
}

//limita yaw
float saturayaw(float c)
{
  if (c >1550)
  {
     c = 1550;
	 return c;
  }
  else if (c < 1450)
  {
	 c = 1450;
     return c;
  }
  return c;
}


//limita yaw
float saturob(float c)
{
  if (c >100)
  {
     c = 100;
	 return c;
  }
  else if (c < -100)
  {
	 c = -100;
     return c;
  }
  return c;
}
//Fcn pid saturado
int satK(float h, float b, float k, float m)
{
	int out;
	float d;

	d=b/k;

	if(abs(h) > d)
	{
		out = b*(pow(abs(h),(m-1)));
		return out;
	}
	else if(abs(h) <=d)
	{
		out = b*(pow(abs(d),(m-1)));
		return out;
	}
	return out;
}

//Fcn Signo
int sgn(float in)
{
  int s;

  if (in > 0)
  {
     s = 1;
	 return s;
  }
  else if (in < 0)
  {
	 s = -1;
     return s;
  }
  return s;

}