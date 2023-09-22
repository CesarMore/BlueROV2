/// version del codigo para probar con vehiculo fisicio ultima version implementada en octosub
#include "CJoystick.h"
#include "CJoystick.cpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/FluidPressure.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Bool.h"
#include "tf/tf.h"
#include "tf/tfMessage.h"
#define PI 3.14159265358979323846
#define Rf 997.0474   //Rho agua fresca densidad 
#define Rs 1023.6   //Rho agua salada densidad 
#define g 9.80665   //Aceleracion de la gravedad
#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>

#include <stdio.h>
#include <ctime>
using namespace std;


//################# Variables Globales #################
double fluid_press, diff_press,z_baro;
double velx,vely,velz,veax,veay,veaz;
double posx, posy, posz, posax, posay, posaz, posaw, posrollrad, pospitchrad, posyawrad, posroll, pospitch, posyaw;
float zDes=25.0, xDes=1.0, yDes=1.0, yawDes=0.0;  				//Referencias


float ez=0, ex=0,  ey=0, eyaw=0;												//Errores
float ex_ant=0,ey_ant=0, ez_ant=0, eyaw_ant=0;	        			//Valores anteriores
float ex_p=0, ey_p=0, ez_p=0, eyaw_p=0;						//Derivadas
float chVal=1500,b=100;


float kpz=8.5, kiz=5.0, kdz=0.2;  											//Ganancias z

uint16_t uz=0, zz=0, uzf=0;
float kpx=100.0, kix=5.0, kdx=5.0;  											//Ganancias x
uint16_t ux=0, xx=0, uxf=0;
float kpy=100.0, kiy=5.0, kdy=5.0; 											//Ganancias y
uint16_t uy=0, yy=0, uyf=0;

float kpyaw=0.20, kiyaw=5.0, kdyaw=0.150; 									//Ganancias yaw
uint16_t uyaw=0, ywyw=0;
int paro=0,  bajar=0,  armar=0,   control_pos= 0,   desarmar=0, controles=0;

//ganancias saturaciones
float b_z = 1, b_yaw = 1;             											//Cota de saturacion
float mu_z=0.5, mu_yaw = 0.5;  												//Mu entre 0 y 1

int axis_frwd, axis_latr, axis_vrtl, axis_yaw;
int satkpz=0, satkiz=0, satkdz=0;



std_msgs::Bool armState;
std_msgs::String modo;
std::stringstream motxt;
std::stringstream filenameDate;

std_msgs::UInt16 vertical;
std_msgs::UInt16 lateral;
std_msgs::UInt16 fforward;
std_msgs::UInt16 yaw;

std_msgs::UInt16 verticalc;
std_msgs::UInt16 lateralc;
std_msgs::UInt16 fforwardc;
std_msgs::UInt16 yawc;

//################# Funciones Callback #################
//Orientación vehiculo real       				%%%%%%%%%%%
void posCallback(const sensor_msgs::Imu::ConstPtr & msg) {
     posax=msg->orientation.x;
	 posay=msg->orientation.y;
	 posaz=msg->orientation.z;
	 posaw=msg->orientation.w;
	 
	 tf::Quaternion q(posax, posay, posaz, posaw);
     tf::Matrix3x3 m(q);
     m.getRPY(posrollrad,pospitchrad,posyawrad);
	 
	// ROS_INFO( "posaw%.3f,", msg->orientation.x);
	 
	 posyaw=posyawrad*(180/PI);
}
//Presión
void presCallback(const sensor_msgs::FluidPressure::ConstPtr & msg) {
    //ROS_INFO( "Press: %.3f,Diff_press:%.3f,", msg->fluid_pressure, msg->variance);
     fluid_press=msg->fluid_pressure;
	 diff_press=msg->variance;
	 
	 //z_baro = (fluid_press/(Rf*g)*100-7.39);
	 z_baro=fluid_press;

}

float satura(float a);  

int satK(float h, float b, float k, float m);  

//################# CICLO PRINCIPAL #################

int main(int argc, char **argv)
{
	//Variables para trayectoria
	int m=0, division=0, t=0, usz=0, usyaw=0;
	float tiempo=0, paso=0.02;
	float Tfinal=10, step=0;
	m= Tfinal/paso;
	division = 10;
	step = m/division;
	float tt[m], xdes[m] , ydes[m],  zdes[m];
	tt[0] = 0;
	
	float valsin=0, intsin=0;
	
	time_t currentTime = time(0);
	tm* currentDate = localtime(&currentTime);
	
	filenameDate << "/home/umi/catkin_ws/src/codigo/src/Data/BluerovCtrl"<< currentDate->tm_hour<< "_"<<currentDate->tm_min<< "_"<<currentDate->tm_sec<<"-"<< currentDate->tm_mday<<"_"<<currentDate->tm_mon+1<< "_"<<currentDate->tm_year+1900<<".txt";
	
	//Generacion de archivo para guardar variables
	ofstream myfile;
	//myfile.open("/home/salatiel/catkin_ws/src/codigo/src/Data/BlueRovPD.txt");    //ruta del archivo
	myfile.open(filenameDate.str());    //ruta del archivo
	

	ros::init(argc, argv, "octosub_node");
	ros::NodeHandle nh;

	
	//ros::Subscriber subVel;
	ros::Subscriber subPos;     				%%%%%%%%%%%
	ros::Subscriber subPres;

	ros::Publisher pubArm;
	ros::Publisher pubMod;

	//Movimientos
	ros::Publisher pubVertical;
	ros::Publisher pubLateral;
	ros::Publisher pubForward;
	ros::Publisher pubYaw;
	

	CJoystick *H = new CJoystick();

  //################# Subscriptores #################
  
 // subVel  = nh.subscribe("/mavros/local_position/velocity_body",6, velCallback);
  //subPos  = nh.subscribe("/mavros/local_position/pose",10, posCallback);  //Topico para simulacion
  
  subPos  = nh.subscribe("/BlueRov2/imu/data",1, posCallback);  //Topico real       				%%%%%%%%%%%
  subPres  = nh.subscribe("/BlueRov2/pressure",1, presCallback);
  
  pubVertical=nh.advertise<std_msgs::UInt16>("BlueRov2/rc_channel3/set_pwm", 1);
  pubLateral=nh.advertise<std_msgs::UInt16>("BlueRov2/rc_channel6/set_pwm", 1);
  pubForward=nh.advertise<std_msgs::UInt16>("BlueRov2/rc_channel5/set_pwm", 1);
  pubYaw=nh.advertise<std_msgs::UInt16>("BlueRov2/rc_channel4/set_pwm", 1);
  
  
  ros::Subscriber Joystick = nh.subscribe("joy", 1000, &CJoystick::chatterCallback,H);
  
  //################# Publicadores #################
  
  //Armar
  pubArm=nh.advertise<std_msgs::Bool>(nh.resolveName("/BlueRov2/arm"), 1);
  
  //Modo deph hold
  pubMod=nh.advertise<std_msgs::String>(nh.resolveName("/BlueRov2/mode/set"), 1);
  motxt << "manual";
  modo.data=motxt.str();
  
  ros::Rate loop_rate(20);    //Muestras por segundos //Hz  //Con 20 equivale a 50ms
 
  
  //modo y armado
	  /*pubArm.publish(armState);
	  */
	  
  //Referencias desde teclado
    //printf("X: ");
    //scanf("%f", &xDes);
    //printf("Y: ");
    //scanf("%f", &yDes);
	//printf("Z: ");
    //scanf("%f", &zDes);
    //printf("Yaw: ");
    //scanf("%f", &yawDes);
	  
//################# Ciclo de acción #################  
  while (ros::ok())
  {  
	//Botones de joystick
	armar=H->a;
    control_pos= H->b;
    desarmar=H->y;
    paro=H->x;
	
	//Palancas de joystick
	axis_latr = 1500-(H->JYaw*200);
    axis_frwd = 1500+(H ->JGaz*200);
    axis_vrtl = 1500-(H ->JPitch*200);
    axis_yaw = 1500+(H ->JRoll*50);
	
	//Pasa valor de joystick a variables para publicar en topicos de ROS
	vertical.data=axis_vrtl; 
	lateral.data=axis_latr; 
	fforward.data=axis_frwd; 	
	yaw.data=axis_yaw; 	
		
	//Boton de armado A
	if(armar==true) 
    {
		printf("Armado\n");
		armState.data=true;
	    pubArm.publish(armState);
	}
	
	//Boton de desarmado Y
    if(desarmar==true) 
    {
		printf("Desarmado\n");
		armState.data=false;
	    pubArm.publish(armState);
    }
	
	//Boton de paro de emergencia, detiene nodo de ros  X
	if(paro==true)  
    {
		printf("Paro\n");
		controles=false;
		vertical.data=1500; 
		lateral.data=1500; 
		fforward.data=1500; 	
		yaw.data=1500; 	
		
	    pubVertical.publish(vertical);
	    pubLateral.publish(lateral);
	    pubForward.publish(fforward);
	    pubYaw.publish(yaw);
		
		armState.data=false;
	    pubArm.publish(armState);
		
		return 0;
    }
	
	
	//Boton de constrol activa una bandera para el ciclo de control que se ejecuta durante cierto tiempo 
	if(control_pos==true)  //boton b
	{
		printf("Control\n");
		controles=true;
	}
	
	//uz=satura(z_baro);
	
	//################# CICLO DEL CONTROLADOR ################# 
	if(controles == true) 
	{ 
		tiempo=tiempo+0.05;   //Iterador de tiempo
		
				//################# INCIA EL CONTROLADOR ################# 
				//////////////////////////////////////////Control de Z
				ez=z_baro-760;  												//Error
				ez_p=(ez-ez_ant)/0.05;										//Error p
				zz=kpz*ez+kiz*ez_p;											//PD
				
				//satK(h_fcn, b,limite saturador, k_ganancia, m_parametro pendiente); 
				//satkp = satK(tiempo,1,kpz,0.5);
				// satkpz = satK(ez,1,kpz,0.5);
				// satkdz = satK(ez_p,1,kdz,0.5);
				// zz=satkpz*ez+satkdz*ez_p;							
				uz=(chVal+zz);													//Resta o suma al valor central de pwm 1500
				usz=satura(uz);														//Satura la señal de pwm entre 1100 a 1900
				//usz=1500;  															//Desactiva controlador en Z
				
				//////////////////////////////////////////Control de yaw
				eyaw= posyaw-yawDes;  												//Error
				eyaw_p=(eyaw-eyaw_ant)/0.05;										//Error p
				ywyw=kpyaw*eyaw+kdyaw*eyaw_p;											//PD
				
				uyaw=(chVal+ywyw);													//Resta o suma al valor central de pwm 1500
				usyaw=satura(uyaw);														//Satura la señal de pwm entre 1100 a 1900
				//usyaw=1500;  															//Desactiva controlador en yaw
				
				//### integral
				valsin=sin(t);
				
				intsin=intsin+(valsin*paso);				

				// COMENTAR SI SE REALIZAN PRUEBAS FISICAS CON EL VEHICULO FUERA DEL AGUA
				verticalc.data=usz;													//se envia valor saturado
				pubVertical.publish(verticalc);
				
				yawc.data=usyaw;
				pubYaw.publish(yawc);
                			    
//			    ey_der=0;
//			    ez_der=0;
//			    eyaw_der=(eyaw-eyaw_ant)/0.01;
			  
			    //////////////////// GUARDA VARIABLES ANTERIORES ////////////////
//			    ex_ant = ex;
//			    ey_ant = ey;
			    ez_ant = ez;
			    eyaw_ant = eyaw;	
			    
				//VARIABLES A GUARDAR EN ARCHIVO TXT SOLO DURANTE CICLO CONTROL
				myfile<< std::setprecision(5) <<tiempo<<"\t"<<zDes<<"\t"<<z_baro<<"\t"<<ez<<"\t"<<uz<<"\n";	
				//Tiempo ros
				//ros::Time RTime = ros::Time::now();
				//TROS << RTime.sec <<"."<< RTime.nsec;
				//std::stringstream ss; 
				t=t+1;  //incrementa iterador para siguiente punto de trayectoria
                
				
				
	
	}
	else
	{
		// MODO DE OPERACION MANUAL
		//Publicadores de señales de thrusters
		pubVertical.publish(vertical);
		pubLateral.publish(lateral);
		pubForward.publish(fforward);
		pubYaw.publish(yaw);

	}
		
	///////////////////// IMPRESION EN TERMINAL //////////////////	  

	//printf("x = %.3f\ty = %.3f\tz = %.3f\tx = %.3f\ty = %.3f\tz = %.3f\n",velx,vely,velz,veax,veay,veaz);
	//printf("x = %.3f\ty = %.3f\tz = %.3f\ti = %.3f\tj = %.3f\tk = %.3f\tw = %.3f\n",posx, posy, posz, posax, posay, posaz, posaw);
	//printf("############## ERROR SEÑAL ############\n");
	//printf("z_des	= %.3f\tz_real = %.3f\tz_err = %0.3f\tz_u = %d\n",zDes, posz, ez, uz);
	//printf("x_des = %.2f\tx_real = %.2f\tx_err = %0.2f\tx_u = %d\n",xDes, posx, ex, ux);
	//printf("y_des = %.2f\ty_real = %.2f\ty_err = %0.2f\ty_u = %d\n",yDes, posy, ey, uy);
	//printf("yaw_des = %.2f\tyaw_real = %.2f\tyaw_err = %0.2f\tyaw_u = %d\n",yawDes, posyaw, eyaw, uyaw);
	//printf("%0.3f\n",ez);
	//printf("%d\n",uz);
	//printf("%0.5f\tCtr=%d\n",tiempo,controles);
	/*
	printf("############## CONTROL ############");
	printf("x_des = %.3f\tx_real = %.3f\n",xDes, posx);
	printf("y_des = %.3f\ty_real = %.3f\n",yDes, posy);
	printf("yaw_des = %0.3f\tyaw_real = %0.3f\n",yawDes, posyaw);*/
	//printf("t= %0.3f \t z= %0.3f \t e= %0.3f \t uz= %d \t us= %d \t kpz= %d \t kdz= %d \n", ros::Time::now().toSec(),z_baro, ez,uz,usz,satkpz,satkdz);
	printf("yaw= %0.3f \t e= %0.3f \t uyaw= %d \t uyaw= %d \n",posyaw, eyaw,uyaw,usyaw);

	//Profundidad calculada con barometro
	//printf("z_barometro ddd=%.3f\n",z_baro);
	
	//Senales del joystick
	//printf("x_s= %d \t y_s= %d \t z_s= %d \t yaw_s= %d\n",axis_frwd, axis_latr, axis_vrtl, axis_yaw);
	
	//Guarda Variables en archivo de texto todo el tiempo
	//myfile<< std::setprecision(4) <<valsin<<"\t"<<intsin<<"\n";
	
	ros::spinOnce();
	loop_rate.sleep();
  }
  delete H;
  myfile.close();
  return 0;
}

float satura(float a)
{
  if (a >1900)
  {
     a = 1900;
	 return a;
  }
  else if (a < 1100)
  {    
	 a = 1100;
     return a;
  }
  return a;
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