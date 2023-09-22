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
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/BatteryState.h"						//variable para bateria
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"
#include "tf/tf.h"
#include "tf/tfMessage.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#define PI 3.14159265358979323846
#define g 9.80665   //Aceleracion de la gravedad

using namespace std;

//################# Variables Globales #################
string tt, dvl1, dvl2, dvl3, dato_anterior;
int i = 0, pos4=0, pos1 = 0, pos2 = 0, pos3 =0, posf, d1=0, d2=0;
float tiempo = 0, T = 0.05;
double tr[8];
double temp1=0, temp2=0;
int B1=0;
int B2=0;
string gr;
ofstream myfile;
ofstream myfile2; 
double posaw, posrollrad, pospitchrad, posyawrad, posroll, pospitch, posyaw;
float posx=0, posy=0, posz, posax, posay, posaz, posroll_p, pospitch_p, posyaw_p;
double velx,vely,velz,veax,veay,veaz;
double bat;
float yawDes, yawDes_p;
float chVal=1500,b=100;
float vnort, vest;
float Tf, pfx, pfy; 
float poss1=0, poss2=0;
float xd, xdp, xdpp, yd, ydp, ydpp, xdp_ant, ydp_ant; //Referencias para trayectoria cuadrada

double Xdeseada=0, Ydeseada=0, Xactual=0, Yactual=0, Yawdeseada=0, Yawactual=0;
//float xd=3, xdp=0, xdpp=0, yd=3, ydp=0, ydpp=0;  //Referencias para trayectoria en x-y, y circular
float ux_I, uy_I;
float ux_B, uy_B;
float Ux, Uy, Uz, Uyaw, sigmaX, sigmaY, sigmaZ, sigmaYaw, sigmaRoll, sigmaPitch;
float exx=0, exx_p=0, eyy=0, eyy_p=0, eyaw=0, eyaw_p=0;
float alfa_x, gamma1_x, gamma2_x, K1_x, K2_x, alfa_y, gamma1_y, gamma2_y, K1_y, K2_y;
float alfa_yaw, gamma1_yaw, gamma2_yaw, K1_yaw, K2_yaw;
float posxx=0, posyy=0;
float funcion=0, funcion2=0;
float funcion_ant=0, funcion_ant2=0;
float dfuncion=0, dfuncion2=0;
int16_t uposx=0;
int16_t xposxx=0, yposy=0, uxx=0, usxx=0, uyy=0, usyy=0;
//int32_t ux_B=0, uy_B=0;
float kpz=2.50, kiz=0.00, kdz=1.0;  
float Kp_x, Ki_x, Kd_x, Kp_y, Ki_y, Kd_y;
//////Ganancias control robusto  z
float z1z=0.0, z2z=0.0, uz_rob=0.0;
double ctrl_z=0;
//float gN= 10.0, g_roll=1.0;	//solo roll
float gNz= 1.0, g_z=0.30;	//roll y z

float kpz2=9.0, kiz2=0.50, kdz2=3.0;	//Ganancias z PD Saturado
uint16_t uz=0, usz=0;
uint16_t ux=0, usx=0;
uint16_t uy=0, usy=0;


double zz=0;
bool fl1=0, fl2=0;

float kpx=100.0, kix=5.0, kdx=5.0;	//Ganancias x
float kpx2=100.0, kix2=5.0, kdx2=5.0;	//Ganancias x PD Saturado
int16_t xx=0, uxf=0;

float kpy=100.0, kiy=5.0, kdy=5.0;	//Ganancias y
int16_t yy=0, uyf=0;

float kpyaw , kiyaw , kdyaw ;	//Ganancias yaw
float kpyaw2=1.0, kiyaw2=1.0, kdyaw2=1.0;	//Ganancias yaw PD Saturado
int32_t uyaw=0, yyaw=0, usyaw=0;

float kproll=0.80, kdroll=0.2;	//ganancias para PD robusto con  referencia fija con controlador en z

float kproll2=8.0, kiroll2=1.0, kdroll2=1.40;	//Ganancias roll PD Saturado
uint16_t uroll=0, usroll=0;
double  rroll=0;

//float kppitch=1.0, kipitch=1.0, kdpitch=1.0;	//Ganancias pitch
float kppitch=3.0, kipitch=1.0, kdpitch=1.0;	//Ganancias pitch  PD

float kppitch2=1.0, kipitch2=1.0, kdpitch2=1.0;	//Ganancias pitch PD Saturado
uint16_t upitch=0, ppitch=0, uspitch=0;

//ganancias saturaciones
float b_z = 1, b_yaw = 1;	//Cota de saturacion

int paro=0,  bajar=0,  armar=0,   control_pos= 0,   desarmar=0, controles=0;
int forwardmove=0, lateralmove=0;
int axis_frwd, axis_latr, axis_vrtl, axis_yaw, axis_roll, axis_pitch;
int satkpz=0, satkiz=0, satkdz=0;
int satkproll=0, satkiroll=0, satkdroll=0;
int satkppitch=0, satkipitch=0, satkdpitch=0;
int satkpyaw=0, satkiyaw=0, satkdyaw=0, satkpx=0, satkix=0, satkdx=0, satkpy=0, satkiy=0, satkdy=0;
int satxx =0, satyy=0;

//Ganancias control robusto
float z1roll=0.0, z2roll=0.0, uroll_rob=0.0;
double ctrl_roll=0;
//float gN= 10.0, g_roll=1.0;	//solo roll
float gN= 10.0, g_roll=1.8;	//roll y z
int Tp;

float satura(float a);  //Definicion de funcion
float saturayaw(float c);  //Definicion de funcion
float saturob(float c);
float saturaposxx(float c);
float saturaposyy(float c);
int satK(float h, float b, float k, float m);
int sgn(float in);
int Sat(float S, float varsigma);
double offset(double d);

//Variables para trayectoria
int t=0, us=0, signo=0, elem=0, r=0;
float tiempo2=0, tiempo3=0, paso=0.05, valor=0;
float tfinal=60,  z_filter_ant=0, t_eje=75;
float roll_filter_ant=0, pitch_tray_ant=0, pitch_filter_ant=0;
float t1=0.0;
float valsin=0, intsin=0;
int chValz=1480;
time_t currentTime = time(0);

float chi_yaw=0, chi_p_yaw=0, chi_pp_yaw=0;
float chi_z=0, chi_p_z=0, chi_pp_z=0;
float chi_x=0, chi_p_x=0, chi_pp_x=0;
float chi_y=0, chi_p_y=0, chi_pp_y=0;
float aux1_yaw=0, aux1_z=0, aux1_x=0, aux1_y=0;
float j_yaw=0, j_z=0, j_x=0, j_y=0;

////////////////////////////////  yaw
float varphi_yaw=0, delta_sup_yaw = 1, delta_inf_yaw = 1, rho_yaw=0, z1_p_yaw=0;
double z1_yaw=0;
float	mu_yaw = 0, eta_yaw = 0, varphi_p_yaw = 0, rho_p_yaw = 0, alpha_yaw = 0, beta_yaw=0; 

////////////////////////////////  z
float varphi_z=0, delta_sup_z = 1, delta_inf_z = 1, rho_z=0, z1_p_z=0, x1_p_x=0, y1_p_y=0;
double z1_z=0, x1_x=0, y1_y=0;
float	mu_z = 0, eta_z = 0, varphi_p_z = 0, rho_p_z = 0, alpha_z = 0, beta_z=0; 

////////////////////////////////  x
float varphi_x=0, delta_sup_x = 1, delta_inf_x = 1, rho_x=0, z1_p_x=0;
double z1_x=0;
float	mu_x = 0, eta_x = 0, varphi_p_x = 0, rho_p_x = 0, alpha_x = 0, beta_x=0; 

////////////////////////////////  y
float varphi_y=0, delta_sup_y = 1, delta_inf_y = 1, rho_y=0, z1_p_y=0;
double z1_y=0;
float	mu_y = 0, eta_y = 0, varphi_p_y = 0, rho_p_y = 0, alpha_y = 0, beta_y=0; 


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
std_msgs::UInt16 xxc;
std_msgs::Int16 u_F;
std_msgs::Int16 u_L;
std_msgs::UInt16 yyc;
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

	posroll_p=msg->angular_velocity.x;
	pospitch_p=msg->angular_velocity.y;
	posyaw_p=msg->angular_velocity.z;


  	veax=msg->angular_velocity.x;

	tf::Quaternion q(posax, posay, posaz, posaw);
  	tf::Matrix3x3 m(q);
  	m.getRPY(posrollrad,pospitchrad,posyawrad);
	posroll=posrollrad*(180/PI);
	pospitch=-1*(pospitchrad*(180/PI));

	//ROS_INFO( "posaw%.3f,", msg->orientation.x);
	posyaw=(posyawrad*(180/PI));
   	if (posyaw > 0)
	{
      posyaw = posyaw - 360;
   	};
}

void odomCallback(const nav_msgs::Odometry::ConstPtr & msg) 
{
    // Extract position information from the Odometry message
    posx = msg->pose.pose.position.x;
    posy = msg->pose.pose.position.y;
    posz = msg->pose.pose.position.z;
}


//Datos DVL
void chatterCallback(const std_msgs::String::ConstPtr & msg)
{
  dato = msg->data.c_str();
  
  if (dato.compare(dato_anterior) != 0)
    {
		if(d2 == 1)
		{
			dvl1 = dvl1.append(dato);
			d1 = 1;
			d2 = 0;			
      	}
		else
		{

      	pos1 = dato.find("$DVPDL");
      
      	if (pos1 > 0 )
      	{
			gr = dato;
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
			string resultant;
        	resultant = to_string(tiempo);
			tt = resultant + "," + dvl1;
		
			myfile2<< std::setprecision(5) << tt << "\t" << dvl1 <<"\n";

			pos4 = dvl1.find("DVPDL")+6;
			for (i=0; i < 8; i++)
            {
				posf = dvl1.find(',', pos4);
    			try 
				{
					tr[i] = stof(dvl1.substr(pos4, posf-pos4));
    			} catch (const invalid_argument& e) 
				{
        			cerr << "Error: " << e.what() << endl;
    			}
                pos4 = posf+1;   
        
            }
			d1=0;

		}
		dato_anterior = dato;
         	              
    }     //FIN DEL DVL 
  
}


//################# CICLO PRINCIPAL #################
int main(int argc, char **argv)
{
	ros::init(argc, argv, "dvl_subscriber");
  	ros::NodeHandle n;
  	ros::NodeHandle nh;
  	ros::Subscriber sub = n.subscribe("dvl_out", 10, chatterCallback);


	ros::init(argc, argv, "publisher");
    ros::Publisher number1_pub = nh.advertise<std_msgs::Float32>("Xdeseada", 10);
	ros::Publisher number2_pub = nh.advertise<std_msgs::Float32>("Ydeseada", 10);
	ros::Publisher number3_pub = nh.advertise<std_msgs::Float32>("Yawdeseada", 10);
	ros::Publisher number4_pub = nh.advertise<std_msgs::Float32>("Xactual", 10);
	ros::Publisher number5_pub = nh.advertise<std_msgs::Float32>("Yactual", 10);
	ros::Publisher number6_pub = nh.advertise<std_msgs::Float32>("Yawactual", 10);

	tm* currentDate = localtime(&currentTime);
  	ros::init(argc, argv, "octosub_node");
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
	ros::Subscriber odomPos;
  	//################# Subscriptores #################
  	subPos  = nh.subscribe("/BlueRov2/imu/data",1, posCallback);  //Topico real
	odomPos  = nh.subscribe("/BlueRov2/odometry",1, odomCallback);  //Topico real

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
	ros::Rate loop_rate(20);

  	
	while (ros::ok())
  	{	
		//################# Publicar datos de la IMU ##############
		Xdeseada = xd; 
		std_msgs::Float32 msg;
        msg.data = Xdeseada;
        number1_pub.publish(msg);
		
		Ydeseada = yd; 
        msg.data = Ydeseada;
        number2_pub.publish(msg);

		Yawdeseada = yawDes; 
        msg.data = Yawdeseada;
        number3_pub.publish(msg);

		Xactual = posxx; 
        msg.data = Xactual;
        number4_pub.publish(msg);
		
		Yactual = posyy; 
        msg.data = Yactual;
        number5_pub.publish(msg);

		Yawactual = posyaw; 
        msg.data = Yawactual;
        number6_pub.publish(msg);

        //ROS_INFO("Number_X: %.2f", number1);
		//ROS_INFO("Number_Y: %.2f", number2);
		
		//##################### Calcular posx and posy #####################
		
		if(temp1 != tr[5] )
			posxx = posxx + tr[5];
		
		if(temp2 != tr[6] )
        	posyy = posyy + tr[6];	
		
		temp1 = tr[5];
		temp2 = tr[6];	

		//ROS_INFO("Number_X: %.8f", tr[5]);
		//&& abs(tr[6]) < 0.1
		funcion = posxx;
		//funcion = posx;				//comentar si se hacen pruebas reales con el DVL
    	dfuncion = (funcion - funcion_ant)/(0.05);
		funcion_ant = funcion;
		vnort = dfuncion;

		funcion2 = posyy;
		//funcion2 = posy;			//comentar si se hacen pruebas reales con el DVL
    	dfuncion2 = (funcion2 - funcion_ant2)/(0.05);
		funcion_ant2 = funcion2;
		vest = dfuncion2;

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

		if(armar==true)	//boton a
    	{
			printf("Armado\n");
			armState.data=true;
	  	  	pubArm.publish(armState);
		}

    	if(desarmar==true)	//boton y
    	{
			printf("Desarmado\n");
			armState.data=false;
	    	pubArm.publish(armState);
    	}

		if(paro==true)	//boton x
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
		if(forwardmove==true)	//boton b5
		{
			//controles=true;
			posxx=0;
			posyy=0;
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
	  	if(control_pos==true)	//boton b
	  	{	
			printf("Control\n");
			// myfile <<  " Control \n";
			controles=true;
			//posxx = 0;
			//posyy = 0;
			//actualiza referncia yaw
			//yawDes=posyaw;
	  	}
		//################# CICLO DEL CONTROLADOR #################
	  	if(controles == true)
	  	{
			//pubVertical.publish(vertical);
	    	//pubLateral.publish(lateral);
	    	//pubForward.publish(fforward);
	    	//pubYaw.publish(yaw);
		  	//pubRoll.publish(roll);
		  	//pubPitch.publish(pitch);
			tiempo2=tiempo2+0.05;	//tiempo
			
			//Tf = 60;  	// Tiempo de trayectoria para el circulo y lineal
			Tf = 60;	// Tiempo de trayectoria cuadrada
			
			pfx = 3;	//Trayectoria lineal
			//pfy = 0;		//Trayectoria lineal
			//pfx = -4;
			//pfy = 4;
			// TRAYECTORIA CUADRADA
			/*if (tiempo2 < Tf){
				xd = 0 + pfx*( 10*pow(tiempo2/Tf,3) - 15*pow(tiempo2/Tf,4) + 6*pow(tiempo2/Tf,5) );
				xdp = pfx*( 30*(pow(tiempo2,2)/pow(Tf,3)) - 60*(pow(tiempo2,3)/pow(Tf,4)) + 30*(pow(tiempo2,4)/pow(Tf,5)) ); 	
				xdpp = pfx*( 60*(tiempo2)/pow(Tf,3) - 180*(pow(tiempo2,3)/pow(Tf,4)) + 120*(pow(tiempo2,4)/pow(Tf,5)) );
				yd = 0;
				ydp = 0;
			}
			if (tiempo2 >= Tf && tiempo2 < Tf*2){
				xd = -4;
				xdp = 0; 
				yd = 0 + pfy*( 10*pow((tiempo2-Tf)/Tf,3) - 15*pow((tiempo2-Tf)/Tf,4) + 6*pow((tiempo2-Tf)/Tf,5) );
				ydp = pfy*( 30*(pow((tiempo2-Tf),2)/pow(Tf,3)) - 60*(pow((tiempo2-Tf),3)/pow(Tf,4)) + 30*(pow((tiempo2-Tf),4)/pow(Tf,5)) ); 	
				ydpp = pfy*( 60*(tiempo2-Tf)/pow(Tf,3) - 180*(pow((tiempo2-Tf),2)/pow(Tf,4)) + 120*(pow((tiempo2-Tf),3)/pow(Tf,5)) );
			
			}
			if (tiempo2 >= Tf*2 && tiempo2 < Tf*3){
				pfx = 4;
				xd = -4 + pfx*( 10*pow((tiempo2-Tf*2)/Tf,3) - 15*pow((tiempo2-Tf*2)/Tf,4) + 6*pow((tiempo2-Tf*2)/Tf,5) );
				xdp = pfx*( 30*(pow((tiempo2-Tf*2),2)/pow(Tf,3)) - 60*(pow((tiempo2-Tf*2),3)/pow(Tf,4)) + 30*(pow((tiempo2-Tf*2),4)/pow(Tf,5)) ); 	 	
				xdpp = pfx*( 60*(tiempo2-Tf*2)/pow(Tf,3) - 180*(pow((tiempo2-Tf*2),2)/pow(Tf,4)) + 120*(pow((tiempo2-Tf*2),3)/pow(Tf,5)) );
				yd = pfy;
				ydp = 0;
			}
			if (tiempo2 >= Tf*3 && tiempo2 < Tf*4){
				xd = 0;
				xdp = 0; 
				yd = pfy - pfy*( 10*pow((tiempo2-Tf*3)/Tf,3) - 15*pow((tiempo2-Tf*3)/Tf,4) + 6*pow((tiempo2-Tf*3)/Tf,5) );
				ydp = pfy*( 30*(pow((tiempo2-Tf*3),2)/pow(Tf,3)) - 60*(pow((tiempo2-Tf*3),3)/pow(Tf,4)) + 30*(pow((tiempo2-Tf*3),4)/pow(Tf,5)) ); 	 	
			 	ydpp = pfy*( 60*(tiempo2-Tf*3)/pow(Tf,3) - 180*(pow((tiempo2-Tf*3),2)/pow(Tf,4)) + 120*(pow((tiempo2-Tf*3),3)/pow(Tf,5)) );	
			
			}
			if (tiempo2 >= Tf*4){
				xd = 0;
				xdp = 0;
				yd = 0;
				ydp = 0; 	
			}*/
			
			// TRAYECTORIA CIRCULAR
			/*
			xd = 5*cos(6.28*tiempo2/Tf); 
			xdp = -5*(6.28/Tf)*sin(6.28*tiempo2/Tf);
			xdpp = -5*(6.28/Tf)*(6.28/Tf)*cos(6.28*tiempo2/Tf);
			//xdp = -(28.26/Tf)*sin(6.28*tiempo2/Tf); 					
			//xdpp = -(28.26/Tf)*(28.26/Tf)*cos(6.28*tiempo2/Tf);
			
			yd = 5*sin(6.28*tiempo2/Tf);
			ydp = 5*(6.28/Tf)*cos(6.28*tiempo2/Tf);
			ydpp = -5*(6.28/Tf)*(6.28/Tf)*sin(6.28*tiempo2/Tf);
			//ydp = (28.26/Tf)*cos(6.28*tiempo2/Tf);
			//ydpp = -(28.26/Tf)*(28.26/Tf)*sin(6.28*tiempo2/Tf);
			*/
			
			// TRAYECTORIA LINEA EN X
			// Iciacializar xd=0 y xdp=0
			/*if (tiempo2 <= Tf)
			{
				xd = pfx*( 10*pow(tiempo2/Tf,3) - 15*pow(tiempo2/Tf,4) + 6*pow(tiempo2/Tf,5) );
				xdp = pfx*( 30*(pow(tiempo2,2)/pow(Tf,3)) - 60*(pow(tiempo2,3)/pow(Tf,4)) + 30*(pow(tiempo2,4)/pow(Tf,5)) ); 	
				//xdpp = pfx*( 60*(tiempo2)/pow(Tf,3) - 180*(pow(tiempo2,3)/pow(Tf,4)) + 120*(pow(tiempo2,4)/pow(Tf,5)) );
			}else 
			{
				xd = pfx;
				xdp = 0;
			}
			yd=0;
			ydp=0;*/
			
			// TRAYECTORIA LINEA EN Y
			// Iciacializar yd=0 y ydp=0
			//if (tiempo2 <= Tf){
			//	yd = pfy*( 10*pow(tiempo2/Tf,3) - 15*pow(tiempo2/Tf,4) + 6*pow(tiempo2/Tf,5) );
			//	ydp = pfy*( 30*(pow(tiempo2,2)/pow(Tf,3)) - 60*(pow(tiempo2,3)/pow(Tf,4)) + 30*(pow(tiempo2,4)/pow(Tf,5)) ); 	
			//	ydpp = pfy*( 60*(tiempo2)/pow(Tf,3) - 180*(pow(tiempo2,3)/pow(Tf,4)) + 120*(pow(tiempo2,4)/pow(Tf,5)) );
			//}else {
			//	yd = pfy;
			//	ydp = 0;
			//}
			//xd=0;
			//xdp=0;
			/*

			int amp=60, perd =60;
			int amp2=10, perd2 =60;
			float a1= (2*PI/perd);
			float a2= (2*PI/perd2);
			
			//ganancias chiyaw
			float gama_yaw=0.01, v_yaw = 0.025, x_inf_yaw = 4;
			
			//ganacias chi z
			float gama_z=0.001, v_z = 0.02, x_inf_z = 4;
			
			//ganacias chi x
			float gama_x=0.001, v_x = 0.02, x_inf_x = 4;

			//ganacias chi y
			float gama_y=0.001, v_y = 0.02, x_inf_y = 4;

			//coseno
			//zDes = 20 + amp2 * cos( a2*tiempo ); 
			//zDes_p = -a2*amp2 *sin( a2*tiempo ); 
			//zDes_pp = - ((a2)*(a2))*amp2 * cos( a2*tiempo ); 
			
			//PPF para x
			chi_x= (1/(tanh(gama_x * tiempo+v_x) )) -1 + x_inf_x ;
			chi_p_x = - gama_x * ((1/sinh(gama_x * tiempo+v_x))*(1/sinh(gama_x * tiempo+v_x))) ;
			chi_pp_x = (2 * (gama_x*gama_x) *((1/sinh(gama_x * tiempo+v_x))*(1/sinh(gama_x * tiempo+v_x))) *  (1/(tanh(gama_x * tiempo+v_x) )));
			
			//PPF para y
			chi_y= (1/(tanh(gama_y * tiempo+v_y) )) -1 + x_inf_y ;
			chi_p_y = - gama_y * ((1/sinh(gama_y * tiempo+v_y))*(1/sinh(gama_y * tiempo+v_y))) ;
			chi_pp_y = (2 * (gama_y*gama_y) *((1/sinh(gama_y * tiempo+v_y))*(1/sinh(gama_y * tiempo+v_y))) *  (1/(tanh(gama_y * tiempo+v_y) )));
			
			//Referencia para yaw
			
			/*yawDes = amp * sin( a1*tiempo ); 
			yawDes_p = a1*amp *cos( a1*tiempo ); 
			yawDes_pp = - ((a1)*(a1))*amp * sin( amp*tiempo ); */
			/*
			//PPF para yaw
			chi_yaw= (1/(tanh(gama_yaw * tiempo+v_yaw) )) -1 + x_inf_yaw ;
			chi_p_yaw = - gama_yaw * ((1/sinh(gama_yaw * tiempo+v_yaw))*(1/sinh(gama_yaw * tiempo+v_yaw))) ;
			chi_pp_yaw = (2 * (gama_yaw*gama_yaw) *((1/sinh(gama_yaw * tiempo+v_yaw))*(1/sinh(gama_yaw * tiempo+v_yaw))) *  (1/(tanh(gama_yaw * tiempo+v_yaw) )));
			

			//################# INICIA EL CONTROLADOR #################
			//######################       PID     ###################
			//##################### Control de Yaw ###################
			kpyaw=4, kiyaw=0.01, kdyaw=0.25;  
			yawDes = -140.0;
			yawDes_p = 0.0;
			
			eyaw =  posyaw - yawDes;  	
			
			if (eyaw > 10)	
				eyaw = 10;
			if (eyaw < -10)
				eyaw = -10;
			
			eyaw_p = posyaw_p - yawDes_p;											

			//eyaw_i = eyaw_i + (eyaw*0.05);
			
			yyaw = -( kpyaw*eyaw + kdyaw*eyaw_p );							
					
			if (yyaw >= 0)
				uyaw=(1510+yyaw);
			else 
				uyaw=(1490+yyaw);
														
			usyaw=saturayaw(uyaw);	//Satura la señal de pwm entre 1100 a 1900
			
			yawc.data=usyaw; 
			eyaw_ant = eyaw;	// guarda valores anteriorees
			
			////////###############################################################################################################		
			////////////////////////////////////////Control de X
		    xd = 3;
			xdp = 0;
			xdpp = 0;

			exx = ( posxx - xd );

			/*int ox=1;
			if (exx > ox)    
			    exx = ox;
			if (exx < -ox)
			    exx = -ox;  */                            
			/*
			exx_p = ( vnort - xdp );
			exx_i = exx_i + (exx*0.05);  

			velx=0;  //velocidad lineal x
			
			//%%%%%%%%%%% performance prescribed
			varphi_x = exx/chi_x;
			
			aux1_x =  1/(varphi_x+delta_inf_x) - 1 / (varphi_x - delta_sup_x) ;
			
			rho_x = (1 / (2*chi_x) ) * aux1_x;
			
			j_x  = (varphi_x + delta_inf_x)/(delta_sup_x-varphi_x) ;
			
			x1_x = log( j_x)/2;
			
			x1_p_x = rho_x * (exx_p - (exx * chi_p_x) /chi_x);
			
			mu_x = -(2*chi_p_x)/(2*chi_x);
			
			eta_x = 1/ (2*chi_x);
			
			varphi_p_x = (exx_p * chi_x - exx * chi_p_x ) / (chi_x*chi_x);
			
			rho_p_x = (0.5* varphi_p_x / chi_x) * ( 1/((varphi_x - delta_sup_x)*(varphi_x - delta_sup_x)) - 1/ ((varphi_x + delta_inf_x)*(varphi_x + delta_inf_x)) ) - (chi_p_x/(chi_x*chi_x)) * aux1_x;
					
			alpha_x = rho_p_x * ( exx_p - (exx * chi_p_x )/chi_x );
			
			beta_x = (-xdpp - exx_p * chi_p_x)/chi_x - (exx * chi_pp_x*chi_x)/(chi_x*chi_x) + exx * (chi_p_x*chi_p_x)/(chi_x*chi_x);

		
			//smc ganancias  superficie
			float s_x=0, delta_x=1, k1_x=0, k2_x=0;
			float alpha1_x=0.65, beta1_x=0.55, m1_x=1.1, m2_x=0.75 ;  //Real ppcfft
										
			k1_x= ((m1_x+1)/2)+((m1_x-1)/2)*sgn(abs(posxx)-1);
			k2_x= ((m2_x+1)/2)+((1-m2_x)/2)*sgn(abs(posxx)-1);
			
			//s = x1_p + delta * x1;
			s_x = x1_p_x + alpha1_x* pow(abs(x1_x),k1_x)*sgn(x1_x) + beta1_x*pow(abs(x1_x),k2_x)*sgn(x1_x);
			
			//smc ganancias
			float   alpha2_x=0.6, beta2_x=0.5, gamma1_x=0, gamma2_x=0, k3_x = 0.5; //real ppcfft funciono bien 18_may_23 ref filtrada
			float n1_x =1.1, n2_x =0.75;
			
			gamma1_x= ((n1_x+1)/2)+((n1_x-1)/2)*sgn(abs(s_x)-1);
			gamma2_x= ((n2_x+1)/2)+((1-n2_x)/2)*sgn(abs(s_x)-1);
			
			xx = ( alpha_x- xdpp + alpha1_x*k1_x*pow(abs(x1_x),(k1_x-1))*velx + beta1_x*k2_x*pow(abs(x1_x),(k2_x-1))*velx + alpha2_x*pow(abs(s_x),gamma1_x)*sgn(s_x) + beta2_x*pow(abs(s_x),gamma2_x)*sgn(s_x)+k3_x*sgn(s_x))/rho_x + beta_x;							//PPC-FFNTSM
			
			////////###############################################################################################################		
			////////////////////////////////////////Control de Y
		    
			yd = 0;
			ydp = 0;
			ydpp = 0;

			eyy = ( posyy - yd );
			
			/*int oy=1;
			if (eyy > oy)    
			    eyy = oy;
			if (eyy < -oy)
			    eyy = -oy;*/                              
			/*
			eyy_p = ( vest - ydp );
			eyy_i = eyy_i + (eyy*0.05);  

			vely=0;  //velocidad lineal y
			
			//%%%%%%%%%%% performance prescribed
			varphi_y = eyy/chi_y;
			
			aux1_y =  1/(varphi_y+delta_inf_y) - 1 / (varphi_y - delta_sup_y) ;
			
			rho_y = (1 / (2*chi_y) ) * aux1_y;
			
			j_y  = (varphi_y + delta_inf_y)/(delta_sup_y-varphi_y) ;
			
			y1_y = log( j_y)/2;
			
			y1_p_y = rho_y * (eyy_p - (eyy * chi_p_y) /chi_y);
			
			mu_y = -(2*chi_p_y)/(2*chi_y);
			
			eta_y = 1/ (2*chi_y);
			
			varphi_p_y = (eyy_p * chi_y - eyy * chi_p_y ) / (chi_y*chi_y);
			
			rho_p_y = (0.5* varphi_p_y / chi_y) * ( 1/((varphi_y - delta_sup_y)*(varphi_y - delta_sup_y)) - 1/ ((varphi_y + delta_inf_y)*(varphi_y + delta_inf_y)) ) - (chi_p_y/(chi_y*chi_y)) * aux1_y;
					
			alpha_y = rho_p_y * ( eyy_p - (eyy * chi_p_y )/chi_y );
			
			beta_y = (-ydpp - eyy_p * chi_p_y)/chi_y - (eyy * chi_pp_y*chi_y)/(chi_y*chi_y) + eyy * (chi_p_y*chi_p_y)/(chi_y*chi_y);

		
			//smc ganancias  superficie
			float s_y=0, delta_y=1, k1_y=0, k2_y=0;
			float alpha1_y=0.65, beta1_y=0.55, m1_y=1.1, m2_y=0.75 ;  //Real ppcfft
										
			k1_y= ((m1_y+1)/2)+((m1_y-1)/2)*sgn(abs(posyy)-1);
			k2_y= ((m2_y+1)/2)+((1-m2_y)/2)*sgn(abs(posyy)-1);
					

			s_y = y1_p_y + alpha1_y* pow(abs(y1_y),k1_y)*sgn(y1_y) + beta1_y*pow(abs(y1_y),k2_y)*sgn(y1_y);
			
			//smc ganancias
			float   alpha2_y=0.6, beta2_y=0.50, gamma1_y=0, gamma2_y=0, k3_y = 0.3; //real ppcfft funciono bien 18_may_23 ref filtrada
			float n1_y =1.1, n2_y =0.75;
			
			gamma1_y= ((n1_y+1)/2)+((n1_y-1)/2)*sgn(abs(s_y)-1);
			gamma2_y= ((n2_y+1)/2)+((1-n2_y)/2)*sgn(abs(s_y)-1);
			
			yy =-( alpha_y- ydpp + alpha1_y*k1_y*pow(abs(y1_y),(k1_y-1))*vely + beta1_y*k2_y*pow(abs(y1_y),(k2_y-1))*vely + alpha2_y*pow(abs(s_y),gamma1_y)*sgn(s_y) + beta2_y*pow(abs(s_y),gamma2_y)*sgn(s_y)+k3_y*sgn(s_y))/rho_y + beta_y;							//PPC-FFNTSM

			//######################### Matriz de rotación ###################
			ux_I = xx*cos(posyaw*3.141592/180) + yy*sin(posyaw*3.141592/180);
			
			uy_I = -xx*sin(posyaw*3.141592/180) + yy*cos(posyaw*3.141592/180);
			
			//######################### Saturaciones ###################
			ux=offset(ux_I);     													//aplica offset de  motores en la señal del controlador con u negativa

			usx=satura((int)ux);											    //Satura la señal de pwm entre 1400 a 1600

			uy=offset(uy_I);     													//aplica offset de  motores en la señal del controlador con u negativa

			usy=satura((int)uy);
		
			fforwardc.data=usx; //cambiar a usx
			lateralc.data=usy; //cambiar a usy;

			pubForward.publish(fforwardc);
			//pubLateral.publish(lateralc);
			pubYaw.publish(yawc);*/
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


		ROS_INFO_STREAM("DatosPos_X: " << tiempo2  << "\t" << exx  << "\t" << xx << "\t" << posxx << "\t" << tr[5] << endl);
		//ROS_INFO_STREAM("DatosPos_Y: " << tiempo2  << "\t" << eyy  << "\t" << yy << "\t" << posyy << "\t" << tr[6] << endl);
		
		//ROS_INFO_STREAM("DatosPos_Yaw: " << tiempo2  << "\t" << eyaw  << "\t" << yyaw << "\t" << usyaw << "\t" << posyaw << endl);
		
		
		myfile << std::setprecision(5) << tiempo2 << "\t" << xd << "\t" << yd << "\t" << yawDes << "\t" << posxx << "\t" << posyy << "\t" << posyaw << "\t" << ux_B << "\t" << uy_B << "\t" << usxx << "\t" << usyy << "\n";	

		ros::spinOnce();
      	loop_rate.sleep();
      	tiempo = tiempo + T;
      	
  	} //FIN DEL WHILE
	
  delete H;
  return 0;
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
  if (c >1900)
  {
     c = 1900;
	 return c;
  }
  else if (c < 1100)
  {
	 c = 1100;
     return c;
  }
  return c;
}
//limita posicion en -> x
float saturaposxx(float c)
{
  if (c >1900)
  {
	c = 1900;
	return c;
  }
  else if (c < 1100)
  {
	c = 1100;
    return c;
  }
  return c;
}
//limita posicion en -> x
float saturaposyy(float c)
{
  if (c >1900)
  {
	c = 1900;
	return c;
  }
  else if (c < 1100)
  {
	c = 1100;
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
	float d=b/k;

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

// Sat function --> sgn function

int Sat(float S, float varsigma)
{
  int out;

  if (S < -varsigma)
  {
     out = -1;
	 return out;
  }
  else if (-varsigma <= S <= varsigma)
  {
	 out = S/varsigma;
     return out;
  }
  else if(S > varsigma )
  {
	out = 1;
	return out;
  }
  return out;

}

double offset(double d)
{
double out;
  if(d >= 0)
  {
     out = ((double)1518+d);
	 return out;
  }
  else if(d  < 0)
  {    
	 out = ((double)1482+d);
     return out;
  }
}