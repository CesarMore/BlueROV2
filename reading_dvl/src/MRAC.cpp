//###########################################################################################
#include "CJoystick.h"
#include "CJoystick.cpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <std_msgs/Float64MultiArray.h>
#include "sensor_msgs/FluidPressure.h"
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
#include <cstdlib> // for atof function
#define PI 3.14159265358979323846
#define g 9.80665   //Aceleracion de la gravedad

using namespace std;

//################# Variables Globales #################
string tt, dvl1, dato_anterior, gr, dato;
ofstream myfile;
ofstream myfile2;
time_t currentTime = time(0);
float tiempo=0, T=0.05, tiempo2=0, tiempo3=0;
float tr[8];
double fluid_press, diff_press, z_baro; 
double atmosphericPressure = 7720.0;  //Presion atmosferica en Pascales 777000.2 
double depth;
float posx=0, posy=0, posz, posax, posay, posaz, posroll_p, pospitch_p, posyaw_p;
float zDes, zDes_p, zDes_pp;
float yawDes, yawDes_p, yawDes_pp;
float ez_ant=0, eyaw_ant=0;
float ez_i=0, eyaw_i=0, exx_i=0, eyy_i=0;	//Derivadas
float vnort, vest, d_z;
float Tf, pfx, pfy; 
float xd=0, xdp=0, xdpp=0, yd=0, ydp=0, ydpp=0; //Referencias para trayectoria cuadrada
float Xdeseada, Ydeseada, Xactual, Yactual, YawDes, YawActual, Zactual, Zdeseada;
float poss1=0, poss2=0;
float ux_I, uy_I;
float ux_B, uy_B;
float exx=0, exx_p=0, eyy=0, eyy_p=0, eyaw=0, eyaw_p=0, ez=0, ez_p=0;
float posxx=0, posyy=0;
float pos_x=0, pos_y=0, vel_x=0, vel_y=0;
float funcion=0, funcion2=0, funcion3=0;
float funcion_ant=0, funcion_ant2=0, funcion_ant3=0;
float dfuncion=0, dfuncion2=0, dfuncion3=0;
float kpz, kiz, kdz;  
float Kp_x, Ki_x, Kd_x, Kp_y, Ki_y, Kd_y;
float zz=0;
float kpyaw , kiyaw , kdyaw ;	//Ganancias yaw
float xr1=0, xr2=0, ex1=0, ex2=0, ex1_p=0, ex2_p=0, wn=1, zeta=0.7;
float yr1=0, yr2=0, ey1=0, ey2=0, ey1_p=0, ey2_p=0;
float zr1=0, zr2=0, ez1=0, ez2=0, ez1_p=0, ez2_p=0;
float yawr1=0, yawr2=0, eyaw1=0, eyaw2=0, eyaw1_p=0, eyaw2_p=0;
float Kyaw_hat1=-2, Kyaw_hat2=-0.5, Kr_hat_yaw=1, Theta_hat1_yaw=2, Theta_hat2_yaw=1;
float Kx_hat1=-2, Kx_hat2=-1, Kr_hat_x=1, Theta_hat1_x=2, Theta_hat2_x=1;
float Ky_hat1=-1, Ky_hat2=-0.5, Kr_hat_y=1, Theta_hat1_y=1, Theta_hat2_y=0.5;
float kz_hat1=-3, kz_hat2 =-1, Kr_hat_z=2, Theta_hat1_z=3, Theta_hat2_z=1;

float satura(float a); 
float saturayaw(float c);
float saturob(float c);
float saturaposxx(float c);
float saturaposyy(float c);
int satK(float h, float b, float k, float m);
int sgn(float in);
int Sat(float S, float varsigma);

double posaw, posrollrad, pospitchrad, posyawrad, posroll, pospitch, posyaw;
double bat, zbf=0.0,zbf_a=0.0, a_z = 0.0;

int satkpz=0, satkiz=0, satkdz=0;
int satkproll=0, satkiroll=0, satkdroll=0;
int satkppitch=0, satkipitch=0, satkdpitch=0;
int satkpyaw=0, satkiyaw=0, satkdyaw=0, satkpx=0, satkix=0, satkdx=0, satkpy=0, satkiy=0, satkdy=0;
int satxx =0, satyy=0;

int paro=0,  bajar=0,  armar=0,   control_pos= 0,   desarmar=0, controles=0;
int forwardmove=0, lateralmove=0;
int axis_frwd, axis_latr, axis_vrtl, axis_yaw, axis_roll, axis_pitch;
int B1=0, B2=0;
int i=0, pos4=0, pos1=0, pos2=0, pos3=0, posf, d1=0, d2=0;

int32_t uyaw=0, yyaw=0, usyaw=0;
uint16_t uz=0, usz=0;
int16_t uxx=0, usxx=0, uyy=0, usyy=0;

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

	tf::Quaternion q(posax, posay, posaz, posaw);
	tf::Matrix3x3 m(q);
    m.getRPY(posrollrad,pospitchrad,posyawrad);
	posroll=posrollrad*(180/PI);
	pospitch=-1*(pospitchrad*(180/PI));

	//ROS_INFO( "posaw%.3f,", msg->orientation.x);
	
	posyaw=(posyawrad*(180/PI));
   	// if (posyaw > 0)
	// {
    //   posyaw = posyaw - 360;
   	// };
}

void odomCallback(const nav_msgs::Odometry::ConstPtr & msg) 
{
    // Extracter posicion del mensaje Odometry
    posx = msg->pose.pose.position.x;
    posy = msg->pose.pose.position.y;
    posz = msg->pose.pose.position.z;

    //ROS_INFO("Position: x=%.3f, y=%.3f, z=%.3f", posx, posy, posz);
	//ROS_INFO_STREAM("Datos: " << posx  << endl);

}

//Presión
void presCallback(const sensor_msgs::FluidPressure::ConstPtr & msg) {

    fluid_press=msg->fluid_pressure;
	diff_press=msg->variance;
	
	depth = (fluid_press - atmosphericPressure) / (997.0 * 9.81);  // Densidad del agua = 997 kg/m^3, 
																   //g = 9.81 m/s^2
	//z_baro = (fluid_press/(Rf*g)*100-7.39);
	z_baro=fluid_press-733.0;

	//z filtrada
	//a_z= 1/(1*PI*RC);  = 0.1061
	a_z = 0.1061;

	zbf = a_z * z_baro + (1-a_z)*zbf_a;
	zbf_a = zbf;
	//printf("Press: %.3f,Altura:%.3f \n", msg->fluid_pressure, zbf);	

	//ROS_INFO( "Press: %.3f,Diff_press:%.3f,", msg->fluid_pressure, msg->variance); 
}

//Datos DVL_DVEXT
void dvextCallback(const std_msgs::String::ConstPtr & msg) 
{
    const char* vel = msg->data.c_str();

    // Extraer el 6th elemento
    char* endPtr;
    vel_x = strtof(vel, &endPtr);
    vel = endPtr;

    // Encontrar la siguiente coma
    vel = strchr(vel, ',');
    if (vel == nullptr) {
        // Elementos no sufiicientes
        return;
    }
    // Mover al siguiente caracter despues de la coma
    ++vel;

    // Extraer el 7th elemento
    vel_y = strtof(vel, &endPtr);

    // Imprimir los elementos extraidos
    //ROS_INFO_STREAM("" << vel_x << "\t" << vel_y << std::endl);
}

// //Datos DVL_DVPDL
// void dvpdlCallback(const std_msgs::String::ConstPtr & msg) 
// {
//     const char* pos = msg->data.c_str();
//     ROS_INFO_STREAM("pos: " << pos << std::endl);
// }
//Datos DVL_DVPDL
void dvpdlCallback(const std_msgs::String::ConstPtr & msg) 
{
    const char* pos = msg->data.c_str();

    // Extraer el 6th elemento
    char* endPtr;
    pos_x = strtof(pos, &endPtr);
    pos = endPtr;

    // Encontrar la siguiente coma
    pos = strchr(pos, ',');
    if (pos == nullptr) {
        // Elementos no sufiicientes
        return;
    }
    // Mover al siguiente caracter despues de la coma
    ++pos;

    // Extraer el 7th elemento
    pos_y = strtof(pos, &endPtr);

    // Imprimir los elementos extraidos
    //ROS_INFO_STREAM("" << pos_x << "\t" << pos_y << std::endl);
}


//Datos DVL
void chatterCallback(const std_msgs::String::ConstPtr & msg)
{
  dato = msg->data.c_str(); // Obtener la información del mensaje

  // Verificar si el mensaje es diferente al mensaje anterior
  if (dato.compare(dato_anterior) != 0)
    {
		if(d2 == 1) // Si d2 es igual a 1, continuar concatenando el mensaje
		{
      		dvl1 = dvl1.append(dato);
			d1 = 1; // Indica que se ha recibido un mensaje completo
			d2 = 0;	// Reiniciar el contador de mensajes
      	}
		else
		{
      	pos1 = dato.find("$DVPDL"); // Buscar la cadena "$DVPDL" en el mensaje
      	if (pos1 > 0 ) // Si la cadena "$DVPDL" se encuentra en el mensaje
      	{
			gr = dato;
        	dvl1 = dato;
			pos2 = dvl1.find("*"); // Buscar la primera aparición del carácter "*"
			pos3 = dvl1.find("*", pos2 + 1); // Buscar la segunda aparición del carácter "*"
			if(pos3 >0 )
			{
				d1 = 1; // Indica que se ha recibido un mensaje completo
			}
			else
				d2 = 1; // Indica que se necesita recibir más mensajes para completar el mensaje actual

      	}
		}
      	if(d1==1) // Si se ha recibido un mensaje completo
		{
			string resultant;
        	resultant = to_string(tiempo); // Convertir el tiempo a una cadena
			tt = resultant + "," + dvl1; // Concatenar el tiempo y el mensaje completo
			
			//ROS_INFO_STREAM("dvl1: " << dvl1 << endl);
			//ROS_INFO_STREAM( "datoA: " << dvl1 << endl);

			myfile2<< std::setprecision(5) << tt << "\t" << dvl1 <<"\n";

			pos4 = dvl1.find("DVPDL")+6; // Obtener la posición donde comienzan los datos del DVL
			for (i=0; i < 8; i++) // Recorrer los 8 datos del DVL
            {
				posf = dvl1.find(',', pos4); // Obtener la posición del siguiente carácter ","
    			try 
				{
					tr[i] = stof(dvl1.substr(pos4, posf-pos4)); // Convertir el valor del dato a un float y almacenarlo en el arreglo "tr"
    			} catch (const invalid_argument& e) 
				{
        			cerr << "Error: " << e.what() << endl; // Si hay un error en la conversión, imprimir el mensaje de error
    			}
                pos4 = posf+1;   // Actualizar la posición para continuar extrayendo el siguiente dato
            }
			d1=0; // Reiniciar la variable "d1" para esperar el siguiente mensaje

			//ROS_INFO_STREAM("Datos: " << tiempo << "\t" << tr[5] << "\t" << tr[6] << endl ) ;
			
			//myfile << std::setprecision(5) << tiempo << "\t"   << tr[1]  << "\t" << tr[5] 
			//<< "\t" << tr[6] << "\t" << tr[7]  << "\t" << B1 << "\t" << B2 <<  "\n";

		}
		dato_anterior = dato; // Actualizar el mensaje anterior
         	              
    }     //FIN DEL DVL 
  
}


//################# CICLO PRINCIPAL #################
int main(int argc, char **argv)
{
	ros::init(argc, argv, "dvl_subscriber");
  	ros::NodeHandle n;
  	ros::NodeHandle nh;
  	ros::Subscriber sub = n.subscribe("dvl_out", 10, chatterCallback);
	ros::init(argc, argv, "pressure_subscriber");
    ros::Subscriber subm = nh.subscribe("/BlueRov2/pressure", 10, presCallback);

	ros::Publisher pub_dvext = nh.advertise<std_msgs::String>("dvext_data", 20);
    ros::Publisher pub_dvpdl = nh.advertise<std_msgs::String>("dvpdl_data", 20);

    ros::Subscriber sub_dvext = nh.subscribe("dvext_data", 20, dvextCallback);
    ros::Subscriber sub_dvpdl = nh.subscribe("dvpdl_data", 20, dvpdlCallback);

	ros::init(argc, argv, "publisher");
    ros::Publisher number1_pub = nh.advertise<std_msgs::Float32>("Xdeseada", 10);
	ros::Publisher number2_pub = nh.advertise<std_msgs::Float32>("Ydeseada", 10);
	ros::Publisher number3_pub = nh.advertise<std_msgs::Float32>("Xactual", 10);
	ros::Publisher number4_pub = nh.advertise<std_msgs::Float32>("Yactual", 10);
	ros::Publisher number5_pub = nh.advertise<std_msgs::Float32>("YawDes", 10);
	ros::Publisher number6_pub = nh.advertise<std_msgs::Float32>("YawActual", 10);
	ros::Publisher number7_pub = nh.advertise<std_msgs::Float32>("Zactual", 10);
	ros::Publisher number8_pub = nh.advertise<std_msgs::Float32>("Zdeseada", 10);

    //ros::Rate loop_rate(1); // Publicar un mensaje por segundo


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


	//Initialize and start the node
    //ros::init(argc, argv, "abc");
     
    //ros::Publisher pub = nh.advertise<std_msgs::Int32>("abc_topic", 1000);
    //Define and create some messages
    //std_msgs::Int32 abc;
    //abc.data = posx;


  	
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

		Xactual = posx; 
        msg.data = Xactual;
        number3_pub.publish(msg);
		
		Yactual = posy; 
        msg.data = Yactual;
        number4_pub.publish(msg);

		YawDes = yawDes; 
        msg.data = YawDes;
        number5_pub.publish(msg);
		
		YawActual = posyaw; 
        msg.data = YawActual;
        number6_pub.publish(msg);

		Zactual = zbf_a;
		msg.data = Zactual;
		number7_pub.publish(msg);

		Zdeseada = zDes;
		msg.data = Zdeseada;
		number8_pub.publish(msg);

        //ROS_INFO("Number_X: %.2f", number1);
		//ROS_INFO("Number_Y: %.2f", number2);
		
		//##################### Calcular pos1 and pos2 #####################
		if (abs(pos_x) > 0.1)
		{
			poss1 = posxx;
		}
		else 
			posxx = posxx + pos_x;
		
		if (abs(pos_y) > 0.1)
		{
			poss2 = posyy;
		}
		else
        	posyy = posyy + pos_y;	
		// if(temp1 != pos_x )
		// 	posxx = posxx + pos_x;
		
		// if(temp2 != pos_y )
        // 	posyy = posyy + pos_y;	
		
		// temp1 = pos_x;
		// temp2 = pos_y;

		//funcion = posxx;
		funcion = posx;				//comentar si se hacen pruebas reales con el DVL
    	dfuncion = (funcion - funcion_ant)/(0.05);
		funcion_ant = funcion;
		vnort = dfuncion;

		//funcion2 = posyy;
		funcion2 = posy;			//comentar si se hacen pruebas reales con el DVL
    	dfuncion2 = (funcion2 - funcion_ant2)/(0.05);
		funcion_ant2 = funcion2;
		vest = dfuncion2;

		funcion3 = zbf_a;
		dfuncion3 = (funcion3 - funcion_ant3) / 0.05;
		funcion_ant3 = funcion3;
		d_z = dfuncion3;

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
			//for (tiempo3=0; i < 50; i++)
			//fforward.data=1500+100;
	    	//pubForward.publish(fforward);
			//B1=1;
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
			tiempo3 = 0;
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
			
			Tf = 40;  	// Tiempo de trayectoria para el circulo y lineal
			//Tf = 40;	// Tiempo de trayectoria cuadrada
			
			//pfx = 4;	//Trayectoria lineal
			//pfy = 2;		//Trayectoria lineal
			pfx = -4;
			pfy = 4;
			// TRAYECTORIA CUADRADA
			if (tiempo2 < Tf){
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
			}
			
			// TRAYECTORIA CIRCULAR
			
			/*xd = 5*cos(6.28*tiempo2/Tf); 
			xdp = -5*(6.28/Tf)*sin(6.28*tiempo2/Tf);
			xdpp = -5*(6.28/Tf)*(6.28/Tf)*cos(6.28*tiempo2/Tf);
			//xdp = -(28.26/Tf)*sin(6.28*tiempo2/Tf); 					
			//xdpp = -(28.26/Tf)*(28.26/Tf)*cos(6.28*tiempo2/Tf);
			
			yd = 5*sin(6.28*tiempo2/Tf);
			ydp = 5*(6.28/Tf)*cos(6.28*tiempo2/Tf);
			ydpp = -5*(6.28/Tf)*(6.28/Tf)*sin(6.28*tiempo2/Tf);
			//ydp = (28.26/Tf)*cos(6.28*tiempo2/Tf);
			//ydpp = -(28.26/Tf)*(28.26/Tf)*sin(6.28*tiempo2/Tf);*/
			
			
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
			
			//################# INICIA EL CONTROLADOR #################
			//######################       MRAC       ###################
			//##################### Control de Yaw ###################
			int amp=60, perd =60;
			int amp2=10, perd2 =60;
			float a1= (2*PI/perd);
			float a2= (2*PI/perd2);
			
			yawDes = amp * sin( a1*tiempo3 ); 
			yawDes_p = a1*amp *cos( a1*tiempo3 ); 
			yawDes_pp = - ((a1)*(a1))*amp * sin( amp*tiempo3 );
			// yawDes = -140.0;
			// yawDes_p = 0.0;

			eyaw =  posyaw - yawDes;  	
			
			if (eyaw > 10)	
				eyaw = 10;
			if (eyaw < -10)
				eyaw = -10;

			//eyaw_p = posyaw_p - yawDes_p;											
			//eyaw_i = eyaw_i + (eyaw*0.05);
			
			//yyaw = -( kpyaw*eyaw + kiyaw*eyaw_i + kdyaw*eyaw_p );							

			/*Gammayaw = 10, Gammar_yaw =10, Gamma_Theta_yaw=10;
			Kyaw_hat = Kyaw_hat + 0.05*( -Gammayaw * posyaw * eyaw * 1 * 1 );
    		Kr_hat_yaw = Kr_hat_yaw + 0.05*( -Gammar_yaw * yawDes * eyaw * 1 * 1 );
    		Theta_hat_yaw = Theta_hat_yaw + 0.05*( Gamma_Theta_yaw * posyaw_p * eyaw * 1 * 1 );*/
			//yawr = yawr + 0.05*( yawr2 - pow(wn,2)*yawr1 - 2*zeta*wn*yawr2 + pow(wn,2)*yawDes );
			yawr1 = yawr1 + 0.05*( yawr2 );
    		yawr2 = yawr2 + 0.05*( - pow(wn,2)*yawr1 - 2*zeta*wn*yawr2 + pow(wn,2)*yawDes );
    
    		eyaw1 = posyaw - yawr1;
    		eyaw2 = posyaw_p - yawr2;

			Kyaw_hat1 = Kyaw_hat1 + 0.05*( - 0.00005*posyaw*eyaw1 - 0.000039286*posyaw*eyaw2 );
    		Kyaw_hat2 = Kyaw_hat2 + 0.05*( -0.00005*posyaw_p*eyaw1 - 0.000039286*posyaw_p*eyaw2 );

    		Kr_hat_yaw = Kr_hat_yaw + 0.05*( -0.00005*yawDes*eyaw1 - 0.000039286*yawDes*eyaw2 );

    		Theta_hat1_yaw = Theta_hat1_yaw + 0.05*( 0.00005*posyaw_p*eyaw1 + 0.000039286*posyaw_p*eyaw2 );
    		Theta_hat2_yaw = Theta_hat2_yaw + 0.05*( 0.00005*posyaw_p*abs(posyaw_p)*eyaw1 + 0.000039286*posyaw_p*abs(posyaw_p)*eyaw2 );

			//yyaw = ( Kyaw_hat1*posyaw + Kyaw_hat2*posyaw_p + Kr_hat_yaw*yawDes -  Theta_hat_yaw1*posyaw_p - Theta_hat_yaw2*posyaw_p*abs(posyaw_p) );
			yyaw = ( Kyaw_hat1*posyaw + Kyaw_hat2*posyaw_p + Kr_hat_yaw*yawDes  - Theta_hat1_yaw*posyaw_p - Theta_hat2_yaw*posyaw_p*abs(posyaw_p) );

			if (yyaw >= 0)
				uyaw=(1515+yyaw);
			else 
				uyaw=(1485+yyaw);
														
			usyaw=saturayaw(uyaw);	//Satura la señal de pwm entre 1100 a 1900
			
			//COMENTAR SI SE REALIZAN PRUEBAS FISICAS CON EL VEHICULO FUERA DEL AGUA
			//verticalc.data=usz; //se envia valor saturado
			//pubVertical.publish(verticalc);

			yawc.data=usyaw; 
			eyaw_ant = eyaw;	// guarda valores anteriorees
			//pubForward.publish(fforward);

			//################## Control en Z #################			
			//kpz = 3.5, kiz = 0.01, kdz = 2; 
			//kpz = 4.5, kiz = 0.05, kdz = 3.5;
			kpz = 4.5, kiz = 0.05, kdz = 3.5;  
			//zDes = 21;
			zDes = 20 + amp2 * cos( a2*tiempo3 ); 
			zDes_p = -a2*amp2 *sin( a2*tiempo3 ); 
			zDes_pp = - ((a2)*(a2))*amp2 * cos( a2*tiempo3 );

			zr1 = zr1 + 0.05*( zr2 );
    		zr2 = zr2 + 0.05*( - pow(wn,2)*zr1 - 2*zeta*wn*zr2 + pow(wn,2)*zDes );
    
    		ez1 = zbf_a - zr1;
    		ez2 = d_z - zr2;

			kz_hat1 = kz_hat1 + 0.05*( -0.00075*zbf_a*ez1 - 0.000*zbf_a*ez2 );
    		kz_hat2 = kz_hat2 + 0.05*( -0.00075*d_z*ez1 - 0.0001*d_z*ez2 );

    		Kr_hat_z = Kr_hat_z + 0.05*( -0.00075*zDes*ez1 - 0.0001*zDes*ez2 );

    		Theta_hat1_z = Theta_hat1_z + 0.05*( 0.00075*d_z*ez1 + 0.0001*d_z*ez2 );
    		Theta_hat2_z = Theta_hat2_z + 0.05*( 0.00075*d_z*abs(d_z)*ez1 + 0.0001*d_z*abs(d_z)*ez2 );

			zz = ( kz_hat1*zbf_a + kz_hat2*d_z + Kr_hat_z*zDes  - Theta_hat1_z*d_z - Theta_hat2_z*d_z*abs(d_z) );

			if (zz >= 0)
				uz=(1515+zz);
			else 
				uz=(1485+zz);

			usz=satura(uz);														//Satura la señal de pwm entre 1100 a 1900
			verticalc.data=usz;	

			//################## Control en X y Y ################
			xr1 = xr1 + 0.05*( xr2 );
    		xr2 = xr2 + 0.05*( - pow(wn,2)*xr1 - 2*zeta*wn*xr2 + pow(wn,2)*xd );
    
    		ex1 = posx - xr1;
    		ex2 = vnort - xr2;
			
			//exx = ( posx - xd );
			int ox=1;
			if (exx > ox)    
			    exx = ox;
			if (exx < -ox)
			    exx = -ox;                              

			//exx_p = ( vnort - xdp );
			//exx_i = exx_i + (exx*0.05);                      
			// Kx_hat1 = Kx_hat1 + 0.05*( -3*posx*ex1 - 3.9286*posx*ex2 );
    		// Kx_hat2 = Kx_hat2 + 0.05*( -3*vnort*ex1 - 3.9286*vnort*ex2 );

    		// Kr_hat_x = Kr_hat_x + 0.05*( -3*xd*ex1 - 3.9286*xd*ex2 );

    		// Theta_hat1_x = Theta_hat1_x + 0.05*( 3*vnort*ex1 + 3.9286*vnort*ex2 );
    		// Theta_hat2_x = Theta_hat2_x + 0.05*( 3*vnort*abs(vnort)*ex1 + 3.9286*vnort*abs(vnort)*ex2 );

			Kx_hat1 = Kx_hat1 + 0.05*( -2*posx*ex1 - 2*posx*ex2 );
    		Kx_hat2 = Kx_hat2 + 0.05*( -2*vnort*ex1 - 2*vnort*ex2 );

    		Kr_hat_x = Kr_hat_x + 0.05*( -2*xd*ex1 - 2*xd*ex2 );

    		Theta_hat1_x = Theta_hat1_x + 0.05*( 2*vnort*ex1 + 2*vnort*ex2 );
    		Theta_hat2_x = Theta_hat2_x + 0.05*( 2*vnort*abs(vnort)*ex1 + 2*vnort*abs(vnort)*ex2 );


			// Kx_hat1 = Kx_hat1 + 0.05*( -0.2*posx*ex1 - 0.2*posx*ex2 );
    		// Kx_hat2 = Kx_hat2 + 0.05*( -0.2*vnort*ex1 - 0.2*vnort*ex2 );

    		// Kr_hat_x = Kr_hat_x + 0.05*( -0.2*xd*ex1 - 0.*xd*ex2 );

    		// Theta_hat1_x = Theta_hat1_x + 0.05*( 0.2*vnort*ex1 + 0.2*vnort*ex2 );
    		// Theta_hat2_x = Theta_hat2_x + 0.05*( 0.*vnort*abs(vnort)*ex1 + 0.2*vnort*abs(vnort)*ex2 );

			ux_B  = ( Kx_hat1*posx + Kx_hat2*vnort + Kr_hat_x*xd  - Theta_hat1_x*vnort - Theta_hat2_x*vnort*abs(vnort) );

			//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			yr1 = yr1 + 0.05*( yr2 );
    		yr2 = yr2 + 0.05*( - pow(wn,2)*yr1 - 2*zeta*wn*yr2 + pow(wn,2)*yd );
    
    		ey1 = posy - yr1;
    		ey2 = vest - yr2;
			
			//exx = ( posx - xd );
			int oy=1;
			if (eyy > oy)    
			    eyy = oy;
			if (eyy < -oy)
			    eyy = -oy;                              

			//eyy_p = ( vest - ydp );
			//eyy_i = eyy_i + (eyy*0.05);                      
			// Ky_hat1 = Ky_hat1 + 0.05*( -3*posy*ey1 - 3.9286*posy*ey2 );
    		// Ky_hat2 = Ky_hat2 + 0.05*( -3*vest*ey1 - 3.9286*vest*ey2 );

    		// Kr_hat_y = Kr_hat_y + 0.05*( -3*yd*ey1 - 3.9286*yd*ey2 );

    		// Theta_hat1_y = Theta_hat1_y + 0.05*( 3*vest*ey1 + 3.9286*vest*ey2 );
    		// Theta_hat2_y = Theta_hat2_y + 0.05*( 3*vest*abs(vest)*ey1 + 3.9286*vest*abs(vest)*ey2 );

			Ky_hat1 = Ky_hat1 + 0.05*( -2*posy*ey1 - 2*posy*ey2 );
    		Ky_hat2 = Ky_hat2 + 0.05*( -2*vest*ey1 - 2*vest*ey2 );

    		Kr_hat_y = Kr_hat_y + 0.05*( -2*yd*ey1 - 2*yd*ey2 );

    		Theta_hat1_y = Theta_hat1_y + 0.05*( 2*vest*ey1 + 2*vest*ey2 );
    		Theta_hat2_y = Theta_hat2_y + 0.05*( 2*vest*abs(vest)*ey1 + 2*vest*abs(vest)*ey2 );

			uy_B  = ( Ky_hat1*posy + Ky_hat2*vest + Kr_hat_y*yd  - Theta_hat1_y*vest - Theta_hat2_y*vest*abs(vest) );

			// Matriz de rotación
			ux_I = ux_B*cos(posyaw*3.141592/180) + uy_B*sin(posyaw*3.141592/180);
		
			uy_I = -ux_B*sin(posyaw*3.141592/180) + uy_B*cos(posyaw*3.141592/180);

			if (ux_I >= 0 )
				uxx=(1515+ux_I);	
			else
				uxx=(1485+ux_I);																					
			usxx=saturaposxx(uxx);								
			u_F.data=usxx;										

			if (uy_I >= 0 )
				uyy=(1515+uy_I );	
			else
				uyy=(1485+uy_I);			
			usyy=saturaposyy(uyy);								
			u_L.data=usyy;

			//pubForward.publish(u_F);
			//pubLateral.publish(u_L);
			//pubVertical.publish(verticalc);
			pubYaw.publish(yawc);

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
			//ROS_INFO_STREAM("Mensaje: " << "Holaa" << endl) ;

	  	}

		//////////////////// GUARDA VARIABLES ANTERIORES ////////////////
		//ex_ant = ex;
		//ey_ant = ey;
		ez_ant = ez;
			
		//ROS_INFO_STREAM("DatosPos_X: " << tiempo2  << "\t" << ex1  << "\t" << ux_B << "\t" << usxx << "\t" << posx << "\t" << xd << "\t" << Kx_hat1 << "\t" << Kx_hat2 << "\t" << Kr_hat_x << "\t" << Theta_hat1_x << "\t" << Theta_hat2_x << endl);
		//ROS_INFO_STREAM("DatosPos_Y: " << tiempo2  << "\t" << ey1  << "\t" << uy_B << "\t" << usyy << "\t" << posy << "\t" << yd << "\t" << Ky_hat1 << "\t" << Ky_hat2 << "\t" << Kr_hat_y << "\t" << Theta_hat1_y << "\t" << Theta_hat2_y << endl);
		//printf("Press: %.3f,Altura:%.3f \n", , zbf);
		//ROS_INFO_STREAM("DatosPos_Y: " << tiempo2  << "\t" << eyy  << "\t" << uy_B << "\t" << usyy << "\t" << posyy << "\t" << yd  << endl);
		//ROS_INFO_STREAM("DatosPos_Z: " << tiempo2  << "\t" << ez  << "\t" << zz << "\t" << usz << "\t" << z_baro << "\t" << zbf_a  << "\t" << zDes  << endl);
		//printf("t= %0.3f \t z= %0.3f \n",tiempo2,z_baro);
		//ROS_INFO_STREAM("DatosPos_Yaw: " << tiempo2  << "\t" << eyaw1  << "\t" << yyaw << "\t" << usyaw << "\t" << posyaw << "\t" << yawDes << "\t" << Kyaw_hat1 << "\t" << Kyaw_hat2 << "\t" << Kr_hat_yaw << "\t" << Theta_hat1_yaw << "\t" << Theta_hat2_yaw << endl);
		ROS_INFO_STREAM("DatosPos_Z: " << tiempo2  << "\t" << ez1  << "\t" << zz << "\t" << usz << "\t" << zbf_a << "\t" << zDes << "\t" << kz_hat1 << "\t" << kz_hat2 << "\t" << Kr_hat_z << "\t" << Theta_hat1_z << "\t" << Theta_hat2_z << endl);

		//ROS_INFO_STREAM("DatosPos_Yaw: " << tiempo2  << "\t" << eyaw  << "\t" << yyaw << "\t" << usyaw << "\t" << posyaw << "\t" << yawDes << endl);
		
		//ROS_INFO_STREAM("DatosPos_Y: " << tiempo2  << "\t" << eyy  << "\t" << uy_B << "\t" << usyy << "\t" << posyy << "\t" << tr[6] << "\t" << poss2  << endl);
		//ROS_INFO_STREAM("DatosPos_Y: " << tiempo2  << "\t" << uy_B  << "\t" << uy_I << "\t" << usyy << "\t" << eyy  << "\t" << posy  << "\t" << yd << endl);
		//ROS_INFO_STREAM("DatosPos_X: " << tiempo2  << "\t" << posx  << "\t" << fforward << endl);
		//ROS_INFO_STREAM("DatosPos_Y: " << posy << "\t" << uy_B  << "\t" << uyy << "\t" << Ydes_f << endl);
		//ROS_INFO_STREAM("DatosYaw: " << posyaw  << "\t" << yyaw << "\t" << uyaw  << endl);
		//ROS_INFO_STREAM("Datos: " << u_F.data  << "\t" << u_L.data   << "\t"  <<  ux_I << "\t"  <<  uy_I <<  "\t" << Ydes_f << "\t" << tiempo2 << endl);
		//ROS_INFO_STREAM("Referencias: " << xd << "\t" << xdp << "\t" << yd << "\t" << ydp << "t" << tiempo2 << endl);

		//myfile << std::setprecision(5) << tiempo << "\t"   << tr[0]  << "\t" << tr[5] 
		//<< "\t" << tr[6] << "\t" << tr[7]  << "\t" << B1 << "\t" << B2 << "\t" << posxx << "\t" << posyy <<  "\n";
		//myfile << std::setprecision(5) << tiempo << "\t" << tr[5] 
		//<< "\t" << tr[6] << "\t" << tr[7]  << "\t" << posx << "\t" << posy <<  "\n";
		myfile << std::setprecision(5) << tiempo2 << "\t" << xd << "\t" << yd << "\t" << zDes << "\t" << yawDes << "\t" << posx << "\t" << posy << "\t" << z_baro << "\t" << zbf_a << "\t" << posyaw << "\t" << ux_B << "\t" << uy_B << "\t" << usxx << "\t" << usyy << "\t" << pos_x << "\t" << pos_y << "\t" << ez << "\t" << fluid_press 
		<< "\t" << Kyaw_hat1 << "\t" << Kyaw_hat2 << "\t" << Kr_hat_yaw << "\t" << Theta_hat1_yaw << "\t" << Theta_hat2_yaw 
		<< "\t" << kz_hat1 << "\t" << kz_hat2 << "\t" << Kr_hat_z << "\t" << Theta_hat1_z << "\t" << Theta_hat2_z << "\n";
      	//##########################Lectura de datos para el mensaje tipo: $DVPDL###################
		

		ros::spinOnce();
      	loop_rate.sleep();
      	tiempo = tiempo + T;
		tiempo3 = tiempo3 + T;
      	
  	} //FIN DEL WHILE
  
  delete H;
  return 0;
  myfile.close();
  myfile2.close();
} //FIN DEL MAIN


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