
#include <iostream>
int main()
{
    std::string s = "$DVPDL,373550,1000,200000,0.000000,-0.000000,-0.000000,0.000,-0.000,-0.000,0*62";
    std::string delimiter_char = ",";
    size_t pos = 0;
    std::string token;
    while ((pos = s.find(delimiter_char)) != std::string::npos) {
        token = s.substr(0, pos);
        std::cout << token << std::endl;
        s.erase(0, pos + delimiter_char.length());
    }
std::cout << s << "\n";
}



//####################################################################################
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Empty.h>
#include <sstream>
#include <iostream>
#include <vector>
#include <fstream>
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/tf.h"
#include "tf/tfMessage.h"
#define PI 3.14159265358979323846
#include <string>     // std::string, std::stof

using namespace std;

//################# Variables Globales #################
double posx, posy, posz, posax, posay, posaz, posaw, posrollrad, 
pospitchrad, posyawrad, posroll, pospitch, posyaw;

string dato;

//################# Funciones Callback #################
//Orientación vehiculo real (Roll, Pitch, Yaw)       				%%%%%%%%%%%
void posCallback(const sensor_msgs::Imu::ConstPtr & msg) {
  posax=msg->orientation.x; 
	posay=msg->orientation.y;
	posaz=msg->orientation.z;
	posaw=msg->orientation.w; 
	
	tf::Quaternion q(posax, posay, posaz, posaw);
  tf::Matrix3x3 m(q);
  m.getRPY(posrollrad,pospitchrad,posyawrad);
	
	//ROS_INFO( "posaw%.3f,", msg->orientation.x);
	
	posyaw=posyawrad*(180/PI);
}

//Datos DVL
void chatterCallback(const std_msgs::String::ConstPtr & msg)
{
  dato = msg->data.c_str();
}
//####################################
int main(int argc, char **argv)
{
  ros::init(argc, argv, "dvl_subscriber");
  ros::NodeHandle n;
  ros::NodeHandle nh;
  ros::Subscriber sub = n.subscribe("dvl_out", 1, chatterCallback);
  ros::Rate loop_rate(25);
  
  int i = 0, pos1 = 0, pos2 = 0, pos3 = 0, pos4 = 0, b1 = 0, b2 = 0, 
  c = 0, c1 = 0, init = 0, init1 = 0, init2 = 0, init3 = 0, posf, posf2;
  string dvl1, dvl2, dvl3, dato_anterior;
  float tiempo = 0, T = 0.05;
  float tr[8];
  float gc[9];
  float tr2[8];

  //#################Generar archivo donde cuardar los datos del DVL DVL##############
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
  
  //Nodo
  ros::Subscriber subPos;
  //################# Subscriptores #################
  subPos  = nh.subscribe("/BlueRov2/imu/data",1, posCallback);  //Topico real

  //ROS_INFO_STREAM("DatosCompletos: " << dato <<endl);
  
  while (ros::ok())
  {
    //#########################Lectura de datos para el mensaje tipo: $DVEXT######################
    string inicio = "$DVEXT";
    if (dato.compare(dato_anterior) != 0)
    {
      pos1 = dato.find("$DVEXT");
      if (pos1 == 0 )
      {
        dvl1 = dato;
      }
      else if (pos1 == -1)
      {
      dvl1 = dvl1 + dato;
      }
      
      pos2 = dvl1.find("*");
      c = 0;
      
      if (pos2 != -1)
      {
        pos2 = dvl1.find("*", pos2 + 1);
        if (pos2 != -1)
        c = 2;
      }
      
      dato_anterior = dato;

      if (c == 2)
      {
        c = 0;
        
        init = 16;
        tr[0] = 1;
        init1 = init;
      
        if (dvl1.find("$DVEXT") != -1)
        {

          for (i=0; i < 8; i++)
          {
          posf = dvl1.find(',', init1 );
          tr[i] = stof(dvl1.substr(init1, posf-init1));
          init1 = posf+1;   
          //ROS_INFO_STREAM("DVL1: " << dvl1 <<endl);
          }
        }
        //ROS_INFO_STREAM("Datos1: " << tr[0] << "\t" << tr[1]  << "\t" << tr[2] << "\t" << tr[3] << "\t" << tr[4] << "\t"
        //<< tr[5] << "\t" << tr[6] << "\t" << tr[7] << endl ) ;
      }
    }

    ///############################Lectura de datos para el mensaje tipo: $DVPDL#############################
    
    if (dvl1.find("$DVPDL") != -1)
      {
        for (i=0; i < 8; i++)
        {
        posf2 = dvl1.find(',', init2 );
        tr2[i] = stof(dvl1.substr(init2, posf2-init2));
        init1 = posf+1;   
        }
      }
    
    ROS_INFO_STREAM("Datos1: " << tr[0] << "\t" << tr[1]  << "\t" << tr[2] << "\t" << tr[3] 
    << "\t" << tr[4] << "\t" << tr[5] << "\t" << tr[6] << "\t" << tr[7] << endl ) ;

    ROS_INFO_STREAM("Datos2: " << tr2[0] << "\t" << tr2[1]  << "\t" << tr2[2] << "\t" << tr2[3] 
    << "\t" << tr2[4] << "\t" << tr2[5] << "\t" << tr2[6] << "\t" << tr2[7] << endl );

    ROS_INFO_STREAM("Orientacion: " << posrollrad << "\t" << pospitchrad  << "\t" << posyawrad << endl );
    
    /*
    string econtrar = "$DVPDL";
    if (dato.compare(dato_anterior) != 0)
    {
      pos3 = dato.find("$DVPDL");
      if (pos3 == 0 )
      {
        dvl1 = dato;
      }
      else if (pos3 == -1)
      {
      dvl1 = dvl1 + dato;
      }
      
      pos4 = dvl1.find("*");
      c1 = 0;
      
      if (pos4 != -1)
      {
        pos4 = dvl1.find("*", pos4 + 1);
        if (pos4 != -1)
        c1 = 2;
      }
      
      dato_anterior = dato;

      if (c1 == 2)
      {
        c1 = 0;
        
        init2 = 16;
        tr2[0] = 1;
        init3 = init2;
      
        if (dvl1.find("$DVPDL") != -1)
        {

          for (i=0; i < 8; i++)
          {
          posf2 = dvl1.find(',', init3 );
          tr2[i] = stof(dvl1.substr(init3, posf-init3));
          init3 = posf2 + 1;   
          ROS_INFO_STREAM("DVL!: " << tr[i] <<endl);
          }
        }
        
      }
      ROS_INFO_STREAM("Datos2: " << tr2[0] << "\t" << tr2[1]  << "\t" << tr2[2] << "\t" << tr2[3] 
      << "\t" << tr2[4] << "\t" << tr2[5] << "\t" << tr2[6] << "\t" << tr2[7] << endl );

      ROS_INFO_STREAM("Orientacion: " << posrollrad << "\t" << pospitchrad  << "\t" << posyawrad << endl );
    }
    */
    ros::spinOnce();
    loop_rate.sleep();
    tiempo = tiempo + T;
    //ROS_INFO_STREAM("Tiempo: " << tiempo <<endl);

    myfile << std::setprecision(5) << tiempo << "\t"   << tr[0]  << "\t" << tr[1]  << "\t" << tr[2]  << "\t" << tr[3]  << "\t" << tr[4]
    << "\t" << tr[5]  << "\t" << tr[6]  << "\t" << tr[7] << "\t" << posrollrad << "\t" << pospitchrad  << "\t" << posyawrad << "\n";
  
  } //FIN DEL WHILE
  
  return 0;
  //myfile << "\n";
  myfile.close();
} // FIN DEL MAIN