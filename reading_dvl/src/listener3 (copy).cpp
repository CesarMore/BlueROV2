
//###########################################################################################

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

using namespace std;

string encontrar_distacias(string s, string start_caracter, string reset_caracter , string delimiter_char, int posicion, int n_de_posiciones)
{    
    
    static int c = 0, i_cadena = 0, n_pos = 0, f_cadena = 0, p_string = 0, p = 0;
    static char start = false;
    string substring = "";
    int d = 0, inicio = 0;
    
    inicio = s.find(start_caracter);
    if(s.find(reset_caracter) != -1){
      start = false;
      c = 0;
      n_pos = 0;
      i_cadena = 0;
      f_cadena = 0;
      p_string = 0;
      return("error");
    }

    if (inicio != -1 || start == true){
        //ROS_INFO_STREAM("posicion de inicio: " << inicio <<endl);
        start = true;
        
        if(s.find(start_caracter) != -1)
          p = inicio;
        else 
          p = 0;

        while (c < posicion)
        {   
            p = s.find(delimiter_char, p+1);
            if( p != -1){
                p_string = p;
                c++;   
              //  ROS_INFO_STREAM("contador: " << c <<endl);
            }else{
                return("error");
                break;
            }
        }
        if(posicion == c){
            i_cadena = p_string;
            //ROS_INFO_STREAM("inicio de la cadena: " << i_cadena <<endl);
            while (n_pos < n_de_posiciones)
            {
                p = s.find(delimiter_char, p+1);
                if(p != -1){
                    p_string = p;
                    n_pos++;
                }else{
                    start = false;
                    c = 0;
                    n_pos = 0;
                    i_cadena = 0;
                    f_cadena = 0;
                    p_string = 0;
                    return("error");
                    break;
                }
                if(n_pos == n_de_posiciones){
                    f_cadena = p_string;
                    d = f_cadena - i_cadena;
                    substring = s.substr(i_cadena+1, d-1);
                    start = false;
                    c = 0;
                    n_pos = 0;
                    i_cadena = 0;
                    f_cadena = 0;
                    p_string = 0;
                    return(substring);
                }
            }
        }
    }
    return("error");
}

//################# Variables Globales #################
double posx, posy, posz, posax, posay, posaz, posaw, posrollrad, 
pospitchrad, posyawrad, posroll, pospitch, posyaw;

string dato;
string encontrar_distacias(string s, string ="$DVPDL", string ="$DVEXT", string =",", int =6, int =3);

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
   if (posyaw > 180){
      posyaw = posyaw -360;
   };
}

//Datos DVL
void chatterCallback(const std_msgs::String::ConstPtr & msg)
{
  dato = msg->data.c_str();
  string retorno = "";
  retorno = encontrar_distacias(dato, "$DVPDL");
  //cout<< "substring :"<< retorno << endl;
  //ROS_INFO_STREAM("dato: " << dato <<endl);
  if(retorno != "error"){
    //ROS_INFO_STREAM("Datos2: " << retorno <<endl);
  }
  //ROS_INFO_STREAM("Datos2: " << dato <<endl);
}


//#############################################
int main(int argc, char **argv)
{
  ros::init(argc, argv, "dvl_subscriber");
  ros::NodeHandle n;
  ros::NodeHandle nh;
  ros::Subscriber sub = n.subscribe("dvl_out", 1, chatterCallback);
  ros::Rate loop_rate(20);
  
  int i = 0, pos1 = 0, pos2 = 0, b1 = 0, b2 = 0, 
  c = 0, init = 0, init1 = 0, posf;
  string dvl1, dvl2, dvl3, dato_anterior;
  float tiempo = 0, T = 0.05;
  float tr[8];
  float gc[9];
  //############### $DVPDL #####################
  

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
  
  while (ros::ok())
  {
    //#########Lectura de datos para del mensaje tipo: $DVEXT##########
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
        
          }
        }
        
      }
      
      //##############Lectura de datos para del mensaje tipo: $DVPDL#################
      /*std::string s = dato;
      std::string delimiter_char = ",";
      size_t pos = 0;
      std::string token;
      while ((pos = s.find(delimiter_char)) != std::string::npos) {
        token = s.substr(0, pos);
        //std::cout << token << std::endl;
        s.erase(0, pos + delimiter_char.length());
      }//FIN DEL WHILE*/
      
      
      //ROS_INFO_STREAM("Datos2: " << s <<endl);

      //ROS_INFO_STREAM("DVL1: " << dvl1 <<endl);

      ROS_INFO_STREAM("Datos1: " << tr[0] << "\t" << tr[1]  << "\t" << tr[2] << "\t" << tr[3] << "\t" 
      << tr[4] << "\t" << tr[5] << "\t" << tr[6] << "\t" << tr[7] << endl ) ;
      //ROS_INFO_STREAM("Datos1: " << tr[0] << endl ) ;

      ROS_INFO_STREAM("Orientacion: " << (posrollrad*180)/3.1416 << "\t" << (pospitchrad*180)/3.1416  << "\t" << (posyawrad*180)/3.1416 << endl);
      //ROS_INFO_STREAM("Orientacion: "  << (posrollrad*180)/3.1416  << endl);
    }

    //##########################Lectura de datos para el mensaje tipo: $DVPDL###################
      ros::spinOnce();
      loop_rate.sleep();
      tiempo = tiempo + T;
      //ROS_INFO_STREAM("Tiempo: " << tiempo <<endl);
      
    myfile << std::setprecision(5) << tiempo << "\t"   << tr[0]  << "\t" << tr[1]  << "\t" << tr[2]   
    << "\t" << tr[3]  << "\t" << tr[4] << "\t" << tr[5]  << "\t" << tr[6]  << "\t" << tr[7] << "\t" 
    << (posrollrad*180)/3.1416 << "\t" << (pospitchrad*180)/3.1416  << "\t" << (posyawrad*180)/3.1416 << "\n";
  
  } //FIN DEL WHILE
  
  return 0;
  //myfile << "\n";
  myfile.close();
} //FIN DEL MAIN






