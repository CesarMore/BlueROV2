#include <iostream>

using namespace std;

string encontrar_distacias(string s, string ="$DVPDL", string =",", int =7, int =3);

int main()
{
    string retorno = "dato";
    string s = "$DVPDL,373550,1000,200000,";
    retorno = encontrar_distacias(s, "$DVPDL");
    cout<< "substring :"<< retorno << endl;
    s = "0.000000,-0.000000,-0.000000";
    retorno = encontrar_distacias(s, "$DVPDL");
    cout<< "substring :"<< retorno << endl;
    s = ",0.000,-0.000,-0.000,0*62";
    retorno = encontrar_distacias(s);
    cout<< "substring :"<< retorno << endl;
}

string encontrar_distacias(string s, string start_caracter, string delimiter_char, int posicion, int numero_de_posiciones)
{    
    
    static int c = 0, i_cadena = 0, n_pos = 0, f_pos = 0, p_string = 0, p = 0;
    static char start = false;
    string substring = "";
    int d = 0, inicio = 0;
    
    inicio = s.find(start_caracter);

    if (inicio != -1 || start == true){
        start = true;
        while (c < posicion)
        {   
            p = s.find(delimiter_char, p+1);
            if( p != -1){
                p_string = p;
                c++;   
            }else{
                return(" ");
                break;
            }
        }
        if(posicion == c){
            i_cadena = p_string;
            while (n_pos < numero_de_posiciones)
            {
                p = s.find(delimiter_char, p+1);
                if(p != -1){
                    p_string = p;
                    n_pos++;
                }else{
                    return("error ");
                    break;
                }
                if(n_pos == numero_de_posiciones){
                    f_pos = p_string;
                    d = f_pos - i_cadena;
                    substring = s.substr(i_cadena+1, d-1);
                    start = false;
                    c = 0;
                    n_pos = 0;
                    i_cadena = 0;
                    f_pos = 0;
                    p_string = 0;
                    return(substring);
                }
            }
        }
    }
}