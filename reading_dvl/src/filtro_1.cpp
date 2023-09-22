#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>

using namespace std;
/*
int main() 
{
    double T = 0.005;
    double tau = 1;
    double xr = 1.5;
    double xr_f = 0;
    double xr_fp = 0;

    vector<double> t;
    for (double i = 0; i <= 10; i += T) {
        t.push_back(i);
    }

    vector<double> xr_f_vec(t.size());
    vector<double> xr_fp_vec(t.size());

    for (int i = 0; i < t.size() - 1; i++) {
        xr_f_vec[i+1] = xr_f_vec[i] + (T/tau)*( xr - xr_f_vec[i] );
        xr_fp_vec[i+1] = (1/tau)*( xr - xr_f_vec[i] );
    }
    
    ofstream fig1;
    fig1.open("fig1.txt");
    for (int i = 0; i < t.size(); i++) {
        fig1 << t[i] << " " << xr_f_vec[i] << " " << xr_fp_vec[i] << endl;
    }
    fig1.close();
    
    ofstream fig2;
    fig2.open("fig2.txt");
    for (int i = 0; i < t.size(); i++) {
        fig2 << t[i] << " " << xr_f_vec[i] << " " << xr_fp_vec[i] << endl;
    }
    fig2.close();

    return 0;

}
*/

int main() 
{
    double T = 0.005;
    double tau = 1;
    double xr = 1.5;
    double xr_f = 0;
    double xr_fp = 0;

    vector<double> t;
    for (double i = 0; i <= 10; i += T) {
        t.push_back(i);
    }

    double xr_f_vec[t.size()];
    double xr_fp_vec[t.size()];

    for (int i = 0; i < t.size() - 1; i++) {
        xr_f_vec[i+1] = xr_f_vec[i] + (T/tau)*( xr - xr_f_vec[i] );
        xr_fp_vec[i+1] = (1/tau)*( xr - xr_f_vec[i] );
    }
    
    ofstream fig1;
    fig1.open("fig1.txt");
    for (int i = 0; i < t.size(); i++) {
        fig1 << t[i] << " " << xr_f_vec[i] << " " << xr_fp_vec[i] << endl;
    }
    fig1.close();
    
    ofstream fig2;
    fig2.open("fig2.txt");
    for (int i = 0; i < t.size(); i++) {
        fig2 << t[i] << " " << xr_f_vec[i] << " " << xr_fp_vec[i] << endl;
    }
    fig2.close();

    return 0;

}
