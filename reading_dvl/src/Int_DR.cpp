#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>

using namespace std;

int main() {

    double T = 0.005;
    vector<double> t;
    for (double i = 0; i <= 10; i += T) {
        t.push_back(i);
    }

    double tau = 1;

    double xr = 1.5;
    vector<double> xr_f(t.size(), 0), xr_fp(t.size(), 0);

    for (int i = 0; i < t.size() - 1; i++) {
        xr_f[i + 1] = xr_f[i] + (T / tau) * (xr - xr_f[i]);
        xr_fp[i + 1] = (1 / tau) * (xr - xr_f[i]);
    }

    // Gráficar los resultados
    ofstream f1("figure1.csv");
    for (int i = 0; i < t.size(); i++) {
        f1 << t[i] << "," << xr_f[i] << endl;
    }
    f1.close();

    ofstream f2("figure2.csv");
    for (int i = 0; i < t.size(); i++) {
        f2 << t[i] << "," << xr_fp[i] << endl;
    }
    f2.close();

    return 0;
}