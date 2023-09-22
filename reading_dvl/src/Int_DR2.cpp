#include <iostream>
#include <cmath>

// Función para calcular la derivada de Euler de una función en un punto específico
double eulerDerivative(double (*f)(double), double x, double h) {
    return (f(x + h) - f(x)) / h;
}

// Ejemplo de función para calcular la derivada de una función dada
double exampleFunction(double x) {
    return sin(x);
}

int main() {
    double x = 1.0;    // Punto en el que se aproxima la derivada
    double h = 0.01;   // Tamaño del intervalo de la aproximación
    double derivative = eulerDerivative(exampleFunction, x, h);   // Aproximación de la derivada de la función en x
    std::cout << "La derivada de la función en x = " << x << " es " << derivative << std::endl;
    return 0;
}