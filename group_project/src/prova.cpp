#include <iostream>
#include <fstream>

using namespace std;

int main() {
ofstream MyFile("hola.txt");

MyFile << "che due palle" << endl;

MyFile.close();
}
