#include <fstream>
#include <random>
#include <time.h>
using namespace std;
//g++ -Wall plrg.c -o plrg
//1%-5%
//1%-2%
int main()
{
       	ofstream out;
	out.open("plr.txt");
	srand((int)time(0));
	for(int i = 1; i <= 66; i++){
		for(int j = i + 1; j <= 66; j++){
			double x = (double)rand()/RAND_MAX;
			out << (x * 1000 + 1000)/100000<<endl;
		}
	}
	return 0;
}
