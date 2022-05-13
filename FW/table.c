#include <iostream>
#include <math.h>
#include <iomanip>

using namespace std;

int main()
{
	cout << fixed << setprecision(0);
	for(int i = 1; i <= 200; i++){
		cout << pow(1.03509, i) + 10 << ",\t";
		if((i % 16) == 0) cout << "\n";
	}
	return 0;
}
