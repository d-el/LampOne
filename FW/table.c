#include <iostream>
#include <math.h>
#include <iomanip>

using namespace std;

int main()
{
	cout << fixed << setprecision(0);
	for(int i = 1; i <= 150; i++){
		cout << pow(1.04707, i) + 9 << ",\t";
		if((i % 8) == 0) cout << "\n";
	}
	return 0;
}
