#include <iostream>
//#include "vectors.h"
#include "matrices.h"//includes vectors.h
#include "Geometry2D.h"

using namespace std;

int main()
{
	vec3 vector = { 3.2135f,1.0f,1.0f };
	vec3 v;
	vec3 vector2 = { 4,2,1 };
	Point2D xd = Point2D(2.5f, 2.5f);
	cout<<(vector+vector2)<<endl;
	cout << "cross" << (Cross(vector, vector2)) << endl;
	cout << "XD";
	cout << endl; system("pause");
	return 0;
}