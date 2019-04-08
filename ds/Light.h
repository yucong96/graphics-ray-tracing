#ifndef __LIGHT_H__
#define __LIGHT_H__

#include <string>
#include "Vec.h"

class Light {
public:
	Light(std::string _type, Vec& _center, double _radius, Vec &_Le) {
		type = _type;
		center = _center;
		radius = _radius;
		Le = _Le;
	}
	Light(std::string _type, Vec& _center, Vec& _norm, Vec &_x_axis, Vec& _size, Vec& _Le) {
		type = _type;
		center = _center;
		norm = _norm;
		x_axis = _x_axis;
		size = _size;
		Le = _Le;
	}
	std::string type;
	Vec center;
	double radius; // for sphere
	Vec norm; // for quad
	Vec x_axis; // for quad
	Vec size; // for quad
	Vec Le;
};

#endif