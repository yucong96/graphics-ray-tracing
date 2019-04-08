#ifndef __RAY_H__
#define __RAY_H__

#include "Vec.h"

class Ray {
public:
	Vec origin;
	Vec direction;
	Ray(Vec &_origin, Vec &_direction) {
		for (int i = 0; i < _origin.size(); i++) {
			origin.push_back(_origin[i]);
		}
		for (int i = 0; i < _direction.size(); i++) {
			direction.push_back(_direction[i]);
		}
		direction.normalize();
	}
	friend std::ostream& operator<<(std::ostream& output, const Ray& ray) {
		output << "origin: " << ray.origin << " direction: " << ray.direction;
		return output;
	}
};

#endif