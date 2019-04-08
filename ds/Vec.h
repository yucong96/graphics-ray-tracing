#ifndef __VEC_H__
#define __VEC_H__

#include <iostream>
#include <vector>
#include <iterator>

class Vec {
public:
	std::vector<double> vec;
	Vec() {};
	Vec(const Vec& _vec);
	Vec(std::initializer_list<double> list);
	size_t size() const;
	void push_back(double val);
	double& operator[](const int i);
	const double& operator[](const int i) const;
	void operator=(Vec& _vec);
	Vec operator+(Vec& _vec);
	Vec operator-(Vec& _vec);
	Vec operator*(double r);
	Vec operator*(Vec& _vec);
	friend Vec operator*(double r, Vec& _vec) {
		return _vec * r;
	}
	Vec operator/(double r);
	Vec operator/(Vec& _vec);
	double norm();
	void normalize();
	double dot(Vec& _vec);
	Vec cross(Vec& _vec);
	void to_positive(void);
	friend std::ostream& operator<<(std::ostream& output, const Vec& v) {
		for (int i = 0, sz = v.vec.size(); i < sz; i++) {
			output << v.vec[i] << " ";
		}
		return output;
	}
};
std::vector<double> substract(std::vector<double>& vec1, std::vector<double>& vec2);

#endif