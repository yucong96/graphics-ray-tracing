#include "Vec.h"

#include <assert.h>

using namespace std;

size_t Vec::size() const {
	return vec.size();
}

Vec::Vec(initializer_list<double> list) {
	for (auto i = list.begin(); i != list.end(); i++) {
		vec.push_back(*i);
	}
}

Vec::Vec(const Vec& _vec) {
	vec.clear();
	for (int i = 0; i < _vec.size(); i++) {
		vec.push_back(_vec[i]);
	}
}

void Vec::push_back(double val) {
	vec.push_back(val);
}

double& Vec::operator[](const int i) {
	return vec[i];
}

const double& Vec::operator[](const int i) const {
	return vec[i];
}

void Vec::operator=(Vec& _vec) {
	vec.clear();
	for (int i = 0; i < _vec.size(); i++) {
		vec.push_back(_vec[i]);
	}
}

Vec Vec::operator+(Vec& _vec) {
	assert(vec.size() == _vec.size());
	Vec res;
	for (int i = 0, sz = vec.size(); i < sz; i++) {
		res.push_back(vec[i] + _vec[i]);
	}
	return res;
}

Vec Vec::operator-(Vec& _vec) {
	assert(vec.size() == _vec.size());
	Vec res;
	for (int i = 0, sz = vec.size(); i < sz; i++) {
		res.push_back(vec[i] - _vec[i]);
	}
	return res;
}

Vec Vec::operator*(double r) {
	Vec result;
	for (int i = 0, sz = vec.size(); i < sz; i++) {
		result.push_back(vec[i] * r);
	}
	return result;
}

Vec Vec::operator*(Vec& _vec) {
	assert(vec.size() == _vec.size());
	Vec res;
	for (int i = 0, sz = vec.size(); i < sz; i++) {
		res.push_back(vec[i] * _vec[i]);
	}
	return res;
}

Vec Vec::operator/(double r) {
	assert(r != 0);
	Vec result;
	for (int i = 0, sz = vec.size(); i < sz; i++) {
		result.push_back(vec[i] / r);
	}
	return result;
}

Vec Vec::operator/(Vec& _vec) {
	assert(vec.size() == _vec.size());
	Vec res;
	for (int i = 0, sz = vec.size(); i < sz; i++) {
		assert(_vec[i] != 0);
		res.push_back(vec[i] / _vec[i]);
	}
	return res;
}

double Vec::norm() {
	double res = 0;
	for (int i = 0, sz = vec.size(); i < sz; i++) {
		res += vec[i] * vec[i];
	}
	return sqrt(res);
}

void Vec::normalize() {
	double n = norm();
	for (int i = 0, sz = vec.size(); i < sz; i++) {
		vec[i] /= n;
	}
}

double Vec::dot(Vec& _vec) {
	assert(vec.size() == _vec.size());
	double res = 0;
	for (int i = 0, sz = vec.size(); i < sz; i++) {
		res += vec[i] * _vec[i];
	}
	return res;
}

Vec Vec::cross(Vec& _vec) {
	assert(vec.size() == 3 && _vec.size() == 3);
	Vec res({ vec[1] * _vec[2] - vec[2] * _vec[1], vec[2] * _vec[0] - vec[0] * _vec[2], vec[0] * _vec[1] - vec[1] * _vec[0] });
	return res;
}

void Vec::to_positive(void) {
	for (int i = 0; i < vec.size(); i++) {
		vec[i] = vec[i] > 0 ? vec[i] : 0;
	}
}