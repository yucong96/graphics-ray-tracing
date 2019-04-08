#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <string>
#include <fstream>
#include <sstream>
#include "ds/Vec.h"
#include <assert.h>

class Config {
private:
	void read_double_val(std::istringstream &instream, Vec &vec, const int size) {
		assert(size == 2 || size == 3);
		double val1, val2, val3;
		instream >> val1 >> val2;
		vec.push_back(val1);
		vec.push_back(val2);
		if (size == 3) {
			instream >> val3;
			vec.push_back(val3);
		}
	}
	void read_lights(std::ifstream &input, std::vector<Light> &lights, const int light_num) {
		std::string line, word;
		for (int i = 0; i < light_num; i++) {
			std::string type;
			Vec center;
			Vec luminance;
			getline(input, line);
			std::istringstream instream(line);
			for (int j = 0; j < 3; j++) {
				instream >> word;
				if (word == "type") {
					instream >> type;
				}
				if (word == "center") {
					read_double_val(instream, center, 3);
				}
				if (word == "luminance") {
					read_double_val(instream, luminance, 3);
				}
			}
			if (type == "sphere") {
				double radius;
				instream >> word;
				if (word == "radius") {
					instream >> radius;
				}
				lights.push_back(Light(type, center, radius, luminance));
				continue;
			}
			if (type == "quad") {
				Vec norm, x_axis, size;
				for (int j = 0; j < 3; j++) {
					instream >> word;
					if (word == "norm") {
						read_double_val(instream, norm, 3);
					}
					if (word == "x_axis") {
						read_double_val(instream, x_axis, 3);
					}
					if (word == "size") {
						read_double_val(instream, size, 2);
					}
				}
				lights.push_back(Light(type, center, norm, x_axis, size, luminance));
			}
		}
	}
public:
	std::string dir;
	std::string obj_name;
	int width, height, samples, depth, iteration, pixel_division_h, pixel_division_w;
	Vec camera;
	Vec lookat;
	Vec up;
	double fovy, fovx;
	std::vector<Light> lights;
	int light_num;

	Config(std::string _dir, std::string _obj_name) {
		dir = _dir;
		obj_name = _obj_name;
		std::ifstream input(dir + "/" + obj_name + ".config");
		std::string line, word;
		while (getline(input, line)) {
			std::istringstream instream(line);
			instream >> word;
			if (word == "width") {
				instream >> width;
			}
			if (word == "height") {
				instream >> height;
			}
			if (word == "pixel_division_h") {
				instream >> pixel_division_h;
			}
			if (word == "pixel_division_w") {
				instream >> pixel_division_w;
			}
			if (word == "samples") {
				instream >> samples;
			}
			if (word == "depth") {
				instream >> depth;
			}
			if (word == "iteration") {
				instream >> iteration;
			}
			if (word == "camera") {
				read_double_val(instream, camera, 3);
			}
			if (word == "lookat") {
				read_double_val(instream, lookat, 3);
			}
			if (word == "up") {
				read_double_val(instream, up, 3);
			}
			if (word == "fovy") {
				instream >> fovy;
				fovy = fovy * PI / 180;
			}
			if (word == "lights") {
				instream >> light_num;
				read_lights(input, lights, light_num);
			}
		}
		fovx = 2 * atan(width / height * tan(fovy / 2));
	}
};

#endif