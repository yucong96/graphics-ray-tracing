#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <random>
#include <time.h>
#include "kdtree/kdtree.h"
#include "ds/Scene.h"
#include "ds/Vec.h"
#include "ds/Light.h"
#include "alg/ray-tracing.h"
#include "bmp.h"
#include <omp.h>

#include "Config.h"

void load_config(const std::string &dir, const std::string &obj_name,
	int& width, int& height, int &samples) {

}

int main(int argc, char* argv[]) {
	std::default_random_engine rand_engine;
	rand_engine.seed(time(0));

	std::string dir;
	std::string obj_name;
	if (argc != 3 && argc != 2) {
		std::cerr << "Error: format should be graphics-ray-tracing.exe <dir> <scene_name>" << std::endl;
		std::cerr << "or format should be graphics-ray-tracing.exe <code>, where 1 for room, 2 for cup, 3 for VeachMIS" << std::endl;
		return 1;
	}
	else if (argc == 2) {
		if (atoi(argv[1]) == 1) {
			dir = "model/room";
			obj_name = "room";
		}
		if (atoi(argv[1]) == 2) {
			dir = "model/cup";
			obj_name = "cup";
		}
		if (atoi(argv[1]) == 3) {
			dir = "model/VeachMIS";
			obj_name = "VeachMIS";
		}
	}
	else { // argc == 3
		dir = argv[1];
		obj_name = argv[2];
	}
	
	Config conf(dir, obj_name);

	// camera
	double z = 1.0;
	double real_width = z * tan(conf.fovx);
	double real_height = z * tan(conf.fovy);
	Vec direction = conf.lookat - conf.camera;
	direction.normalize();
	Vec width_axis = direction.cross(conf.up); // left to right
	Vec height_axis = width_axis.cross(direction); // bottom to up
	// load scene
	Scene scene;
	scene.set(dir, obj_name + ".obj");
	std::cout << "loading scene finished" << std::endl;
	// SAH_KDTree_3D kdtree = build_kd_tree(scene);
	for (int i = 0, sz = scene.objs.size(); i < sz; i++) {
		// scene.objs[i].compute_center(scene.points);
		scene.objs[i].compute_bounding_boxs(scene.points, i);
		scene.objs[i].build_kdtree();
	}
	std::cout << "build kd tree finished" << std::endl;
	// ray tracing
	Vec **color = new Vec*[conf.height], **color_final = new Vec*[conf.height]; {
		for (int h = 0; h < conf.height; h++) {
			color[h] = new Vec[conf.width];
			color_final[h] = new Vec[conf.width];
			for (int w = 0; w < conf.width; w++) {
				color[h][w] = Vec({ 0,0,0 });
				color_final[h][w] = Vec({ 0,0,0 });
			}
		}
	}
	for (int i = 0; i < conf.iteration; i++) {
		std::cout << i << std::endl;
		// ray tracing
		for (int h = 0; h < conf.height; h++) {
			std::cout << h << " ";
#pragma omp parallel for
			for (int w = 0; w < conf.width; w++) {
				for (int sub_h = 0; sub_h < conf.pixel_division_h; sub_h++) {
					for (int sub_w = 0; sub_w < conf.pixel_division_w; sub_w++) {
						for (int s = 0; s < conf.samples; s++) {
							Vec ray_direction = width_axis * (1.0 * w / conf.width + 1.0 * sub_w / conf.width / conf.pixel_division_w - 0.5) * real_width
								+ height_axis * (1.0 * h / conf.height + 1.0 * sub_h / conf.height / conf.pixel_division_h - 0.5) * real_height
								+ direction;
							Ray ray(conf.camera, ray_direction);
							Vec point, norm;
							int inter_obj_idx, inter_face_idx;
							Vec tracing_color = ray_tracing(scene, conf.lights, ray, rand_engine, point, inter_obj_idx, inter_face_idx, norm, 0, conf.depth);
							for (int i = 0; i < 3; i++) {
								tracing_color[i] = tracing_color[i] < 0.0 ? 0.0 : tracing_color[i];
								tracing_color[i] = tracing_color[i] > 1.0 ? 1.0 : tracing_color[i];
							}
							color[h][w] = color[h][w] + tracing_color;
						}
					}
				}
				color_final[h][w] = color[h][w] / (conf.pixel_division_h*conf.pixel_division_w*conf.samples*(i + 1));
			}
		}
		// save image
		mat2bmp(conf.width, conf.height, 3, color_final, "render/image_" + std::to_string(i) + ".bmp");
		/*std::ofstream output("render/image_" + std::to_string(i) + ".ppm");
		output << "P3" << std::endl << width << " " << height << std::endl << "255" << std::endl;
		for (int h = 0; h < height; h++) {
			for (int w = 0; w < width; w++) {
				for (int i = 0; i < 3; i++) {
					double color_0_1 = color[h][w][i] / (pixel_division_h*pixel_division_w*samples*(i + 1));
					color_0_1 = color_0_1 > 2.55 ? 2.55 : color_0_1;
					output << (int)(color_0_1 * 100) << " ";
				}
			}
			output << endl;
		}
		std::ofstream output("render/image_" + std::to_string(i) + ".ppm");
		output << "P3" << std::endl << 2 << " " << 2 << std::endl << "255" << std::endl;
		output << "255 0 0 0 255 0 0 0 255 255 255 255 ";
		output.close();*/
	}
}