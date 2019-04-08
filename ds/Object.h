#ifndef __OBJECT_H__
#define __OBJECT_H__

#include "Vec.h"
#include "PointCloud.h"
#include "../kdtree/kdtree.h"

#include <iostream>

class Object {
public:
	size_t f_num;
	std::vector<std::vector<int>> f_mat;
	std::vector<std::vector<int>> ft_mat;
	std::vector<std::vector<int>> fn_mat;
	//std::vector<double> f_area;
	std::vector<AABB_3D*> bounding_boxs;
	SAH_KDTree_3D* kdtree;
	std::string mat_name;
	Vec center;
	Object() {
		f_num = 0;
		kdtree = nullptr;
	}
	/*void compute_area(PointCloud& points) {
		for (int i = 0; i < f_num; i++) {
			Vec center({ 0.0,0.0,0.0 });
			for (int j = 0; j < f_mat[i].size(); j++) {
				center = center + points.v_mat[f_mat[i][j]];
			}
			center = center / f_mat[i].size();
			double area = 0;
			Vec edge0 = points.v_mat[f_mat[i][0]] - center;
			for (int j = 0, sz = f_mat[i].size(); j < sz; j++) {
				Vec edge1 = points.v_mat[f_mat[i][(j + 1) % sz]] - center;
				area += edge0.cross(edge1).norm();
			}
			area = area > 0 ? area / 2 : -area / 2;
			f_area.push_back(area);
		}
	}*/
	/*void compute_center(PointCloud& points) {
		center = Vec({ 0.0, 0.0, 0.0 });
		int num = 0;
		for (int i = 0; i < f_num; i++) {
			for (int j = 0, sz = f_mat[i].size(); j < sz; j++) {
				center = center + points.v_mat[f_mat[i][j]];
				num++;
			}
		}
		center = center / num;
	}*/
	void compute_bounding_boxs(PointCloud& points, const int obj_idx) {
		for (int i = 0; i < f_num; i++) {
			bounding_boxs.push_back(new AABB_3D(points, f_mat, obj_idx, i));
		}
	}
	void build_kdtree() {
		kdtree = new SAH_KDTree_3D(bounding_boxs, 0, true);
	}
	~Object() {
		for (int i = 0, sz = bounding_boxs.size(); i < sz; i++) {
			delete bounding_boxs[i];
		}
		if (kdtree != nullptr) delete kdtree;
	}
};

#endif