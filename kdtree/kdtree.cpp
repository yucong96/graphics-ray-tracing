#include "kdtree.h"
#include <algorithm>
#include <assert.h>

using namespace std;

double EPS = 1e-6;

void AABB_3D::set_face(std::vector<Vec>& face) {
	for (int i = 0; i < 3; i++) {
		bound_max[i] = -1e10;
		bound_min[i] = 1e10;
		for (int j = 0, sz = face.size(); j < sz; j++) {
			if (face[j][i] < bound_min[i]) {
				bound_min[i] = face[j][i];
			}
			if (face[j][i] > bound_max[i]) {
				bound_max[i] = face[j][i];
			}
		}
		center[i] = (bound_max[i] + bound_min[i]) / 2;
	}
	compute_plane(face);
}

void AABB_3D::compute_plane(vector<Vec>& face) {
	assert(face.size() >= 3);
	Vec edge1 = face[1] - face[0];
	Vec edge2 = face[2] - face[0];
	Vec norm = edge1.cross(edge2);
	assert(norm.norm() > EPS);
	norm.normalize();
	a = norm[0];
	b = norm[1];
	c = norm[2];
	d = -(a*face[0][0] + b*face[0][1] + c*face[0][2]);
}

AABB_3D::AABB_3D(vector<Vec>& face) {
	set_face(face);
}

/*AABB_3D::AABB_3D(Scene& scene, const int _obj_idx, const int _face_idx) {
	obj_idx = _obj_idx;
	face_idx = _face_idx;
	vector<Vec> face;
	Object& obj = scene.objs[obj_idx];
	vector<int>& face_conn = obj.f_mat[face_idx];
	for (int i = 0, sz = face_conn.size(); i < sz; i++) {
		face.push_back(scene.points.v_mat[face_conn[i]]);
	}
	set_face(face);
}*/

AABB_3D::AABB_3D(PointCloud& points, vector<vector<int>> face_conns, const int _obj_idx, const int _face_idx) {
	obj_idx = _obj_idx;
	face_idx = _face_idx;
	vector<Vec> face;
	vector<int>& face_conn = face_conns[face_idx];
	for (int i = 0, sz = face_conn.size(); i < sz; i++) {
		face.push_back(points.v_mat[face_conn[i]]);
	}
	set_face(face);
}

Vec AABB_3D::intersection(Ray& ray) {
	double t = -(a*ray.origin[0] + b*ray.origin[1] + c*ray.origin[2] + d) / (a*ray.direction[0] + b*ray.direction[1] + c*ray.direction[2]);
	return t > 0 ? ray.origin + ray.direction * t : Vec();
}

SAH_KDTree_3D::SAH_KDTree_3D(vector<AABB_3D*> &bounding_boxs, const int dimension, const bool is_root) {
	assert(bounding_boxs.size() > 0);

	if (bounding_boxs.size() == 1) {
		child = bounding_boxs[0];
		left_tree = nullptr;
		right_tree = nullptr;
		for (int i = 0; i < 3; i++) {
			bound_max[i] = child->bound_max[i];
			bound_min[i] = child->bound_min[i];
		}
		return;
	}
	else {
		child = nullptr;
	}

	for (int i = 0; i < 3; i++) {
		bound_min[i] = 1e10;
		bound_max[i] = -1e10;
	}
	for (int i = 0, sz = bounding_boxs.size(); i < sz; i++) {
		for (int j = 0; j < 3; j++) {
			if (bounding_boxs[i]->bound_min[j] < bound_min[j]) {
				bound_min[j] = bounding_boxs[i]->bound_min[j];
			}
			if (bounding_boxs[i]->bound_max[j] > bound_max[j]) {
				bound_max[j] = bounding_boxs[i]->bound_max[j];
			}
		}
	}
	
	vector<double> split_candidates; {
		for (int i = 0, sz = bounding_boxs.size(); i < sz; i++) {
			split_candidates.push_back(bounding_boxs[i]->center[dimension]);
		}
		sort(split_candidates.begin(), split_candidates.end(), [](double a, double b) -> bool { return a < b; });
	}
	double split = split_candidates[bounding_boxs.size()/2] - EPS;
	double left_r_bound = split, right_l_bound = split;
	//double left_tree_bound_min[3], left_tree_bound_max[3], right_tree_bound_min[3], right_tree_bound_max[3];
	vector<AABB_3D*> left_bboxs, right_bboxs; {
		for (int i = 0, sz = bounding_boxs.size(); i < sz; i++) {
			if (i < sz / 2) {
				left_bboxs.push_back(bounding_boxs[i]);
				/*if (bounding_boxs[i]->bound_max[dimension] > left_r_bound) {
					left_r_bound = bounding_boxs[i]->bound_max[dimension];
				}*/
			}
			else {
				right_bboxs.push_back(bounding_boxs[i]);
				/*if (bounding_boxs[i]->bound_min[dimension] > right_l_bound) {
					right_l_bound = bounding_boxs[i]->bound_min[dimension];
				}*/
			}
		}
	}
	
	/*for (int i = 0; i < 3; i++) {
		if (i == dimension) {
			left_tree_bound_min[i] = bound_min[i];
			left_tree_bound_max[i] = left_r_bound;
			right_tree_bound_min[i] = right_l_bound;
			right_tree_bound_max[i] = bound_max[i];
		}
		else {
			left_tree_bound_min[i] = bound_min[i];
			left_tree_bound_max[i] = bound_max[i];
			right_tree_bound_min[i] = bound_min[i];
			right_tree_bound_max[i] = bound_max[i];
		}
	}*/
	left_tree = new SAH_KDTree_3D(left_bboxs, (dimension + 1) % 3);
	right_tree = new SAH_KDTree_3D(right_bboxs, (dimension + 1) % 3);
}

bool SAH_KDTree_3D::is_intersect(const Ray &ray) {
	for (int i = 0; i < 3; i++) {
		int ii = (i + 1) % 3, iii = (i + 2) % 3;
		double rate = (bound_min[i] - ray.origin[i]) / ray.direction[i];
		if (rate < 0) continue;
		double x2 = ray.origin[ii] + rate * ray.direction[ii];
		double x3 = ray.origin[iii] + rate * ray.direction[iii];
		if (x2 > bound_min[ii] - EPS && x2 < bound_max[ii] + EPS && x3 > bound_min[iii] - EPS && x3 < bound_max[iii] + EPS) return true;
	}
	for (int i = 0; i < 3; i++) {
		int ii = (i + 1) % 3, iii = (i + 2) % 3;
		double rate = (bound_max[i] - ray.origin[i]) / ray.direction[i];
		if (rate < 0) continue;
		double x2 = ray.origin[ii] + rate * ray.direction[ii];
		double x3 = ray.origin[iii] + rate * ray.direction[iii];
		if (x2 > bound_min[ii] && x2 < bound_max[ii] && x3 > bound_min[iii] && x3 < bound_max[iii]) return true;
	}
	return false;
}

vector<AABB_3D*> SAH_KDTree_3D::intersect(const Ray &ray) {
	vector<AABB_3D*> result;
	if (child != nullptr) {
		if (is_intersect(ray)) {
			result.push_back(child);
		}
		return result;
	}

	if (is_intersect(ray)) {
		vector<AABB_3D*> left_result, right_result;
		left_result = left_tree->intersect(ray);
		right_result = right_tree->intersect(ray);
		for (int i = 0, sz = left_result.size(); i < sz; i++) {
			result.push_back(left_result[i]);
		}
		for (int i = 0, sz = right_result.size(); i < sz; i++) {
			result.push_back(right_result[i]);
		}
	}
	return result;
}