#ifndef __KDTREE_H__
#define __KDTREE_H__

#include <vector>
#include "../ds/Vec.h"
#include "../ds/Ray.h"
#include "../ds/PointCloud.h"

// Axis Aligned Bounding Box
class AABB_3D {
private:
	void set_face(std::vector<Vec>& face);
	void compute_plane(std::vector<Vec>& face);
public:
	double center[3];
	double bound_max[3];
	double bound_min[3];
	int obj_idx, face_idx;
	double a, b, c, d;
	AABB_3D(std::vector<Vec>& face);
	AABB_3D::AABB_3D(PointCloud& points, std::vector<std::vector<int>> face_conns, const int _obj_idx, const int _face_idx);
	Vec intersection(Ray& ray);
};

class SAH_KDTree_3D {
private:
	bool is_intersect(const Ray &ray);
public:
	double bound_max[3];
	double bound_min[3];
	SAH_KDTree_3D *left_tree, *right_tree;
	AABB_3D* child;
	SAH_KDTree_3D(std::vector<AABB_3D*> &bounding_boxs, const int dimension, const bool is_root = false);
	std::vector<AABB_3D*> intersect(const Ray &ray);
};

#endif