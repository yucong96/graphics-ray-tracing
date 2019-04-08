#include <vector>
#include <random>

#include "../ds/Vec.h"
#include "../ds/Light.h"
#include "../ds/Material.h"
#include "../ds/Object.h"
#include "../ds/Scene.h"
#include "../kdtree/kdtree.h"

#define PI (3.1415926)

// SAH_KDTree_3D build_kd_tree(Scene& scene);
Vec ray_tracing(Scene& scene, std::vector<Light>& lights, Ray& ray, std::default_random_engine& rand_engine,
	Vec& inter_point, int& inter_obj_idx, int& inter_face_idx, Vec& norm_vec, int depth, int max_depth);