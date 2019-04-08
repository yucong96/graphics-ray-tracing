#include "ray-tracing.h"

#include <time.h>
#include <assert.h>

using namespace std;

#define EPS 1e-6
#define EPS_VEC (Vec({EPS, EPS, EPS}))
const double max_face_area = 1.0;


double intersect(Scene& scene, AABB_3D* box, Ray& ray, Vec& inter_point) {
	vector<Vec> face;
	Object& obj = scene.objs[box->obj_idx];
	vector<int>& face_conn = obj.f_mat[box->face_idx];
	for (int i = 0, sz = face_conn.size(); i < sz; i++) {
		face.push_back(scene.points.v_mat[face_conn[i]]);
	}
	inter_point = box->intersection(ray);
	if (inter_point.size() == 3) {
		Vec edge0 = face[0] - inter_point;
		double winding_number = 0;
		for (int i = 0, sz = face.size(); i < sz; i++) {
			Vec edge1 = face[(i + 1) % sz] - inter_point;
			double cos_val = edge0.dot(edge1) / (edge0.norm()*edge1.norm());
			cos_val = cos_val > 1 ? 1 : (cos_val < -1 ? -1 : cos_val);
			winding_number += acos(cos_val);
			edge0 = edge1;
		}
		if (winding_number < 2 * PI + 1e-2 && winding_number > 2 * PI - 1e-2) {
			return (inter_point - ray.origin).norm();
		}
	}
	return -1;
}

bool intersect(Scene& scene, vector<AABB_3D*> boxs, Ray& ray, Vec& inter_point, double &nearest_length, int &nearest_idx) {
	nearest_idx = -1;
	nearest_length = 1e10;
	for (int i = 0, sz = boxs.size(); i < sz; i++) {
		Vec temp_inter_point;
		double length = intersect(scene, boxs[i], ray, temp_inter_point);
		if (length < 0) continue;
		if (length < nearest_length) {
			nearest_length = length;
			nearest_idx = i;
			inter_point = temp_inter_point;
		}
	}
	if (nearest_idx < 0) return false;
	else return true;
}

void build_cordinate(Vec &origin, Vec &z_axis, Vec &x_axis, Vec &y_axis) {
	z_axis.normalize();
	if (x_axis.norm() == 0) {
		double d = -(z_axis[0] * origin[0] + z_axis[1] * origin[1] + z_axis[2] * origin[2]); // a,b,c,d that specify a plane
		Vec a_pt_in_plane; {
			if (z_axis[0] != 0) {
				a_pt_in_plane = Vec({ -d / z_axis[0], 0, 0 });
			}
			else if (z_axis[1] != 0) {
				a_pt_in_plane = Vec({ 0, -d / z_axis[1], 0 });
			}
			else {
				a_pt_in_plane = Vec({ 0, 0, -d / z_axis[2] });
			}
		}
		x_axis = a_pt_in_plane - origin;
		if (x_axis.norm() < EPS) {
			if (z_axis[0] != 0) {
				a_pt_in_plane = Vec({ -d - z_axis[1] - z_axis[2] / z_axis[0], 1, 1 });
			}
			else if (z_axis[1] != 0) {
				a_pt_in_plane = Vec({ 1, -d - z_axis[1] - z_axis[2] / z_axis[0], 1 });
			}
			else {
				a_pt_in_plane = Vec({ -d - z_axis[1] - z_axis[2] / z_axis[0], 1, 1 });
			}
			x_axis = a_pt_in_plane - origin;
		}
	}
	x_axis.normalize();
	y_axis = z_axis.cross(x_axis);
	assert(x_axis.dot(y_axis) < EPS);
	assert(x_axis.dot(z_axis) < EPS);
	assert(y_axis.dot(z_axis) < EPS);
}

Vec random_pick_sphere_point(default_random_engine& rand_engine, Vec &origin, Vec &norm, double radius, bool add_origin = true, double z_angle = PI / 2) {
	Vec x_axis, y_axis;
	build_cordinate(origin, norm, x_axis, y_axis);

	double x_val, y_val, z_val; {
		double bound = sin(z_angle);
		uniform_real_distribution<double> u1(-bound, bound);
		x_val = u1(rand_engine);
		bound = sqrt(bound*bound - x_val*x_val);
		uniform_real_distribution<double> u2(-bound, bound);
		y_val = u2(rand_engine);
		z_val = sqrt(1 - x_val*x_val - y_val*y_val);
	}

	if (add_origin) return (x_axis*x_val + y_axis*y_val + norm*z_val)*radius + origin;
	else return x_axis*x_val + y_axis*y_val + norm*z_val; // only a direction
}

Vec random_pick_rect_point(default_random_engine& rand_engine, Vec &origin, Vec &norm, Vec &x_axis, double x_length, double y_length) {
	Vec y_axis;
	build_cordinate(origin, norm, x_axis, y_axis);
	
	double x_val, y_val; {
		uniform_real_distribution<double> u(-1, 1);
		x_val = u(rand_engine);
		y_val = u(rand_engine);
	}
	
	return x_axis*x_val * (x_length/2.0) + y_axis*y_val * (y_length/2.0) + origin;
}

vector<AABB_3D*> intersect_bounding_boxs(Scene& scene, Ray& ray) {
	ray.origin = ray.origin + ray.direction * EPS;
	vector<AABB_3D*> inter_bbs;
	for (int i = 0, sz = scene.objs.size(); i < sz; i++) {
		vector<AABB_3D*> bbs = scene.objs[i].kdtree->intersect(ray);
		for (auto bb = bbs.begin(); bb != bbs.end(); bb++) {
			inter_bbs.push_back(*bb);
		}
	}
	return inter_bbs;
}

Vec ray_tracing(Scene& scene, vector<Light>& lights, Ray& ray, std::default_random_engine& rand_engine,
	Vec& inter_point, int& inter_obj_idx, int& inter_face_idx, Vec& norm_vec,
	int depth, int max_depth) {
	
	if (depth >= max_depth) return Vec({ 0.0, 0.0, 0.0 });
		
	ray.origin = ray.origin + ray.direction * EPS;
	vector<AABB_3D*> inter_bbs = intersect_bounding_boxs(scene, ray);
	double length;
	int inter_box_idx;
	
	if (intersect(scene, inter_bbs, ray, inter_point, length, inter_box_idx)) {
		AABB_3D* box = inter_bbs[inter_box_idx];
		inter_obj_idx = box->obj_idx;
		Object& obj = scene.objs[box->obj_idx];
		inter_face_idx = box->face_idx;
		PointCloud& points = scene.points;
		Material& mtl = scene.mats[obj.mat_name];
		// Blin-Phong: Ie = Ka*Ia + [Kd*(N*L) + Ks*(H*N)^Ns]*Ii
		Vec view_vec; {
			view_vec = ray.direction*(-1);
			view_vec.normalize();
		}
		int sign = 1; // sign==1: outer surface; sign==-1: inner surface
		Vec plane_norm_vec = Vec({ 0.0,0.0,0.0 });
		norm_vec = Vec({ 0.0,0.0,0.0 }); {
			for (int i = 0, face_sz = obj.fn_mat[box->face_idx].size(); i < face_sz; i++) {
				double weight_on_vertex = 1 / ((points.v_mat[obj.f_mat[box->face_idx][i]] - inter_point).norm() + EPS);
				norm_vec = norm_vec + points.vn_mat[obj.fn_mat[box->face_idx][i]] * weight_on_vertex;
				plane_norm_vec = plane_norm_vec + points.vn_mat[obj.fn_mat[box->face_idx][i]];
			}
			norm_vec.normalize();
			plane_norm_vec.normalize();
			if (view_vec.dot(norm_vec) < 0.0 && view_vec.dot(plane_norm_vec) < 0.0) {
				sign = -1;
				norm_vec = norm_vec * (-1);
			}
		}
		vector<Vec> light_vecs; {
			for (int i = 0, sz = lights.size(); i < sz; i++) {
				Vec light_point; {
					if (lights[i].type == "sphere") {
						light_point = random_pick_sphere_point(rand_engine, lights[i].center, inter_point - lights[i].center, lights[i].radius, true);
					}
					else if (lights[i].type == "quad") {
						light_point = random_pick_rect_point(rand_engine, lights[i].center, lights[i].norm, lights[i].x_axis, lights[i].size[0], lights[i].size[1]);
					}
					else {
						assert(0);
					}
				}
				Vec light_vec = light_point - inter_point;
				double light_distance = light_vec.norm();
				light_vec.normalize();
				// check if direct light is occluded
				Ray light_ray(inter_point, light_vec);
				vector<AABB_3D*> in_light_ray_boxs = intersect_bounding_boxs(scene, light_ray);
				Vec light_point2; // useless
				double light_distance2;
				int nearest_idx_for_light; // useless
				intersect(scene, in_light_ray_boxs, light_ray, light_point2, light_distance2, nearest_idx_for_light);
				if (light_distance2 < light_distance*0.9) {
					light_vecs.push_back(Vec({ 0.0, 0.0, 0.0 }));
				}
				else {
					light_vecs.push_back(light_vec);
				}
			}
		}
		vector<Vec> heuristic_vecs; {
			for (int i = 0, sz = light_vecs.size(); i < sz; i++) {
				heuristic_vecs.push_back((light_vecs[i] + view_vec) * 0.5);
				heuristic_vecs[i].normalize();
			}
		}
		// ambient part
		Vec ambient = mtl.Ka;
		// if (depth == 0 && ambient.norm() > 1) cout << "in depth " << depth << ", ambient: " << ambient << endl;
		// direct light part
		Vec direct_light({ 0.0, 0.0, 0.0 }); {
			for (int i = 0, sz = light_vecs.size(); i < sz; i++) {
				if (light_vecs[i].norm() > 1e-6) {
					Ray direct_light_ray(inter_point, light_vecs[i]);
					Vec light_direction = lights[i].center - inter_point;
					double light_distance = light_direction.norm();
					Vec light_emission;
					double tolerate_angle;
					if (lights[i].type == "sphere") {
						light_emission = lights[i].Le / ((light_distance / lights[i].radius) * (light_distance / lights[i].radius));
						tolerate_angle = asin(lights[i].radius / light_distance);
					}
					else if (lights[i].type == "quad") {
						light_emission = lights[i].Le / (2*PI*light_distance*light_distance / (lights[i].size[0] * lights[i].size[0]));
						tolerate_angle = asin(lights[i].size[0] / light_distance);
					}
					Vec light_on_mtl_diff = mtl.Kd * norm_vec.dot(light_vecs[i]) * light_emission;
					Vec light_on_mtl_spec;
					double cos_val = norm_vec.dot(heuristic_vecs[i]);
					cos_val = cos_val > 1 ? 1 : cos_val;
					if (acos(cos_val) > tolerate_angle / 2) {
						light_on_mtl_spec = mtl.Ks * light_emission * pow(cos_val, mtl.Ns);
					}
					else {
						light_on_mtl_spec = mtl.Ks * light_emission;
					}
					Vec light_on_mtl = light_on_mtl_diff + light_on_mtl_spec;
					light_on_mtl.to_positive();
					direct_light = direct_light + light_on_mtl;
				}
			}
			direct_light = (mtl.Tf + EPS_VEC) * direct_light;
			// if (depth == 0 && ambient.norm() > 1) cout << "in depth " << depth << ", direct_light: " << direct_light << endl;
		}
		// diffuse part
		Vec diffuse({ 0.0, 0.0, 0.0 });
		if (mtl.Kd.norm() > 1e-6) {
			Ray diff_ray(inter_point, random_pick_sphere_point(rand_engine, inter_point, norm_vec, 1.0, false));
			Vec diff_point, diff_norm;
			int diff_obj_idx, diff_face_idx;
			Vec diff_color = ray_tracing(scene, lights, diff_ray, rand_engine, diff_point, diff_obj_idx, diff_face_idx, diff_norm, depth + 1, max_depth);
			if (diff_color.norm() > EPS) {
				Object& diff_obj = scene.objs[diff_obj_idx];
				double diff_distance = (diff_point - inter_point).norm();
				Vec diff_emission = diff_color; // *diff_norm.dot(diff_ray.direction*(-1));
				diffuse = (mtl.Tf + EPS_VEC) * mtl.Kd * norm_vec.dot(diff_ray.direction) * diff_emission;
			}
			diffuse.to_positive();
			// if (depth == 0 && ambient.norm() > 1) cout << "in depth " << depth << ", diffuse: " << diffuse << endl;
		}
		// specular part
		Vec specular({ 0.0, 0.0, 0.0 });
		if (mtl.Ks.norm() > 1e-6) {
			Ray spec_ray(inter_point, norm_vec * (2*view_vec.dot(norm_vec)) - view_vec);
			Vec spec_point, spec_norm;
			int spec_obj_idx, spec_face_idx;
			Vec spec_color = ray_tracing(scene, lights, spec_ray, rand_engine, spec_point, spec_obj_idx, spec_face_idx, spec_norm, depth + 1, max_depth);
			if (spec_color.norm() > EPS) {
				specular = (mtl.Tf + EPS_VEC) * mtl.Ks * spec_color;
			}
			specular.to_positive();
			/*for (int i = 0; i < 200; i++) {
				Vec other_spec_direction = random_pick_sphere_point(rand_engine, inter_point, norm_vec, 1.0, false, PI/180*30);
				Ray other_spec_ray(inter_point, other_spec_direction);
				Vec other_spec_color = ray_tracing(scene, lights, other_spec_ray, rand_engine, spec_point, spec_obj_idx, spec_face_idx, spec_norm, (depth+1<max_depth-1) ? max_depth-1 : depth+1, max_depth);
				double other_spec_rate = pow(norm_vec.dot((view_vec + other_spec_direction) / 2), mtl.Ns);
				Vec other_specular = (mtl.Tf + EPS_VEC) * mtl.Ks * other_spec_rate * other_spec_color;
				other_specular.to_positive();
				if (other_specular.norm() > specular.norm()) {
					specular = other_specular;
				}
			}*/
			// if (depth == 0 && specular.norm() > 0.25 && obj.mat_name == "breakfast_room_cup_Material_005") cout << "in depth " << depth << ", specular: " << specular << endl;
		}
		// refraction part
		Vec refraction({ 0.0, 0.0, 0.0 });
		if ((Vec({ 1,1,1 }) - mtl.Tf).norm() > EPS) {
			double n = mtl.Ni; // refraction rate
			if (sign == -1) {
				n = 1 / n;
			}
			double cos_alpha = view_vec.dot(norm_vec);
			cos_alpha = cos_alpha > 1 ? 1 : cos_alpha;
			double alpha = acos(cos_alpha);
			double sin_beta = sin(alpha) / n;
			bool total_reflection = false;
			if (sin_beta >= 1.0) {
				total_reflection = true;
			}

			Vec refra_point, refra_norm;
			int refra_obj_idx, refra_face_idx;
			if (total_reflection) {
				Ray refra_ray(inter_point, norm_vec * (2 * view_vec.dot(norm_vec)) - view_vec);
				refraction = ray_tracing(scene, lights, refra_ray, rand_engine, refra_point, refra_obj_idx, refra_face_idx, refra_norm, depth + 1, max_depth);
			} else {
				double beta = asin(sin_beta);
				Vec y_axis = norm_vec*(view_vec.dot(norm_vec))*(-1);
				Vec x_axis = (view_vec + y_axis)*(-1);
				y_axis.normalize();
				x_axis.normalize();
				assert(x_axis.dot(y_axis) < EPS);
				Ray refra_ray(inter_point, x_axis * sin(beta) + y_axis * cos(beta));
				refraction = (Vec({ 1,1,1 }) - mtl.Tf) * ray_tracing(scene, lights, refra_ray, rand_engine, refra_point, refra_obj_idx, refra_face_idx, refra_norm, depth, max_depth);
			}
			refraction.to_positive();
			// if (depth == 0) cout << "in depth " << depth << ", refraction: " << refraction << " sign: " << sign << endl;
		}
		// total
		Vec final_color = ambient + direct_light + diffuse + specular + refraction;
		return final_color;
	}
	else {
		return Vec({ 0.0, 0.0, 0.0 });
	}
}