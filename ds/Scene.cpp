#include "Scene.h"

#include <fstream>
#include <sstream>
#include <assert.h>

using namespace std;

#define REPORT_LOCATION __FILE__ << " " << __LINE__ << ": "

inline bool is_line_invalid(const string &line) {
	return (line.empty() || line[0] == 13 || line[0] == '#');
}

inline void report_err(const string &info) {
	cerr << "Error in " << REPORT_LOCATION << info << endl;
	assert(0);
}

void Scene::set_mtl(const std::string &dir, const std::string &mtl_file) {
	ifstream input(dir + "/" + mtl_file);
	string line, word, name;
	double val1, val2, val3;
	while (getline(input, line)) {
		if (is_line_invalid(line)) continue;
		istringstream instream(line);
		instream >> word;
		if (word == "newmtl") {
			instream >> name;
			Material mat;
			mats.insert({ name, mat });
			continue;
		}
		else if (word == "illum") {
			instream >> mats[name].illum;
		}
		else if (word == "Kd") {
			instream >> val1 >> val2 >> val3;
			mats[name].Kd = Vec({ val1,val2,val3 });
		}
		else if (word == "Ka") {
			instream >> val1 >> val2 >> val3;
			mats[name].Ka = Vec({ val1,val2,val3 });
		}
		else if (word == "Tf") {
			instream >> val1 >> val2 >> val3;
			mats[name].Tf = Vec({ val1,val2,val3 });
		}
		else if (word == "Ni") {
			instream >> mats[name].Ni;
			continue;
		}
		else if (word == "Ks") {
			instream >> val1 >> val2 >> val3;
			mats[name].Ks = Vec({ val1,val2,val3 });
		}
		else if (word == "Ns") {
			instream >> mats[name].Ns;
		}
		else {
			report_err("Error: In MTL file, find unexpected parameter");
		}
	}
}

void get_from_face_data(string str, int &f_val, int &ft_val, int &fn_val) {
	string f_str = str.substr(0, str.find('/'));
	string ft_str = str.substr(str.find('/') + 1, str.rfind('/'));
	string fn_str = str.substr(str.rfind('/') + 1, str.length());
	if (f_str.length() > 0) {
		istringstream iss1(f_str);
		iss1 >> f_val;
	}
	if (ft_str.length() > 0) {
		istringstream iss2(ft_str);
		iss2 >> ft_val;
	}
	if (fn_str.length() > 0) {
		istringstream iss3(fn_str);
		iss3 >> fn_val;
	}
}

void Scene::set(const string &dir, const string &obj_file) {
	ifstream input(dir + "/" + obj_file);
	string line, word;
	int points_idx = -1, obj_idx = -1;
	bool setting_obj_flag = false;
	while (getline(input, line)) {
		double val1, val2, val3;
		if (is_line_invalid(line)) continue;
		istringstream instream(line);
		instream >> word;
		if (word == "mtllib") {
			setting_obj_flag = false;
			string mtl_file;
			instream >> mtl_file;
			set_mtl(dir, mtl_file);
			continue;
		}
		else if (word == "g" || word == "G" || word == "s" || word == "S") {
			continue;
		}
		else if (word == "v" || word == "V") {
			setting_obj_flag = false;
			instream >> val1 >> val2 >> val3;
			points.v_num++;
			points.v_mat.push_back({ val1, val2, val3 });
			continue;
		}
		else if (word == "vt" || word == "VT") {
			setting_obj_flag = false;
			instream >> val1 >> val2 >> val3;
			points.vt_num++;
			points.vt_mat.push_back({ val1, val2, val3 });
			continue;
		}
		else if (word == "vn" || word == "VN") {
			setting_obj_flag = false;
			instream >> val1 >> val2 >> val3;
			points.vn_num++;
			points.vn_mat.push_back({ val1, val2, val3 });
			continue;
		}
		else if (word == "usemtl" || word == "USEMTL") {
			obj_idx++;
			setting_obj_flag = true;
			objs.push_back(Object());
			instream >> objs[obj_idx].mat_name;
			continue;
		}
		else if (word == "f" || word == "F") {
			if (!setting_obj_flag) {
				obj_idx++;
				setting_obj_flag = true;
				objs.push_back(Object());
			}
			int f_val = 0, ft_val = 0, fn_val = 0;
			vector<int> f_vec, ft_vec, fn_vec;
			while (true) {
				string face_data;
				instream >> face_data;
				if (face_data.empty()) break;
				get_from_face_data(face_data, f_val, ft_val, fn_val);
				f_vec.push_back(f_val-1);
				ft_vec.push_back(ft_val-1);
				fn_vec.push_back(fn_val-1);
			}
			objs[obj_idx].f_num++;
			objs[obj_idx].f_mat.push_back(f_vec);
			objs[obj_idx].ft_mat.push_back(ft_vec);
			objs[obj_idx].fn_mat.push_back(fn_vec);
			continue;
		}
		else {
			report_err("Error: In OBJ file, find unexpected parameter");
		}
	}
}

void Scene::clear() {
	vector<Object>().swap(objs);
	map<string, Material>().swap(mats);
}