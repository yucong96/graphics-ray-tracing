#ifndef __SCENE_H__
#define __SCENE_H__

#include <vector>
#include <string>
#include <map>
#include "Object.h"
#include "Material.h"

class Scene {
private:
	void set_mtl(const std::string &dir, const std::string &mtl_file);
public:
	PointCloud points;
	std::vector<Object> objs;
	std::map<std::string, Material> mats;
	void set(const std::string &dir, const std::string &obj_file);
	void clear();
};

#endif