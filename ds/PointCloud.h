#ifndef __POINTCLOUD_H__
#define __POINTCLOUD_H__

#include "Vec.h"

class PointCloud {
public:
	size_t v_num, vn_num, vt_num;
	std::vector<Vec> v_mat;
	std::vector<Vec> vn_mat;
	std::vector<Vec> vt_mat;
	PointCloud() {
		v_num = 0;
		vn_num = 0;
		vt_num = 0;
	}
};

#endif