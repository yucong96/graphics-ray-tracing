#ifndef __MATERIAL_H__
#define __MATERIAL_H__

class Material {
	// Phone: Ie = Ka*Ia + [Kd*(N*L) + Ks*(V*R)^Ns]*Ii
	// Blin-Phone: Ie = Ka*Ia + [Kd*(N*L) + Ks*(H*N)^Ns]*Ii
	// N: 反射面法向
	// V: 视角方向
	// L: 光源方向
	// R: 光源反射方向
	// H: V和L中间方向
public:
	int illum; // 光照模型
	Vec Kd; // Kd diffuse 散射光 0.0-1.0
	Vec Ka; // Ka ambient 环境光 
	Vec Tf; // Tf 滤光透射率 0.0-1.0 0.0表示完全透光，1.0表示完全不透光，中间则是半透明
	double Ni; // Ni 折射率 0.1-10.0
	Vec Ks; // Ks specular 镜面光 0.0-1.0 （通常）Ks+Kd=1 分别表示散射和反射的比例
	double Ns; // Ns specular exponent 反射高光值 0-1000
	Material() {
		illum = 0;
		Kd = Vec({ 0.0,0.0,0.0 });
		Ka = Vec({ 0.0,0.0,0.0 });
		Ks = Vec({ 0.0,0.0,0.0 });
		Tf = Vec({ 0.0,0.0,0.0 });
		Ni = 1;
		Ns = 0;
	}
};

#endif