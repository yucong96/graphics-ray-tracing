#ifndef __MATERIAL_H__
#define __MATERIAL_H__

class Material {
	// Phone: Ie = Ka*Ia + [Kd*(N*L) + Ks*(V*R)^Ns]*Ii
	// Blin-Phone: Ie = Ka*Ia + [Kd*(N*L) + Ks*(H*N)^Ns]*Ii
	// N: �����淨��
	// V: �ӽǷ���
	// L: ��Դ����
	// R: ��Դ���䷽��
	// H: V��L�м䷽��
public:
	int illum; // ����ģ��
	Vec Kd; // Kd diffuse ɢ��� 0.0-1.0
	Vec Ka; // Ka ambient ������ 
	Vec Tf; // Tf �˹�͸���� 0.0-1.0 0.0��ʾ��ȫ͸�⣬1.0��ʾ��ȫ��͸�⣬�м����ǰ�͸��
	double Ni; // Ni ������ 0.1-10.0
	Vec Ks; // Ks specular ����� 0.0-1.0 ��ͨ����Ks+Kd=1 �ֱ��ʾɢ��ͷ���ı���
	double Ns; // Ns specular exponent ����߹�ֵ 0-1000
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