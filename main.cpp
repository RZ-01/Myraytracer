#include <limits>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include "Geo.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION	
#define STB_IMAGE_IMPLEMENTATION
#define _USE_MATH_DEFINES
#include "stb_image_write.h"
#include "stb_image.h"
const int width = 1920;
const int height = 1080;
const int fov = M_PI / 2.0;
struct Light
{
	float intensity;
	Vec3f position;
	Light(const Vec3f &p, const float &i): intensity(i), position(p) {}
};
struct Material
{
	Vec3f diffuseColor;
	Vec4f coefficient;
	float specular_exp;
	float refractive_ind;
	Material(const float &r, const Vec4f& al, const Vec3f& color, const float& spec) : refractive_ind(r), coefficient(al), diffuseColor(color), specular_exp(spec)  {}
	Material() : refractive_ind(1), coefficient(1, 0, 0, 0), diffuseColor(), specular_exp() {}

};
struct Sphere
{
	Vec3f center;
	float radius;
	Material material;

	Sphere(const Vec3f &c, const float& r, const Material &m): center(c), radius(r), material(m) {}

	bool rayintersect(const Vec3f& origin, const Vec3f& direction, float& t0) const {
		Vec3f L = center - origin;
		float pc = L * direction;
		float d_2 = L * L - pc * pc;
		if (d_2 > radius * radius)
			return false;
		float dt = sqrtf(radius * radius - d_2);
		t0 = pc - dt;
		float t1 = pc + dt;
		if(t0 < 0)
			t0 = t1;
		if(t0 < 0)
			return false;
		return true;
	}
	

};
Vec3f* loadEnvironmentMap(const char* filename, int& width, int& height) {
	int n;
	unsigned char* data = stbi_load(filename, &width, &height, &n, 0);
	if (!data) {
		std::cerr << "Unable to load" << std::endl;
		return nullptr;
	}
	Vec3f* envMap = new Vec3f[width * height];
	for (int i = 0; i < width * height; ++i) {
		envMap[i] = Vec3f(data[i * n] / 255.0f, data[i * n + 1] / 255.0f, data[i * n + 2] / 255.0f);
	}
	stbi_image_free(data);
	return envMap;
}
Vec3f sampleEnvironmentMap(const Vec3f& dir, Vec3f* envMap, int width, int height) {
	float theta = acosf(dir.y);
	float phi = atan2f(dir.z, dir.x);
	if (phi < 0) phi += 2 * M_PI;

	float u = phi / (2 * M_PI);
	float v = theta / M_PI;

	int x = std::min(std::max(0, int(u * width)), width - 1);
	int y = std::min(std::max(0, int(v * height)), height - 1);

	return envMap[x + y * width];
}
Vec3f reflect(const Vec3f& I, const Vec3f& N) {
	return I - N * 2.0f * (I * N);
}
Vec3f refract(const Vec3f& I, const Vec3f& N, const float& refractive_index) { 
	float cosinei = -std::max(-1.f, std::min(1.f, I * N));
	float etai = 1, etat = refractive_index;
	Vec3f n = N;
	if (cosinei < 0){
		cosinei = -cosinei;
		std::swap(etai, etat); n = -N;
	}
	float eta = etai / etat;
	float k = 1 - eta * eta * (1.0 - cosinei * cosinei);
	if (k < 0)
		return Vec3f(0, 0, 0);
	return I * eta + n * (eta * cosinei - sqrtf(k));
}
bool SceneProcess(const Vec3f& ori, const Vec3f& dir, const std::vector<Sphere> &spheres, Vec3f &hitpoint, Vec3f&Normal, Material &material) {
	float spheres_d_min = std::numeric_limits<float>::max();
	for (size_t i = 0; i < spheres.size(); i++)
	{
		float spheres_d;
		if (spheres[i].rayintersect(ori, dir, spheres_d) && spheres_d < spheres_d_min)
		{
			spheres_d_min = spheres_d;
			hitpoint = ori + dir * spheres_d;
			Normal = (hitpoint - spheres[i].center).normalize();
			material = spheres[i].material;
		}
	}
	float checkerboard_dist = std::numeric_limits<float>::max();
	if (fabs(dir.y) > 1e-3) {
		float d = -(ori.y + 4) / dir.y; 
		Vec3f pt = ori + dir * d;
		if (d > 0 && fabs(pt.x) < 10 && pt.z<-10 && pt.z>-30 && d < spheres_d_min) {
			checkerboard_dist = d;
			hitpoint = pt;
			Normal = Vec3f(0, 1, 0);
			material.diffuseColor = (int(0.5 * hitpoint.x + 500) + int(0.5 * hitpoint.z)) & 1 ? Vec3f(1, 1, 1) : Vec3f(0.588, 0.294, 0.0);
			material.diffuseColor = material.diffuseColor * 0.3;
		}
	}
	return std::min(spheres_d_min, checkerboard_dist) < 1000;
}
Vec3f cast(const Vec3f& ori, const Vec3f& dir, const std::vector<Sphere>& spheres, const std::vector<Light> &lights, Vec3f* envMap, int envMapWidth, int envMapHeight, size_t reflection_depth = 0)
{
	Vec3f hitpt, N;
	Material mat;

	if (reflection_depth > 4 || !SceneProcess(ori, dir, spheres, hitpt, N, mat))
		return sampleEnvironmentMap(dir, envMap, envMapWidth, envMapHeight);

	Vec3f reflection_dir = reflect(dir, N);
	Vec3f refract_dir = refract(dir, N, mat.refractive_ind).normalize();
	Vec3f reflect_orig;
	if (reflection_dir * N < 0)
		reflect_orig = hitpt - N * 1e-3;
	else
		reflect_orig = hitpt + N * 1e-3; //·ÀÖ¹ self-reflection

	Vec3f refract_orig;
	if (refract_dir * N < 0)
		refract_orig = hitpt - N * 1e-3;
	else
		refract_orig = hitpt + N * 1e-3;

	Vec3f reflect_color, refract_color;

	reflect_color = cast(reflect_orig, reflection_dir, spheres, lights, envMap, envMapWidth, envMapHeight, reflection_depth + 1);
	refract_color = cast(refract_orig, refract_dir, spheres, lights, envMap, envMapWidth, envMapHeight, reflection_depth + 1);

	float diffuse_Light_Intensity = 0.0, specular_Light_Intensity = 0.0;
	for (size_t i = 0; i < lights.size(); i++)
	{
		Vec3f light_d = (lights[i].position - hitpt).normalize();
		float light_distance = (lights[i].position - hitpt).norm();

		Vec3f shadow_orig;
		if (light_d * N < 0)
			shadow_orig = hitpt - N * 1e-3;
		else
			shadow_orig = hitpt + N * 1e-3; //±ÜÃâ Self-shadowing
		Vec3f shadow_p, shadow_N;
		Material tmp;

		if (SceneProcess(shadow_orig, light_d, spheres, shadow_p, shadow_N, tmp) && (shadow_p - shadow_orig).norm() < light_distance)
			continue;

		diffuse_Light_Intensity += lights[i].intensity * std::max(0.f, light_d * N);
		specular_Light_Intensity += powf(std::max(0.f, reflect(light_d, N) * dir), mat.specular_exp) * lights[i].intensity;
	}
	return mat.diffuseColor * diffuse_Light_Intensity * mat.coefficient[0] + Vec3f(1., 1., 1.) * specular_Light_Intensity * mat.coefficient[1] + reflect_color * mat.coefficient[2] + refract_color * mat.coefficient[3];

}
void render(const std::vector<Sphere>& spheres, const std::vector<Light>& lights, Vec3f* envMap, int envMapWidth, int envMapHeight)
{
	std::vector<Vec3f> framebuffer(width * height);

	for (size_t i = 0; i < width; i++)
	{
		for (size_t j = 0; j < height; j++)
		{
			float x = (2 * (i + 0.5) / (float)width - 1) * tan(fov / 2.) * width / (float(height));
			float y = - (2 * (j + 0.5) / (float)height - 1) * tan(fov / 2.);
			Vec3f dir = Vec3f(x, y, -1).normalize();
			framebuffer[i + j * width] = cast(Vec3f(0, 0, 0), dir, spheres, lights, envMap, envMapWidth, envMapHeight, 0);
		}
	}

	std::vector<unsigned char> image(width * height * 3);
	for (size_t i = 0; i < height * width; i++)
	{
		Vec3f& c = framebuffer[i];
		float max = std::max(c[0], std::max(c[1], c[2]));
		if (max > 1)
			c = c * (1.0 / max);
		for (size_t j = 0; j < 3; j++)
		{
			image[i * 3 + j] = (unsigned char)(255 * std::max(0.f, std::min(1.f, framebuffer[i][j])));
		}
	}
	stbi_write_png("./out2.png", width, height, 3, image.data(), width * 3);
}
int main()
{
	std::vector<Material> materials;
	Material ivory(1.0,Vec4f(0.3, 0.3, 0.2, 0.0), Vec3f(0.0, 1.0, 1.0), 50.0);
	Material red_rubber(1.0,Vec4f(0.5, 0.1, 0.2, 0.0), Vec3f(1.0, 0.271, 0.0), 10.0);
	Material Reflect(1.0,Vec4f(0.0, 10.0, 0.8, 0.0), Vec3f(1.0, 1.0, 1.0), 1425.);
	Material glass(1.5, Vec4f(0.0, 0.5, 0.1, 0.8), Vec3f(0.6, 0.7, 0.8), 125.);
	materials.push_back(ivory);
	materials.push_back(red_rubber);

	std::vector<Sphere> spheres;
	spheres.push_back(Sphere(Vec3f(-3, 0, -16), 2, ivory));
	spheres.push_back(Sphere(Vec3f(-1.0, -1.5, -12), 2, glass));
	spheres.push_back(Sphere(Vec3f(1.5, -0.5, -18), 2, red_rubber));
	spheres.push_back(Sphere(Vec3f(7, 5, -18), 4, Reflect));
	spheres.push_back(Sphere(Vec3f(-9, 5, -18), 3, Reflect));

	std::vector<Light>  lights;
	lights.push_back(Light(Vec3f(-20, 20, 20), 1.5));
	lights.push_back(Light(Vec3f(30, 50, -25), 1.8));
	lights.push_back(Light(Vec3f(30, 20, 30), 1.7));

	int envMapWidth, envMapHeight;
	Vec3f* envMap = loadEnvironmentMap("D:\\Coding\\Project\\Myraytracer\\symmetrical_garden_02_4k.jpg", envMapWidth, envMapHeight);
	if (!envMap) {
		return -1; 
	}
	render(spheres, lights, envMap, envMapWidth, envMapHeight);
	delete[] envMap;
	return 0;
}