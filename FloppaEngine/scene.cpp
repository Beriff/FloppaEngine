#include "SDL.h"
#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <algorithm>

#include "scene.h"

typedef uint32_t u32;
typedef uint8_t u8;

namespace floppa::render {

	int inline flatten(int x, int y, int w) {
		return y * w + x;
	}
	float lerp(float a, float b, float t) { return a * (1 - t) + b * t; };
	float norm(float mx, float mn, float v) { return (v - mn) / (mx - mn); }

	float vector3f::length() {
		return sqrtf(x * x + y * y + z * z);
	}

	float vector3f::lengthsq() {
		return x * x + y * y + z * z;
	}
	vector3f::vector3f() {
		x = 0;
		y = 0;
		z = 0;
	}
	vector3f::vector3f(float x, float y, float z) {
		this->x = x;
		this->y = y;
		this->z = z;
	}
	std::string vector3f::tostring() {
		return "(" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ")";
	}

	vector3f vector3f::normalized() {
		float abs = length();
		return vector3f(x / abs, y / abs, z / abs);
	}
	vector3f vector3f::zero() { return vector3f(0,0,0); }

	float vector3f::dot(vector3f other) {
		return x * other.x + y * other.y + z * other.z;
	}

	vector3f vector3f::operator+ (vector3f other) {
		return vector3f(x + other.x, y + other.y, z + other.z);
	}
	vector3f vector3f::operator- (vector3f other) {
		return vector3f(x - other.x, y - other.y, z - other.z);
	}
	vector3f vector3f::operator* (float scale) {
		return vector3f(x * scale, y * scale, z * scale);
	}
	vector3f vector3f::operator* (vector3f other) {
		return vector3f(x * other.x, y * other.y, z * other.z);
	}
	vector3f vector3f::operator/ (float scale) {
		return vector3f(x / scale, y / scale, z / scale);
	}
	vector3f vector3f::operator/ (vector3f other) {
		return vector3f(x / other.x, y / other.y, z / other.z);
	}

	vector3f rect3f::center() {
		return vector3f((end.x - start.x) / 2, (end.y - start.y) / 2, (end.z - start.z) / 2) + start;
	}

	rect3f::rect3f(vector3f start, vector3f end) {
		this->start = start;
		this->end = end;
	}
	rect3f::rect3f() {
		start = vector3f();
		end = vector3f(1, 1, 1);
	}

	matrix3x3::matrix3x3(float fill = 0) {
		for (int x = 0; x < 3; x++) {
			for (int y = 0; y < 3; y++) { values[x][y] = fill; }
		}
	}
	vector3f matrix3x3::operator* (vector3f vec) {
		return vector3f(
			(values[0][0] * vec.x) + (values[1][0] * vec.y) + (values[2][0] * vec.z),
			(values[0][1] * vec.x) + (values[1][1] * vec.y) + (values[2][1] * vec.z),
			(values[0][2] * vec.x) + (values[1][2] * vec.y) + (values[2][2] * vec.z)
		);
	}

	rotation::rotation(float yaw, float pitch, float roll) {
		this->euler = vector3f(yaw, pitch, roll);
	}
	float rotation::yaw() { return euler.x; }
	float rotation::pitch() { return euler.z; }
	float rotation::roll() { return euler.y; }

	matrix3x3 rotation::rotmatrix() {
		matrix3x3 matrix = matrix3x3();
		matrix.values[0][0] = cos(yaw()) * cos(pitch());
		matrix.values[1][0] = cos(yaw()) * sin(pitch()) * sin(roll()) - sin(yaw()) * cos(roll());
		matrix.values[2][0] = cos(yaw()) * sin(pitch()) * cos(roll()) + sin(yaw()) * sin(roll());

		matrix.values[0][1] = sin(yaw()) * cos(pitch());
		matrix.values[1][1] = sin(yaw()) * sin(pitch()) * sin(roll()) + cos(yaw()) * cos(roll());
		matrix.values[2][1] = sin(yaw()) * sin(pitch()) * cos(roll()) - cos(yaw()) * sin(roll());

		matrix.values[0][2] = -sin(pitch());
		matrix.values[1][2] = cos(pitch()) * sin(roll());
		matrix.values[2][2] = cos(pitch()) * cos(roll());

		return matrix;
	}

	std::string matrix3x3::tostring() {
		return "[" + std::to_string(values[0][0]) + " "
			+ std::to_string(values[1][0]) + " "
			+ std::to_string(values[2][0]) + "]\n["
			+ std::to_string(values[0][1]) + " "
			+ std::to_string(values[1][1]) + " "
			+ std::to_string(values[2][1]) + "]\n["
			+ std::to_string(values[0][2]) + " "
			+ std::to_string(values[1][2]) + " "
			+ std::to_string(values[2][2]) + "]";
			
	}

	vector3f rotation::rotate(vector3f point) {
		return rotmatrix() * point;
	}
	vector3f rotation::rotate(vector3f origin, vector3f point) {
		vector3f difference = point - origin;
		difference = rotmatrix() * difference;
		return difference + origin;
	}

	u32 getpixel(SDL_Surface* surface, int x, int y) {
		u32* pixels = (u32*)surface->pixels;
		return pixels[flatten(x, y, surface->w)];
	}
	void setpixel(SDL_Surface* surface, int x, int y, u32 pixel) {
		u32* pixels = (u32*)surface->pixels;
		pixels[flatten(x, y, surface->w)] = pixel;
	}
	u32 packpixel(u8 r, u8 g, u8 b, u8 a) {
		return (u32)(b | (g << 8) | (r << 16) | (a << 24));
	}

	//------- SCENE RAYCASTING

	//------ GEOMETRIC BODIES

	sphere::sphere(vector3f center, float radius) {
		this->center = center;
		this->radius = radius;
		Type = ShapeType::Sphere;
	}

	//------ GEOMETRIC RAYS

	ray::ray(vector3f direction, vector3f origin) {
		this->direction = direction;
		this->origin = origin;
	}
	
	vector3f ray::getpoint(float t) {
		return origin + (direction * t);
	}

	bool ray::intersects_at(sphere body, float t) {
		vector3f point = getpoint(t);
		vector3f diff = point - body.center;
		return (diff * diff).lengthsq() == (body.radius * body.radius);
	}
	bool ray::intersection(sphere body, vector3f* result1, vector3f* result2) {
		//get b and c coefficients of quadratic equation (a = 1 because the ray direction is normalized)
		//At^2+Bt+C computes sphere intersection points (if discriminant is less than 0, then the ray misses the sphere)

		/*
		* the equation is obtained by substituting ray equation (R(t) = origin + direction * t)
		* into the implicit sphere equation (X^2 + Y^2 + Z^2 = r^2)
		*/
		float B = 2 * (
			direction.x * (origin.x - body.center.x) +
			direction.y * (origin.y - body.center.y) +
			direction.z * (origin.z - body.center.z));
		float C = (origin.x - body.center.x) * (origin.x - body.center.x) +
			(origin.y - body.center.y) * (origin.y - body.center.y) +
			(origin.z - body.center.z) * (origin.z - body.center.z)
			- body.radius * body.radius;

		//compute the discriminant
		float d = B * B - 4 * C;
		if (d < 0) { return false; }
		d = sqrt(d);

		//compute the intersection points
		vector3f t1 = getpoint((-B - d) / 2);
		vector3f t2 = getpoint((-B + d) / 2);

		*result1 = t1;
		*result2 = t2;
		return true;
	}

	camera::camera(int fov) {
		screen = rect3f(vector3f(0, 600, 0), vector3f(0, 0, 800));
		position = screen.center() - vector3f(fov, 0, 0);
		clipping = 1000;
	}
	ray camera::getray(float x_t, float y_t) {
		vector3f cplane_point = vector3f(
			lerp(screen.start.x, screen.end.x, x_t),
			lerp(screen.start.y, screen.end.y, y_t),
			lerp(screen.start.z, screen.end.z, x_t)
		);

		vector3f dir = (cplane_point - position).normalized();
		return ray(dir, position);
	}

	vector3f camera::direction() {
		return (screen.center() - position).normalized();
	}

	bool camera::isrenderable(vector3f point) {
		vector3f rvec = point - position;

		//if point's distance too small, then its between the camera and the clipping plane
		// => dont render it
		//if dot product of the point's vector and camera's direction is
		//smaller than 0, then its behind the camera (with respect to its facing direction)
		return rvec.lengthsq() > clipping && rvec.dot(direction()) > 0;
	}

	void camera::move(vector3f shift) {
		screen = rect3f(screen.start + shift, screen.end + shift);
		position = position + shift;
	}

	void camera::moveforward(float amount) {
		vector3f forward = (screen.center() - position).normalized();
		move(forward * amount);
	}

	void camera::rotateright(float degree) {
		rotation r = rotation(0, 0, -degree * M_PI / 180);
		screen.start = r.rotate(position, screen.start);
		screen.end = r.rotate(position, screen.end);
	}
	void camera::rotateleft(float degree) {
		rotateright(-degree);
	}

	scene::scene() {
		scene_camera = camera(240);
		objects = std::vector<shape*>();
		objects.push_back(new sphere(vector3f(100, 300, 350), 50));

		objects.push_back(new sphere(vector3f(100, 300, 600), 50));
		/*std::cout << scene_camera.position.tostring() << "\n";
		std::cout << scene_camera.screen.start.tostring() << "\n";
		std::cout << scene_camera.screen.end.tostring() << "\n";*/
	}

	void setpixel_interp(u32* pixelarr, int x, int y, int pitch, u32 pixel) {
		pixelarr[flatten(x, y, pitch)] = pixel;
		pixelarr[flatten(x+1, y, pitch)] = pixel;
		pixelarr[flatten(x, y+1, pitch)] = pixel;
		pixelarr[flatten(x+1, y+1, pitch)] = pixel;
	}

	vector3f getshapecenter(shape* s) {
		switch (s->Type) {
		case ShapeType::Sphere:
			return ((sphere*)s)->center;
		default:
			throw 0;
		}
	}

	void scene::render(SDL_Texture* texture, int w, int h) {
		u32 grey = packpixel(137, 137, 137, 0);
		vector3f t1 = vector3f::zero();
		vector3f t2 = vector3f::zero();

		void* pixels;
		int pitch;

		std::vector<shape*> bodies_sorted = objects;
		std::sort(bodies_sorted.begin(), bodies_sorted.end(),
			[this](shape*& lhs, shape*& rhs) { 
				return (getshapecenter(lhs) - scene_camera.position).lengthsq() > (getshapecenter(rhs) - scene_camera.position).lengthsq();
			});

		SDL_LockTexture(texture, NULL, &pixels, &pitch);

		
		for (int x = 0; x < w; x += 2) {
			for (int y = 0; y < h; y += 2) {
				ray camera_ray = scene_camera.getray(x/(float)w, y/(float)h);
				setpixel_interp((u32*)pixels, x, y, pitch / 4, packpixel(0, 0, 0, 0));
				for (auto body : bodies_sorted) {
					if (body->Type == ShapeType::Sphere) {
						if (camera_ray.intersection(*((sphere*)body), &t1, &t2) && (scene_camera.isrenderable(t1) || scene_camera.isrenderable(t2) ) ) {
							sphere* s = (sphere*)body;
							int coeff = t1.lengthsq() * 0.00005f;
							coeff = coeff * coeff;
							setpixel_interp((u32*)pixels, x, y, pitch / 4, 
								packpixel(
									norm(s->center.x + s->radius, s->center.x - s->radius, t1.x) * 255, 
									norm(s->center.y + s->radius, s->center.y - s->radius, t1.y) * 255, 
									norm(s->center.z + s->radius, s->center.z - s->radius, t1.z) * 255, 0));
						}
					}
				}
			}
		}

		SDL_UnlockTexture(texture);
	}

}
