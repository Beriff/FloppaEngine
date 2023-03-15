#include "SDL.h"
#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <thread>

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
		abs = 1 / abs;
		return vector3f(x * abs, y * abs, z * abs);
	}
	vector3f vector3f::zero() { return vector3f(0,0,0); }

	float vector3f::dot(vector3f other) {
		return x * other.x + y * other.y + z * other.z;
	}

	vector3f vector3f::cross(vector3f other) {
		return vector3f(z*other.y - y*other.z, x*other.z - z*other.x, y*other.x - x*other.y);
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

	float vector3f::max() {
		return std::max(std::max(x,y),z);
	}
	float vector3f::min() {
		return std::min(std::min(x, y), z);
	}
	vector3f vector3f::abs() {
		return vector3f(std::abs(x), std::abs(y), std::abs(z));
	}

	vector3f rect3f::center() {
		return vector3f((end.x - start.x) / 2, (end.y - start.y) / 2, (end.z - start.z) / 2) + start;
	}

	vector2f::vector2f(float x, float y) { this->x = x; this->y = y; }
	vector2f::vector2f(vector3f v3) {
		if (v3.x == 0) {
			x = v3.y;
			y = v3.z;
		}
		else if (v3.y == 0) {
			x = v3.x;
			y = v3.z;
		}
		else {
			x = v3.x;
			y = v3.y;
		}
	}
	vector2f::vector2f() { x = 0; y = 0; }

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

	bool plane::inplane(vector3f point) {
		return (point - origin).dot(normal) == 0;
	}

	plane::plane(vector3f normal, vector3f origin) {
		this->normal = normal;
		this->origin = origin;
		norm_origin_dot = origin.dot(normal);
	}
	plane::plane() {
		normal = vector3f::zero();
		origin = vector3f::zero();
		norm_origin_dot = origin.dot(normal);
	}
	polygon::polygon(vector3f p1, vector3f p2, vector3f p3) {
		this->p1 = p1;
		this->p2 = p2;
		this->p3 = p3;
		Type = ShapeType::Polygon;

		vector3f mid = vector3f(lerp(p1.x, p2.x, .5f), lerp(p1.y, p2.y, .5f), lerp(p1.z, p2.z, .5f));
		center = vector3f(lerp(mid.x, p3.x, .5f), lerp(mid.y, p3.y, .5f), lerp(mid.z, p3.z, .5f));
		polygon_plane = plane((p2 - p1).cross(p3 - p1), center);

		//HASH THE SHIT OUTTA IT

		vector3f absnormal = polygon_plane.normal.abs();
		float max = absnormal.max();

		if (absnormal.x == max) { flattening_mask = vector3f(0, 1, 1); }
		else if (absnormal.y == max) { flattening_mask = vector3f(1, 0, 1); }
		else { flattening_mask = vector3f(1, 1, 0); }

		flat_p1 = vector2f(p1 * flattening_mask);
		flat_p2 = vector2f(p2 * flattening_mask);
		flat_p3 = vector2f(p3 * flattening_mask);
	}
	polygon::polygon() {
		p1 = vector3f::zero();
		p2 = vector3f::zero();
		p3 = vector3f::zero();
		Type = ShapeType::Polygon;
		center = vector3f::zero();
		polygon_plane = plane();
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
	bool ray::intersection(sphere* body, vector3f* result1, vector3f* result2) {
		//get b and c coefficients of quadratic equation (a = 1 because the ray direction is normalized)
		//At^2+Bt+C computes sphere intersection points (if discriminant is less than 0, then the ray misses the sphere)

		/*
		* the equation is obtained by substituting ray equation (R(t) = origin + direction * t)
		* into the implicit sphere equation (X^2 + Y^2 + Z^2 = r^2)
		*/
		float B = 2 * (
			direction.x * (origin.x - body->center.x) +
			direction.y * (origin.y - body->center.y) +
			direction.z * (origin.z - body->center.z));
		float C = (origin.x - body->center.x) * (origin.x - body->center.x) +
			(origin.y - body->center.y) * (origin.y - body->center.y) +
			(origin.z - body->center.z) * (origin.z - body->center.z)
			- body->radius * body->radius;

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
	bool ray::intersection(plane* planebody, vector3f* result) {
		float coeff1 = planebody->normal.dot(this->direction);
		// if dot product of plane normal and ray's direction is 0, then the ray is parallel to the plane
		// therefore, no intersection occurs
		if (coeff1 == 0) { return false; }
		
		float coeff2 = -(planebody->normal.dot(this->origin) + planebody->norm_origin_dot);

		*result = getpoint(coeff2 / coeff1);
		return true;
	}
	float sign(vector2f p1, vector2f p2, vector2f p3) { return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y); }
	bool ray::intersection(polygon* poly, vector3f* result) {
		vector3f intsc_point;
		if (!intersection(&poly->polygon_plane, &intsc_point)) { return false; }

		/*
		Project the polygon onto a 2d plane. Project by throwing away one of the coordinates.
		To avoid convering the polygon into a single line (in a worst-case scenario), throw
		away the coordinate that is the biggest one in plane's normal vector.
		*/

		//algorithm to check if a 2d point inside a triangle \/\/\/

		vector2f intsc_2d = vector2f(intsc_point * poly->flattening_mask);

		float d1, d2, d3;
		bool hasneg, haspos;
		d1 = sign(intsc_2d, poly->flat_p1, poly->flat_p2);
		d2 = sign(intsc_2d, poly->flat_p2, poly->flat_p3);
		d3 = sign(intsc_2d, poly->flat_p3, poly->flat_p1);
		hasneg = (d1 < 0) || (d2 < 0) || (d3 < 0);
		haspos = (d1 > 0) || (d2 > 0) || (d3 > 0);

		if (!(hasneg && haspos)) {
			*result = intsc_point;
			return true;
		} return false;
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

	bool camera::isrenderable(vector3f& point) {
		vector3f rvec = point - position;

		//if point's distance too small, then its between the camera and the clipping plane
		// => dont render it
		//if dot product of the point's vector and camera's direction is
		//smaller than 0, then its behind the camera (with respect to its facing direction)
		return rvec.dot(direction()) > 1 && rvec.lengthsq() > clipping;
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
		planes = std::vector<plane*>();
		objects.push_back(new sphere(vector3f(100, 300, 350), 50));
		objects.push_back(new polygon(vector3f(90, 200, 450), vector3f(150, 220, 450), vector3f(60, 240, 450)));
		//planes.push_back(new plane(vector3f(0, 1, 0), vector3f(0,-50,0)));

		objects.push_back(new sphere(vector3f(100, 300, 600), 50));
		/*std::cout << scene_camera.position.tostring() << "\n";
		std::cout << scene_camera.screen.start.tostring() << "\n";
		std::cout << scene_camera.screen.end.tostring() << "\n";*/
	}

	void setpixel_interp(u32* pixelarr, int x, int y, int pitch, u32 pixel) {
		int flattened = flatten(x, y, pitch);
		pixelarr[flattened] = pixel;
		pixelarr[flattened+1] = pixel;
		pixelarr[flattened+pitch] = pixel;
		pixelarr[flattened+1+pitch] = pixel;
	}

	vector3f getshapecenter(shape* s) {
		switch (s->Type) {
		case ShapeType::Sphere:
			return ((sphere*)s)->center;
		case ShapeType::Polygon:
			return ((polygon*)s)->center;
		default:
			throw 0;
		}
	}

	void renderregion(u32* pixels, int startx, int starty, int w, int h, float invw, float invh, std::vector<shape*>* sorted_bodies, camera* scene_camera,
		int absw, int absh) {
		vector3f t1, t2;
		//float added_x = startx * invw;
		//float added_y = starty * invh;


		for (int x = startx; x < w; x += 2) {
			for (int y = starty; y < h; y += 2) {
				ray camera_ray = scene_camera->getray(x/(float)absw, y/(float)absh);
				setpixel_interp(pixels, x, y, absw, packpixel(0, 0, 0, 0));
				for (auto body : *sorted_bodies) {
					vector3f center = getshapecenter(body);
					if (body->Type == ShapeType::Sphere) {
						if (camera_ray.intersection((sphere*)body, &t1, &t2) && (scene_camera->isrenderable(t1) || scene_camera->isrenderable(t2))) {
							sphere* s = (sphere*)body;
							setpixel_interp(pixels, x, y, absw,
								packpixel(
									norm(s->center.x + s->radius, s->center.x - s->radius, t1.x) * 255,
									norm(s->center.y + s->radius, s->center.y - s->radius, t1.y) * 255,
									norm(s->center.z + s->radius, s->center.z - s->radius, t1.z) * 255, 0));
							break;
						}
					}
					else if (body->Type == ShapeType::Polygon) {
						if (camera_ray.intersection((polygon*)body, &t1) && (scene_camera->isrenderable(t1))) {
							setpixel_interp(pixels, x, y, absw, packpixel(137, 137, 137, 0));
							break;
						}
					}
				}
			}
		}
	}

	void scene::render(SDL_Texture* texture, int w, int h, float invw, float invh) {
		u32 grey = packpixel(137, 137, 137, 0);
		vector3f t1 = vector3f::zero();
		vector3f t2 = vector3f::zero();

		void* pixels;
		int pitch;

		std::vector<shape*> bodies_sorted = objects;
		std::sort(bodies_sorted.begin(), bodies_sorted.end(),
			[this](shape*& lhs, shape*& rhs) { 
				return (getshapecenter(lhs) - scene_camera.position).lengthsq() < (getshapecenter(rhs) - scene_camera.position).lengthsq();
			});

		SDL_LockTexture(texture, NULL, &pixels, &pitch);
		
		std::thread q1 = std::thread(&renderregion, (u32*)pixels, 0, 0, w / 2, h / 2, invw, invh, &bodies_sorted, &scene_camera,w,h);
		std::thread q2 = std::thread(&renderregion, (u32*)pixels, w / 2, 0, w, h / 2, invw, invh, &bodies_sorted, &scene_camera, w, h);
		std::thread q3 = std::thread(&renderregion, (u32*)pixels, 0, h / 2, w, h, invw, invh, &bodies_sorted, &scene_camera, w, h);
		std::thread q4 = std::thread(&renderregion, (u32*)pixels, w / 2, h / 2, w, h, invw, invh, &bodies_sorted, &scene_camera, w, h);

		q1.join();
		q2.join();
		q3.join();
		q4.join();

		//std::cout << invh;
		//renderregion((u32*)pixels, 0, 0, w, h, invw, invh, &bodies_sorted, &scene_camera);

		SDL_UnlockTexture(texture);
	}

}
