#pragma once
#include "SDL.h"
#include <math.h>
#include <string>
#include <vector>
typedef uint32_t u32;
typedef uint8_t u8;

namespace floppa::render {

	u32 getpixel(SDL_Surface* surface, int x, int y);
	void setpixel(SDL_Surface* surface, int x, int y, u32 pixel);
	u32 packpixel(u8 a, u8 r, u8 g, u8 b);

	float lerp(float a, float b, float t);

	enum class ShapeType {
		Sphere,
		Polygon
	};

	struct shape {
		ShapeType Type;
	};

	struct vector3f {
		float x;
		float y;
		float z;
		float length();
		float lengthsq();
		vector3f(float x, float y, float z);
		vector3f(void);
		std::string tostring();
		vector3f normalized();
		float dot(vector3f other);
		static vector3f zero();
		vector3f operator+ (vector3f other);
		vector3f operator- (vector3f other);
		vector3f operator* (float scale);
		vector3f operator* (vector3f other);
		vector3f operator/ (float scale);
		vector3f operator/ (vector3f other);
	};

	struct matrix3x3 {
		float values[3][3];
		matrix3x3(float fill);
		vector3f operator* (vector3f vec);
		std::string tostring();
	};

	struct rotation {
		vector3f euler;
		rotation(float yaw = 0, float pitch = 0, float roll = 0);
		float yaw();
		float pitch();
		float roll();

		matrix3x3 rotmatrix();

		vector3f rotate(vector3f point);
		vector3f rotate(vector3f origin, vector3f point);
	};

	struct rect3f {
		vector3f start;
		vector3f end;
		vector3f center();

		rect3f(vector3f start, vector3f end);
		rect3f();
	};

	struct sphere : shape {
		float radius;
		vector3f center;
		sphere(vector3f center, float radius);
	};

	struct ray {
		vector3f direction;
		vector3f origin;
		ray(vector3f direction, vector3f origin);
		vector3f getpoint(float t);

		bool intersects_at(sphere body, float t);
		bool intersection(sphere body, vector3f* result1, vector3f* result2);
	};
	

	struct camera {
		rect3f screen;
		vector3f position;
		float clipping;
		camera(int fov = 60);
		ray getray(float x_t, float y_t);
		
		vector3f direction();
		bool isrenderable(vector3f point);

		void move(vector3f shift);
		void moveforward(float amount);

		void rotateright(float degree);
		void rotateleft(float degree);
	};

	struct scene {
		camera scene_camera;
		std::vector<shape*> objects;
		scene();
		void render(SDL_Texture* texture, int w, int h);
	};
}