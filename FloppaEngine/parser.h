#pragma once
#include <vector>
#include <string>

#include "scene.h"
namespace floppa::render {

	struct face {
		int v1;
		int v2;
		int v3;
		face(int v1, int v2, int v3);
	};

	struct meshinfo {
		std::vector<vector3f> vertices;
		std::vector<face> faces;
		meshinfo();
		bool parse(std::string path);

		polygon* getpolygon(face meshface);
		void loadpolygons(scene* mainscene);
	};



}