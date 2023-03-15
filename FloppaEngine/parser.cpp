#include "parser.h"
#include <fstream>
#include <sstream>
#include <iostream>

namespace floppa::render {

	std::vector<std::string> split(const std::string& s, char delim) {
		std::vector<std::string> result;
		std::stringstream ss(s);
		std::string item;

		while (getline(ss, item, delim)) {
			result.push_back(item);
		}

		return result;
	}

	face::face(int v1, int v2, int v3) {
		this->v1 = v1;
		this->v2 = v2;
		this->v3 = v3;
	}

	meshinfo::meshinfo() {
		vertices = std::vector<vector3f>();
		faces = std::vector<face>();
	}

	bool meshinfo::parse(std::string path) {
		std::ifstream objfile(path);
		std::string line;
		if (!objfile.is_open()) { return false; }

		while (std::getline(objfile, line)) {

			if (line.at(0) == '#') { continue; } // a comment
			std::vector<std::string> tokens = split(line, ' ');

			if (tokens[0] == "v") {
				vertices.push_back(vector3f(std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3])));
			}
			else if (tokens[0] == "f") {
				faces.push_back(face(tokens[1].at(0) - '0', tokens[2].at(0) - '0', tokens[3].at(0) - '0'));
			}
		}

		objfile.close();

		return true;
	}

	polygon* meshinfo::getpolygon(face meshface) {
		return new polygon(vertices[meshface.v1 - 1], vertices[meshface.v2 - 1], vertices[meshface.v3 - 1]);
	}
	void meshinfo::loadpolygons(scene* mainscene) {
		for (auto meshface : faces) {
			mainscene->objects.push_back(getpolygon(meshface));
		}
	}

}