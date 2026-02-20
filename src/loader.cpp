//
// Created by Forbes Howington on 4/4/20.
//

#include "geometry.h"
#include "loader.h"
#include "shader.h"
#include "tgaimage.h"
#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>

Model::Model(const std::string& fileName, const TGAImage* texture) : _texture(texture) { loadModel(fileName, texture->get_width(), texture->get_height()); }


void  Model::loadModel(const std::string& fileName, const unsigned width, const unsigned height) {
  std::string line;
  std::ifstream infile(fileName);

  if (!infile) {
    std::cout << "File failed: " << fileName << std::endl;
    return;
  }

  std::vector<vertex<float>> textures;

  // Implicit assumption that all faces come after the vertices so this may be done in a single pass
  while (std::getline(infile, line))
  {
    if (line.size() > 1 && line[0] == 'v' && line[1] == ' ') {
      line = line.substr(1, line.size());
      std::istringstream iss(line);
      float x;
      float y;
      float z;
      if (!(iss >> x >> y >> z)) {
        std::cout << "Parsing for vectors failed at line: " << line << std::endl;
        break;
      }

      vertices.emplace_back(x, y, z);

    } else if (line.size() > 2 && line[0] == 'v' && line[1] == 't' && line[2] == ' ') {
      line = line.substr(2, line.size());
      std::istringstream iss(line);
      float x;
      float y;
      if (!(iss >> x >> y)) {
        std::cout << "Parsing for textures failed at line: " << line << std::endl;
        break;
      }

      textures.emplace_back(x, y, 0);

    } else if (line.size() > 1 && line[0] == 'f' && line[1] == ' ') {
      if (line.find('/') != std::string::npos) {

        std::replace(line.begin(), line.end(), '/', ' ');
        line = line.substr(1, line.size());

        std::istringstream iss(line);
        unsigned v0;
        unsigned v1;
        unsigned v2;

        unsigned t0;
        unsigned t1;
        unsigned t2;
        unsigned trash;

        if (!(iss >> v0 >> t0 >> trash >> v1 >> t1 >> trash >> v2 >> t2)) {
          std::cout << "Parsing for faces failed at line: " <<  line << std::endl;
          break;
        }

        faces.emplace_back(v0 - 1, v1 - 1, v2-1,
                           textures.at(t0-1)._x * (float)width, textures.at(t0-1)._y * (float)height,
                           textures.at(t1-1)._x * (float)width, textures.at(t1-1)._y * (float)height,
                           textures.at(t2-1)._x * (float)width, textures.at(t2-1)._y * (float)height);
      } else {
        line = line.substr(1, line.size());
        std::istringstream iss(line);

        unsigned v0;
        unsigned v1;
        unsigned v2;

        if (!(iss >> v0 >> v1 >> v2)) {
          std::cout << "Parsing for faces failed at line: " <<  line << std::endl;
          break;
        }

        faces.emplace_back(v0 - 1, v1 - 1, v2-1);
      }
    } else if (line.size() > 2 && line[0] == 'v' && line[1] == 'n' && line[2] == ' ') {
      line = line.substr(2, line.size());
      std::istringstream iss(line);
      float x;
      float y;
      float z;
      if (!(iss >> x >> y >> z)) {
          std::cout << "Parsing for vertex normals failed at line: " <<  line << std::endl;
        break;
      }

      vertexNormals.emplace_back(x, y, z);

    }
  }
}

void loadScene(std::vector<std::shared_ptr<ModelInstance>>& modelInstances, std::map<const std::string, Model>& models, std::map<const std::string, TGAImage>& textures, const std::string& sceneFile) {
  std::string line;
  std::ifstream infile(sceneFile);
  unsigned planeCount = 0;

  while (std::getline(infile, line))
  {
    // Skip comments and empty lines
    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::for_each(line.begin(), line.end(), [](char & c){
                                                c = ::tolower(c);
                                              });

    if (line.size() > 8 && !(bool)strncmp(line.c_str(), "instance", 8)) {
      line.erase(std::remove(line.begin(), line.end(), '['), line.end());
      line.erase(std::remove(line.begin(), line.end(), ']'), line.end());
      line.erase(std::remove(line.begin(), line.end(), ','), line.end());
      std::istringstream iss(line);

      std::string junk;
      std::string modelName;
      std::string shader;
      float x;
      float y;
      float z;
      float xScale;
      float yScale;
      float zScale;
      float xRot;
      float yRot;
      float zRot;
      if (!(iss >> junk >> modelName >> shader >> x >> y >> z >> xScale >> yScale >> zScale >> xRot >> yRot >> zRot)) {
        std::cout << "Parsing failed for line " << line << std::endl;
        break;
      }

      shaderType st;


      if (shader == "gourand") {
        st = shaderType::GouraudShader;
      } else if (shader == "flat") {
        st = shaderType::FlatShader;
      } else if (shader == "interpolateflat") {
        st = shaderType::InterpFlatShader;
      } else if (shader == "interpolategourand") {
        st = shaderType::InterpGouraudShader;
      } else {
        std::cout << "Unknown shader type " << shader << " for instance " << line << std::endl;
        break;
      }

      if (models.find(modelName) == models.end()) {
        std::cout << "Unable to find model " << modelName << " for instance " << line << std::endl;
        break;
      }
      std::shared_ptr<ModelInstance> modelInstance = std::make_shared<ModelInstance>(models[modelName], st);
      modelInstance->_position.set(0, 0, xScale);
      modelInstance->_position.set(1, 1, yScale);
      modelInstance->_position.set(2, 2, zScale);
      modelInstance->_position.set(3, 3, 1);

      // Now rotate
      modelInstance->_position = modelInstance->_position * matrix<4,4>::rotationX(xRot);
      modelInstance->_position = modelInstance->_position * matrix<4,4>::rotationY(yRot);
      modelInstance->_position = modelInstance->_position * matrix<4,4>::rotationZ(zRot);

      // Finally apply coordinates
      modelInstance->_position.set(3, 0, x);
      modelInstance->_position.set(3, 1, y);
      modelInstance->_position.set(3, 2, z);
      modelInstances.push_back(modelInstance);

    } else if (line.size() > 5 && !(bool)strncmp(line.c_str(), "model", 5)) {
      line.erase(std::remove(line.begin(), line.end(), '['), line.end());
      line.erase(std::remove(line.begin(), line.end(), ']'), line.end());
      line.erase(std::remove(line.begin(), line.end(), ','), line.end());
      std::istringstream iss(line);

      std::string junk;
      std::string modelName;
      std::string modelFile;
      std::string textureName;
      if (!(iss >> junk >> modelName >> modelFile >> textureName)) {
        std::cout << "Parsing failed for line " << line << std::endl;
        break;
      }

      if (textures.find(textureName) == textures.end()) {
        std::cout << "Unable to find texture " << modelName << " for model " << line << std::endl;
        break;
      }
      models.emplace(std::piecewise_construct,
                     std::forward_as_tuple(modelName),
                     std::forward_as_tuple(modelFile, &textures[textureName]));

    } else if (line.size() > 7 && !(bool)strncmp(line.c_str(), "texture", 7)) {
      line.erase(std::remove(line.begin(), line.end(), '['), line.end());
      line.erase(std::remove(line.begin(), line.end(), ']'), line.end());
      line.erase(std::remove(line.begin(), line.end(), ','), line.end());
      std::istringstream iss(line);

      std::string junk;
      std::string textureName;
      std::string textureFile;
      if (!(iss >> junk >> textureName >> textureFile)) {
        std::cout << "Parsing failed for line " << line << std::endl;
        break;
      }

      textures[textureName] = TGAImage();
      textures[textureName].read_tga_file(textureFile.c_str());
      textures[textureName].flip_vertically();

    }  else if (line.size() > 5 && !(bool)strncmp(line.c_str(), "plane", 5)) {
      line.erase(std::remove(line.begin(), line.end(), '['), line.end());
      line.erase(std::remove(line.begin(), line.end(), ']'), line.end());
      line.erase(std::remove(line.begin(), line.end(), ','), line.end());
      std::istringstream iss(line);

      std::string junk;
      std::string shader;
      float x0;
      float y0;
      float z0;

      float x1;
      float y1;
      float z1;

      float x2;
      float y2;
      float z2;

      float x3;
      float y3;
      float z3;

      if (!(iss >> junk >> shader >> x0 >> y0 >> z0 >> x1 >> y1 >> z1 >> x2 >> y2 >> z2 >> x3 >> y3 >> z3)) {
        std::cout << "Parsing failed for line " << line << std::endl;
        break;
      }
      std::vector<vertex<float>> planeVertices;

      planeVertices.emplace_back(x1, y1, z1);
      planeVertices.emplace_back(x0, y0, z0);
      planeVertices.emplace_back(x2, y2, z2);

      planeVertices.emplace_back(x2, y2, z2);
      planeVertices.emplace_back(x0, y0, z0);
      planeVertices.emplace_back(x3, y3, z3);

      planeVertices.emplace_back(x1, y1, z1);
      planeVertices.emplace_back(x0, y0, z0);
      planeVertices.emplace_back(x2, y2, z2);

      planeVertices.emplace_back(x2, y2, z2);
      planeVertices.emplace_back(x0, y0, z0);
      planeVertices.emplace_back(x3, y3, z3);

      std::vector<vertex<float>> planeNorms;

      float Ax = x0 - x1;
      float Ay = y0 - y1;
      float Az = z0 - z1;

      float Bx = x1 - x2;
      float By = y1 - y2;
      float Bz = z1 - z2;

      float Nx = Ay * Bz - Az * By;
      float Ny = Az * Bx - Ax * Bz;
      float Nz = Ax * By - Ay * Bx;

      float Ax2 = x0 - x2;
      float Ay2 = y0 - y2;
      float Az2 = z0 - z2;

      float Bx2 = x2 - x3;
      float By2 = y2 - y3;
      float Bz2 = z2 - z3;

      float Nx2 = Ay2 * Bz2 - Az2 * By2;
      float Ny2 = Az2 * Bx2 - Ax2 * Bz2;
      float Nz2 = Ax2 * By2 - Ay2 * Bx2;

      vertex<float> norm(Nx, Ny, Nz);
      norm.normalize();

      vertex<float> norm2(Nx2, Ny2, Nz2);
      norm2.normalize();

      planeNorms.emplace_back(norm._x, norm._y, norm._z);
      planeNorms.emplace_back(norm._x, norm._y, norm._z);
      planeNorms.emplace_back(norm._x, norm._y, norm._z);

      planeNorms.emplace_back(norm2._x, norm2._y, norm2._z);
      planeNorms.emplace_back(norm2._x, norm2._y, norm2._z);
      planeNorms.emplace_back(norm2._x, norm2._y, norm2._z);

      planeNorms.emplace_back(-norm._x, -norm._y, -norm._z);
      planeNorms.emplace_back(-norm._x, -norm._y, -norm._z);
      planeNorms.emplace_back(-norm._x, -norm._y, -norm._z);

      planeNorms.emplace_back(-norm2._x, -norm2._y, -norm2._z);
      planeNorms.emplace_back(-norm2._x, -norm2._y, -norm2._z);
      planeNorms.emplace_back(-norm2._x, -norm2._y, -norm2._z);

      std::vector<face> planeFaces;
      planeFaces.emplace_back(2, 1, 0, x2, y2, x0, y0, x1, y1);
      planeFaces.emplace_back(5, 4, 3, x3, y3, x0, y0, x2, y2);

      planeFaces.emplace_back(6, 7, 8, x1, y1, x0, y0, x2, y2);
      planeFaces.emplace_back(9, 10, 11, x2, y2, x0, y0, x3, y3);


      std::string planeName("plane");
      planeName.append(std::to_string(planeCount));

      models.emplace(std::piecewise_construct,
                     std::forward_as_tuple(planeName),
                     std::forward_as_tuple());

      models[planeName].setVertices(std::move(planeVertices));
      models[planeName].setNormals(std::move(planeNorms));
      models[planeName].setFaces(std::move(planeFaces));

      shaderType st;

      if (shader == "planexy") {
        st = shaderType::PlaneXYShader;
      } else if (shader == "planexz") {
        st = shaderType::PlaneXZShader;
      } else if (shader == "planeyz") {
        st = shaderType::PlaneYZShader;
      } else {
        std::cout << "Unknown shader type " << shader << std::endl;
      }

      std::shared_ptr<ModelInstance> planeInstance = std::make_shared<ModelInstance>(models[planeName], st);

      planeInstance->_position = matrix<4,4>::identity();
      modelInstances.push_back(planeInstance);

      ++planeCount;

    } else {
        std::cout << "Parsing failed for line " << line << std::endl;
        break;
    }
  }
}
