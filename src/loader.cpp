//
// Created by Forbes Howington on 4/4/20.
//

#include "geometry.h"
#include "loader.h"
#include "collision.h"
#include "scene_node.h"
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

void loadScene(std::vector<std::shared_ptr<ModelInstance>>& modelInstances, std::map<const std::string, Model>& models, std::map<const std::string, TGAImage>& textures, const std::string& sceneFile, std::vector<std::shared_ptr<SceneNode>>& roots, std::map<std::string, std::shared_ptr<SceneNode>>& nodesByName, CollisionWorld& collision) {
  std::string line;
  std::ifstream infile(sceneFile);
  unsigned planeCount = 0;

  // Deferred parent links: child name -> parent name
  std::vector<std::pair<std::string, std::string>> deferredParents;

  // Helper: register a node, optionally with a name and parent
  auto registerNode = [&](std::shared_ptr<SceneNode> node, const std::string& parentName) {
    if (!node->_name.empty())
      nodesByName[node->_name] = node;
    if (!parentName.empty())
      deferredParents.push_back({node->_name, parentName});
    else
      roots.push_back(node);
  };

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

      // Parse optional name:xxx parent:xxx tokens
      std::string nodeName, parentName, token;
      while (iss >> token) {
        if (token.substr(0, 5) == "name:") nodeName = token.substr(5);
        else if (token.substr(0, 7) == "parent:") parentName = token.substr(7);
      }
      if (nodeName.empty()) nodeName = "instance_" + std::to_string(modelInstances.size() - 1);
      auto node = std::make_shared<SceneNode>(nodeName);
      node->_localTransform = modelInstance->_position;
      node->_model = modelInstance;
      registerNode(node, parentName);

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
      // Compute normal for the plane
      float Ax = x0 - x1;
      float Ay = y0 - y1;
      float Az = z0 - z1;
      float Bx = x1 - x2;
      float By = y1 - y2;
      float Bz = z1 - z2;
      vertex<float> norm(Ay * Bz - Az * By, Az * Bx - Ax * Bz, Ax * By - Ay * Bx);
      norm.normalize();

      // Subdivide the quad into N x N sub-quads to reduce per-triangle depth range
      constexpr int N = 4;
      std::vector<vertex<float>> planeVertices;
      std::vector<vertex<float>> planeNorms;
      std::vector<face> planeFaces;

      // Generate (N+1)x(N+1) grid of vertices via bilinear interpolation
      // Corners: p0--p1
      //          |    |
      //          p3--p2
      for (int j = 0; j <= N; ++j) {
        float v = (float)j / N;
        for (int i = 0; i <= N; ++i) {
          float u = (float)i / N;
          float px = (1-u)*(1-v)*x0 + u*(1-v)*x1 + u*v*x2 + (1-u)*v*x3;
          float py = (1-u)*(1-v)*y0 + u*(1-v)*y1 + u*v*y2 + (1-u)*v*y3;
          float pz = (1-u)*(1-v)*z0 + u*(1-v)*z1 + u*v*z2 + (1-u)*v*z3;
          planeVertices.emplace_back(px, py, pz);
          planeNorms.emplace_back(norm._x, norm._y, norm._z);
        }
      }

      for (int j = 0; j < N; ++j) {
        for (int i = 0; i < N; ++i) {
          unsigned a = j * (N+1) + i;
          unsigned b = a + 1;
          unsigned c = a + (N+1) + 1;
          unsigned d = a + (N+1);
          planeFaces.emplace_back(c, b, a,
            planeVertices[c]._x, planeVertices[c]._y,
            planeVertices[b]._x, planeVertices[b]._y,
            planeVertices[a]._x, planeVertices[a]._y);
          planeFaces.emplace_back(d, a, c,
            planeVertices[d]._x, planeVertices[d]._y,
            planeVertices[a]._x, planeVertices[a]._y,
            planeVertices[c]._x, planeVertices[c]._y);
        }
      }


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
      } else if (shader == "stonexy") {
        st = shaderType::StoneXYShader;
      } else if (shader == "stonexz") {
        st = shaderType::StoneXZShader;
      } else if (shader == "stoneyz") {
        st = shaderType::StoneYZShader;
      } else if (shader == "woodxy") {
        st = shaderType::WoodXYShader;
      } else if (shader == "woodxz") {
        st = shaderType::WoodXZShader;
      } else if (shader == "woodyz") {
        st = shaderType::WoodYZShader;
      } else if (shader == "waterxz") {
        st = shaderType::WaterXZShader;
      } else {
        std::cout << "Unknown shader type " << shader << std::endl;
      }

      std::shared_ptr<ModelInstance> planeInstance = std::make_shared<ModelInstance>(models[planeName], st, 0.2, true);

      planeInstance->_position = matrix<4,4>::identity();
      modelInstances.push_back(planeInstance);

      auto node = std::make_shared<SceneNode>(planeName);
      node->_localTransform = planeInstance->_position;
      node->_model = planeInstance;
      registerNode(node, "");

      ++planeCount;

    } else if (line.size() > 4 && !(bool)strncmp(line.c_str(), "node", 4)) {
      // Group node: node <name> <x> <y> <z> [parent:<parentName>]
      line.erase(std::remove(line.begin(), line.end(), '['), line.end());
      line.erase(std::remove(line.begin(), line.end(), ']'), line.end());
      line.erase(std::remove(line.begin(), line.end(), ','), line.end());
      std::istringstream iss(line);
      std::string junk, nodeName;
      float x = 0, y = 0, z = 0;
      if (!(iss >> junk >> nodeName >> x >> y >> z)) {
        std::cout << "Parsing failed for node line " << line << std::endl;
        break;
      }
      auto node = std::make_shared<SceneNode>(nodeName);
      node->_localTransform = matrix<4,4>::identity();
      node->_localTransform.set(3, 0, x);
      node->_localTransform.set(3, 1, y);
      node->_localTransform.set(3, 2, z);
      std::string parentName, token;
      while (iss >> token) {
        if (token.substr(0, 7) == "parent:") parentName = token.substr(7);
      }
      registerNode(node, parentName);

    } else if (line.size() > 8 && !(bool)strncmp(line.c_str(), "collider", 8)) {
      // COLLIDER <x0> <y0> <z0> <x1> <y1> <z1>
      line.erase(std::remove(line.begin(), line.end(), '['), line.end());
      line.erase(std::remove(line.begin(), line.end(), ']'), line.end());
      line.erase(std::remove(line.begin(), line.end(), ','), line.end());
      std::istringstream iss(line);
      std::string junk;
      float x0, y0, z0, x1, y1, z1;
      if (!(iss >> junk >> x0 >> y0 >> z0 >> x1 >> y1 >> z1)) {
        std::cout << "Parsing failed for collider line " << line << std::endl;
        break;
      }
      collision.addBox(x0, y0, z0, x1, y1, z1);

    } else {
        std::cout << "Parsing failed for line " << line << std::endl;
        break;
    }
  }

  // Resolve deferred parent links
  for (auto& [childName, parentName] : deferredParents) {
    auto cit = nodesByName.find(childName);
    auto pit = nodesByName.find(parentName);
    if (cit != nodesByName.end() && pit != nodesByName.end()) {
      pit->second->addChild(cit->second);
      // Remove from roots since it's now a child
      roots.erase(std::remove(roots.begin(), roots.end(), cit->second), roots.end());
    } else {
      std::cout << "Warning: could not resolve parent '" << parentName << "' for node '" << childName << "'" << std::endl;
    }
  }
}
