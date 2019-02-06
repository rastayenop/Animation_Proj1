#include "joint.h"
#include <QtGui/QMatrix4x4>

#include <sstream>
#include <iterator>

using namespace std;

int Joint::glIdCounter = 0;


Joint* Joint::createFromFile(std::string fileName) {
  Joint* root = NULL;
  cout << "Loading from " << fileName << endl;
  ifstream inputfile(fileName.data());
  if(inputfile.good()) {
    while(!inputfile.eof()) {
      string buf;
      inputfile >> buf;
      if (buf=="HIERARCHY") {
        // Commandes pour connaitre le tableau de dépendance
        inputfile >> buf;
        Joint::checkToken("ROOT", buf);
        root = Joint::readChild(inputfile, NULL);
        root->nbDofs();
      } else if(buf=="MOTION"){
        if (root == NULL) {
          std::cerr << "root == NULL" << std::endl;
          exit(1);
        }
        inputfile >> buf;
        Joint::checkToken("Frames:", buf);
        inputfile >> buf;
        int nbFrames = std::stoi(buf);
        // La ligne "Frame time: ..." dans le BVH ne nous interesse point
        inputfile >> buf;
        inputfile >> buf;
        inputfile >> buf;
        //itérations sur les mouvements
        for (int i=0; i<nbFrames; i++) {
          Joint::readMotion(inputfile, root);
        }
        cout << nbFrames << " frames loaded" << endl;
      }
    }
    inputfile.close();
  } else {
    std::cerr << "Failed to load the file " << fileName.data() << std::endl;
    fflush(stdout);
  }
  cout << "file loaded" << endl;
  root->printJoint3DPoints();
  cout << root->initalizeGLIds() << endl;
  return root;
}


void Joint::checkToken(std::string expected, std::string buf) {
  if (expected != buf) {
    std::cerr << "Unexpected token " << buf << "; expected " << expected;
    exit(1);
  }
}


Joint* Joint::readChild(std::ifstream &ifs, Joint* parent) {
  string buf;

  ifs >> buf;
  std::string name(buf);
  ifs >> buf;
  checkToken("{", buf);
  ifs >> buf;
  checkToken("OFFSET", buf);
  ifs >> buf;
  double offX = std::stod(buf);
  ifs >> buf;
  double offY = std::stod(buf);
  ifs >> buf;
  double offZ = std::stod(buf);
  Joint* j = create(name, offX, offY, offZ, parent);

  ifs >> buf;
  if (buf == "CHANNELS") {
    ifs >> buf;
    int nbChan = std::stoi(buf);
    int i = 0;
    while (i < nbChan) {
      AnimCurve ac;
      ifs >> buf;
      ac.name = buf;
      j->_dofs.push_back(ac);
      i++;
    }
    ifs >> buf;
  }
  if (buf == "End") {
      Joint::readChild(ifs, j);
      ifs >> buf;
  } else {
    while (buf == "JOINT") {
      Joint::readChild(ifs, j);
      ifs >> buf;
    }
  }
  checkToken("}", buf);
  return j;
}


void Joint::readMotion(std::ifstream &ifs, Joint* node){
  std::string buf;
  //std::cout << "d_size = " << node->_dofs.size() << std::endl;
  int i = 0;
  for(AnimCurve &animCurve : (*node)._dofs){
    ifs >> buf;
    //std::cout << "curve value (" << i << ") = " << buf << std::endl;
    animCurve._values.push_back(std::stod(buf));
    i++;
  }
  for(Joint* child : (*node)._children){
    Joint::readMotion(ifs, child);
  }
}


void Joint::animate(Joint* parent, int iframe){
  // Update dofs :
  _curTx = 0; _curTy = 0; _curTz = 0;
  _curRx = 0; _curRy = 0; _curRz = 0;
  for (unsigned int idof = 0 ; idof < _dofs.size() ; idof++) {
    if(!_dofs[idof].name.compare("Xposition")) _curTx = _dofs[idof]._values[iframe];
    if(!_dofs[idof].name.compare("Yposition")) _curTy = _dofs[idof]._values[iframe];
    if(!_dofs[idof].name.compare("Zposition")) _curTz = _dofs[idof]._values[iframe];
    if(!_dofs[idof].name.compare("Zrotation")) _curRz = _dofs[idof]._values[iframe];
    if(!_dofs[idof].name.compare("Yrotation")) _curRy = _dofs[idof]._values[iframe];
    if(!_dofs[idof].name.compare("Xrotation")) _curRx = _dofs[idof]._values[iframe];
  }
  this->updateMatrix(parent);
  // Animate children :
  for (unsigned int ichild = 0 ; ichild < _children.size() ; ichild++) {
    _children[ichild]->animate(this, iframe);
  }
}


void Joint::updateMatrix(Joint* parent){
  glm::mat4 rotationParent(glm::mat4(1.0));
  glm::mat4 localTransform(glm::transpose(glm::eulerAngleYXZ(glm::radians(_curRy), glm::radians(_curRx), glm::radians(_curRz))));
  if(parent!=NULL){
    rotationParent = parent->_curMat;
    localTransform[3] = glm::vec4(this->_offX, this->_offY, this->_offZ, 1.0);
  }else {
    localTransform[3] = glm::vec4(this->_curTx, this->_curTy, this->_curTz, 1.0);
  }
  this->_curMat = glm::transpose(localTransform) * rotationParent;
}


void Joint::nbDofs() {
  if (_dofs.empty()) return;
  double tol = 1e-4;
  int nbDofsR = -1;
  cout << _name << " anims : ";
  for(AnimCurve &animCurve : _dofs) {
    cout << animCurve.name << " ";
  }
  cout << endl;
  // Propagate to children :
  for (unsigned int ichild = 0 ; ichild < _children.size() ; ichild++) {
    _children[ichild]->nbDofs();
  }
}


void Joint::printJoint3DPoints() {
  ofstream file;
  file.open ("skel_pos3");
  printJoin3DPointsRec(file, 0, 0, 0);
  file.close();
}


void Joint::printJoin3DPointsRec(ofstream &file, float x, float y, float z, Joint* parent) {
  glm::vec4 base = glm::vec4(x, y, z, 1.0);
  int iframe = 3;
  this->animate(parent, iframe);
  glm::vec4 position = base * this->_curMat;
  file << _name << " ";
  for (int i=0; i<4; i++){
    file << position[i] << " ";
  }
  file << "\n";
  for(Joint* child : _children) {
    child->printJoin3DPointsRec(file, x, y, z, this);
  }
}

void Joint::_printJoin3DPointsRec(ofstream &file, float x, float y, float z) {
  x += _offX;
  y += _offY;
  z += _offZ;
  file << _name << " " << x << " " << y << " " << z << "\n";
  for(Joint* child : _children) {
    child->_printJoin3DPointsRec(file, x, y, z);
  }
}

int Joint::initalizeGLIds() {
  Joint::glIdCounter = 0;
  initGLIdsRec();
  return Joint::glIdCounter;
}

void Joint::initGLIdsRec() {
  _glIdentifier = Joint::glIdCounter;
  //std::cout << _name << " id = " << _glIdentifier << endl;
  Joint::glIdCounter++;
  for(Joint* child : _children) {
    child->initGLIdsRec();
  }
}

void Joint::setVertices(trimesh::point *vertices) {
  vertices[_glIdentifier] = trimesh::point(_offX, _offY, _offZ, 1);
  for(Joint* child : _children) {
    child->setVertices(vertices);
  }
}

void Joint::setIndices(int *indices) {
  int index = 0;
  _setIndicesRec(indices, index);
}

void Joint::_setIndicesRec(int *indices, int &index) {
  for(Joint* child : _children) {
    indices[index] = _glIdentifier;
    index++;
    indices[index] = child->_glIdentifier;
    index++;
    child->_setIndicesRec(indices, index);
  }
}
