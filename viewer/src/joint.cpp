#include "joint.h"
#include <QtGui/QMatrix4x4>

#include <sstream>
#include <iterator>

using namespace std;

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
        cout << "TEST TEST TEST" << endl;
        root->printJoint3DPoints();
        root->nbDofs();
      } else if(buf=="MOTION"){
        if (root == NULL) {
          std::cerr << "root == NULL" << std::endl;
          exit(1);
        }
        // Commandes pour avoir les mouvements #PAUL(enfin j'espère)
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
  updateMatrix(parent);
  // Animate children :
  for (unsigned int ichild = 0 ; ichild < _children.size() ; ichild++) {
    _children[ichild]->animate(this, iframe);
  }
}


void Joint::updateMatrix(Joint* parent){
  glm::mat4 previous(this->_curMat);
  glm::mat4 rotationParent;
  glm::mat4 localTransform;
  rotationParent = glm::mat4(1.0);
  if(parent!=NULL){
    rotationParent = parent->_curMat;
  }
  localTransform = glm::translate(glm::mat4(), glm::vec3(_curTx, _curTy, _curTz));/* * glm::rotate(_curRx, 1., 0., 0.);/* *
      glm::rotate(_curRy, 0, 1, 0) *
      glm::rotate(_curRz, 0, 0, 1) * */
  _curMat = rotationParent * localTransform * previous;
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

void Joint::computeState() {
  //TODO
  // calculer les matrice de rotation
}

void Joint::printJoint3DPoints() {
  ofstream file;
  file.open ("skel_pos");
  printJoin3DPointsRec(file, 0, 0, 0);
  file.close();
}

void Joint::printJoin3DPointsRec(ofstream &file, float x, float y, float z) {
  x += _offX;
  y += _offY;
  z += _offZ;
  file << _name << " " << x << " " << y << " " << z << "\n";
  for(Joint* child : _children) {
    child->printJoin3DPointsRec(file, x, y, z);
  }
}
