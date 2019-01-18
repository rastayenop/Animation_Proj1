#include "joint.h"
#include <QtGui/QMatrix4x4>

using namespace std;

Joint* Joint::createFromFile(std::string fileName) {
	Joint* root = NULL;
	cout << "Loading from " << fileName << endl;

	ifstream inputfile(fileName.data());
  int i = 0;
	if(inputfile.good()) {
		while(!inputfile.eof()) {
			string buf;	
			inputfile >> buf;
      cout << i << ": " << buf << std::endl;
			// TODO : construire la structure de données root à partir du fichier
      i++;
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
      j->_children.push_back(Joint::readChild(ifs, j));
      ifs >> buf;
  } else {
    while (buf == "JOINT") {
      j->_children.push_back(Joint::readChild(ifs, j));
      ifs >> buf;
    }
  }

  checkToken("}", buf);

  return j;
}

void Joint::animate(int iframe) 
{
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
	// Animate children :
	for (unsigned int ichild = 0 ; ichild < _children.size() ; ichild++) {
		_children[ichild]->animate(iframe);
	}
}


void Joint::nbDofs() {
	if (_dofs.empty()) return;

	double tol = 1e-4;

	int nbDofsR = -1;

	// TODO :
	cout << _name << " : " << nbDofsR << " degree(s) of freedom in rotation\n";

	// Propagate to children :
	for (unsigned int ichild = 0 ; ichild < _children.size() ; ichild++) {
		_children[ichild]->nbDofs();
	}

}
