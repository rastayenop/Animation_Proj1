#ifndef _JOINT_H_
#define _JOINT_H_
#define GLM_ENABLE_EXPERIMENTAL

#include <string>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include "glm/glm/glm.hpp"
#include <glm/glm/gtc/matrix_transform.hpp>
#include <glm/glm/gtx/euler_angles.hpp>

class AnimCurve {
public :
	AnimCurve() {};
	~AnimCurve() {
		_values.clear();
	}
public :
	std::string name;					// name of dof
	std::vector<double> _values;		// different keyframes = animation curve
};


enum RotateOrder {roXYZ=0, roYZX, roZXY, roXZY, roYXZ, roZYX};

class Joint {
public :
	std::string _name;					// name of joint
	double _offX;						// initial offset in X
	double _offY;						// initial offset in Y
	double _offZ;						// initial offset in Z
	std::vector<AnimCurve> _dofs;		// keyframes : _animCurves[i][f] = i-th dof at frame f;
	double _curTx;						// current value of translation on X
	double _curTy;						// current value of translation on Y
	double _curTz;						// current value of translation on Z
	double _curRx;						// current value of rotation about X (deg)
	double _curRy;						// current value of rotation about Y (deg)
	double _curRz;						// current value of rotation about Z (deg)
	int _rorder;						// order of euler angles to reconstruct rotation
	std::vector<Joint*> _children;	// children of the current joint
  glm::mat4 _curMat;

public :
	// Constructor :
	Joint() {};
	// Destructor :
	~Joint() {
		_dofs.clear();
		_children.clear();
	}

	// Create from data :
	static Joint* create(std::string name, double offX, double offY, double offZ, Joint* parent) {
		Joint* child = new Joint();
		child->_name = name;
		child->_offX = offX;
		child->_offY = offY;
		child->_offZ = offZ;
		child->_curTx = 0;
		child->_curTy = 0;
		child->_curTz = 0;
		child->_curRx = 0;
		child->_curRy = 0;
		child->_curRz = 0;
		if(parent != NULL) {
			parent->_children.push_back(child);
			glm::mat4 makeMatrix = glm::mat4(0.0);
			makeMatrix[3] = glm::vec4(offX, offY, offZ, 0.);
			child->_curMat = parent->_curMat + glm::transpose(makeMatrix);
		}
		else{
			glm::mat4 makeMatrix = glm::mat4(1.0);
			makeMatrix[3] = glm::vec4(offX, offY, offZ, 1.);
			child->_curMat = glm::transpose(makeMatrix);
		}

		return child;
	}

  static void checkToken(std::string expected, std::string buf);

  static Joint* readChild(std::ifstream &ifs, Joint* parent);
	static void readMotion(std::ifstream &ifs, Joint* node);

	// Load from file (.bvh) :
	static Joint* createFromFile(std::string fileName);

	void animate(Joint* parent, int iframe=0);
	void updateMatrix(Joint* parent	);

	// Analysis of degrees of freedom :
	void nbDofs();

  void printJoint3DPoints();
  void printJoin3DPointsRec(std::ofstream &file, float x, float y, float z, Joint* parent	= NULL);
};


#endif
