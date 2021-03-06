#ifndef _JOINT_H_
#define _JOINT_H_

#include <string>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include "TriMesh.h"
#include <QMatrix4x4>
#include <QVector3D>


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
  QMatrix4x4 _curMat;
  int _glIdentifier;
	std::vector<float> weightOnPoints;
  static float frameTime;
  static int frames;
  static int glIdCounter;

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
			QMatrix4x4 makeMatrix;
			makeMatrix.setToIdentity();
			makeMatrix.translate(offX,offY,offZ);
			child->_curMat = parent->_curMat + makeMatrix;
		}
		else{
			QMatrix4x4 makeMatrix;
			makeMatrix.setToIdentity();
			makeMatrix.translate(offX, offY, offZ);
			child->_curMat = makeMatrix;
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


  int initalizeGLIds();
  void initGLIdsRec();

  void setVertices(trimesh::point *vertices, int iframe, Joint* parent=NULL);
  void setIndices(int *indices);
  void _setIndicesRec(int *indices, int &index);

	void readWeightFile(std::string fileName);
	void checkName(std::ifstream &ifs);
	void readWeight(std::ifstream &ifs);
  void skinModel(trimesh::TriMesh *skinMesh, int frame);
	void addJointWeight(QVector3D initVec, QVector3D &sumVec, int point);
};


#endif
