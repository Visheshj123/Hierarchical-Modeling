//
//  euler.hpp
//  SimpleGlut
//
//  Created by Vishesh Javangula on 9/27/20.
//  Copyright © 2020 lab. All rights reserved.
//

#ifndef euler_hpp
#define euler_hpp
#include <iomanip>
#include <stdio.h>
#include <GLUT/GLUT.h>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "key.hpp"
#include <string>
#include "node.hpp"


extern int ii;
extern GLfloat t;

struct arc{
    std::string id;
    Eigen::Matrix4f transformation; //for translation and rotation
    node* nodeptr; //associated node
    arc* left = NULL; //pointer to left child
    arc* right = NULL; //pointer to right child
};

extern arc body;
extern void forwardKinematics(arc* Arc, Eigen::Matrix4f matrix);

extern Eigen::Matrix4f m;
extern std::vector<key> vectorKeys;
extern void matrixToArray(Eigen::Matrix4f &m, GLfloat rotationMatrix[]);
Eigen::Matrix4f rotateEuler(GLfloat eAngles[3], GLfloat x, GLfloat y, GLfloat z);
void interpolateEuler();


#endif /* euler_hpp */
