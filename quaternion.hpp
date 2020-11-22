//
//  quaternion.hpp
//  SimpleGlut
//
//  Created by Vishesh Javangula on 9/25/20.
//  Copyright © 2020 lab. All rights reserved.
//

#ifndef quaternion_hpp
#define quaternion_hpp

#include <stdio.h>
#include <GLUT/GLUT.h>
#include <Eigen/Dense>
#include <vector>
#include <unordered_map>
#include <utility>
#include <iomanip>
#include "key.hpp"
//#include <iostream>

//tells linker to look elsewhere for this

struct Quaternion
{
    GLfloat w = 0.0;
    GLfloat x = 0.0;
    GLfloat y = 0.0;
    GLfloat z = 0.0;
};

//need this here otherwise c++ has no idea what a key is and wont be able to find vectorKeys


extern std::vector<key> vectorKeys;
extern Eigen::Matrix4f m;
extern std::vector<std::pair<GLfloat, GLfloat>> arclength_time_pairs;
extern GLfloat distance(GLfloat x1, GLfloat y1,GLfloat z1, GLfloat x2, GLfloat y2, GLfloat z2);
extern GLfloat t;
extern void matrixToArray(Eigen::Matrix4f &m, GLfloat rotationMatrix[]);
extern int ii;
extern Eigen::MatrixXf m1;
extern Quaternion q1;


//converts final matrix to array, which is needed in order to load the matrix with GLUT
//void matrixToArray(Eigen::Matrix4f &m, GLfloat rotationMatrix[]);

//uses quaternion values to create the neccessary matrix needed for rotation and translation and reads it into qm
void getQuatMatrix(Quaternion& q, Eigen::Matrix4f& qm, GLfloat x, GLfloat y, GLfloat z);

//converts Euler angles to quaternion, takes in euler angles and x,y,z translation
void createQuat(Quaternion& q,GLfloat eAngles[]);

/*
 given euler angles and x,y,z translation, calls createQuat() to make Quaternion, then converts to matrix
 with getQuatMatrix(), then creates array using MatrixtoArray() and loads the result into OpenGL, also used for boxes
 */
void rotateQuat(GLfloat eAngles[], GLfloat x, GLfloat y, GLfloat z);

//handles interpolation of quaternion over the key-frames 
void interpolateQuat();
#endif /* quaternion_hpp */
