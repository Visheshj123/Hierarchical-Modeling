//
//  pelvis.hpp
//  SimpleGlut
//
//  Created by Vishesh Javangula on 10/6/20.
//  Copyright © 2020 lab. All rights reserved.
//

#ifndef node_hpp
#define node_hpp

#include <stdio.h>
#include <Eigen/Dense>
#include <GLUT/GLUT.h>
#include <vector>
#include <fstream>
extern void matrixToArray(Eigen::Matrix4f &m, GLfloat rotationMatrix[]);
class node{
    Eigen::Matrix4f m; //default matrix that sets position
    std::vector<std::vector<double> > vertices;
    std::vector< std::vector<int> > vertptrs;
    void initializeShape(std::string type);
    
public:
    node(std::string type);
    void display();
   
    
    Eigen::Matrix4f getMatrix();
    void setMatrix(Eigen::Matrix4f transformation);
};

#endif /* node_hpp */
