//
//  pelvis.cpp
//  SimpleGlut
//
//  Created by Vishesh Javangula on 10/6/20.
//  Copyright © 2020 lab. All rights reserved.
//

#include "node.hpp"
#include <stdio.h>
#include <Eigen/Dense>
#include <iostream>
#include <GLUT/GLUT.h>
#include "readShape.hpp"
using namespace std;

node::node(std::string type){
    m << 1,0,0,0,
    0,1,0,0,
    0,0,1,-5,
    0,0,0,1;
    initializeShape(type);
    
};

void node::initializeShape(std::string type){
    std::ifstream myfile;
       std::string line;
       int lineCount = 0;
    std::string filename = "/Users/vishesh.javangula@ibm.com/Code_Proj/Hierarchical-Modeling/data-files/" + type;
           myfile.open(filename);
       if(myfile.is_open()){
           while(getline(myfile, line)){
              
                   storeVerticies(line, vertices);
               //if(lineCount >= 9){
                   storePointers(line,vertptrs);
               //}
               lineCount++;
                   
                   
               
           }
       }
        myfile.close();
}

void node::display(){
    GLfloat rotationMatrix[16];
    matrixToArray(m, rotationMatrix);
    glPushMatrix();
    glLoadMatrixf(rotationMatrix);
         glBegin(GL_QUADS);
        for(int i=1;i<vertptrs.size();i++){
            for(int j=1;j<vertptrs[i].size();j++){
                int ptr = vertptrs[i][j];
                 glVertex3f(vertices[ptr][0], vertices[ptr][1], vertices[ptr][2]);
                
            }
        }
        glEnd();
    glPopMatrix();
}

Eigen::Matrix4f node::getMatrix(){
    return m;
}

void node::setMatrix(Eigen::Matrix4f transformation){
    m = transformation;
}
