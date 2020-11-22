//
//  euler.cpp
//  SimpleGlut
//
//  Created by Vishesh Javangula on 9/27/20.
//  Copyright © 2020 lab. All rights reserved.
//

#include "euler.hpp"
#include <stdio.h>
#include <GLUT/GLUT.h>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <math.h>
#include <iomanip>


//multiplies all rotation matrices together then loads it
Eigen::Matrix4f rotateEuler(GLfloat eAngles[3], GLfloat x, GLfloat y, GLfloat z){

    Eigen::Matrix4f result;
    Eigen::Matrix4f m;

    int i=0;

    GLfloat rad = 0;
    GLfloat cosine = 0;
    GLfloat sine = 0;

    while(i<3){
         rad = (eAngles[i] * M_PI)/180;
        cosine = cos(rad);
         sine = sin(rad);
        if(i == 0){
            m << cosine, -1*sine, 0, 0,
                sine, cosine, 0,0,
                0,0,1,0,
                0,0,0,1;
                result = m;
        }
        if(i == 1){
            m << cosine, 0,sine, 0,
                    0,1,0,0,
                    -1*sine, 0,cosine, 0,
                    0,0,0,1;
            result *= m;
        }
        if(i == 2){
            m << 1,0,0,0,
                0,cosine,-1*sine, 0,
                0,sine,cosine,0,
                0,0,0,1;
            result *= m;
        }
        i++;

    }

    //add on translation
   result.coeffRef(0,3) = x;
    result.coeffRef(1,3) = y;
   result.coeffRef(2,3) = z;
    

    return result;


}



//loads basis matrix, 4x6 geometric matrix, and row-vector to generate Q(t)
void interpolateEuler(){
    Eigen::Matrix4f M;
    
    
        Eigen::Matrix<GLfloat, 4, 6> p;
      p << vectorKeys[ii-3].x, vectorKeys[ii-3].y, vectorKeys[ii-3].z, vectorKeys[ii-3].Angles[0], vectorKeys[ii-3].Angles[1], vectorKeys[ii-3].Angles[2],
           vectorKeys[ii-2].x, vectorKeys[ii-2].y, vectorKeys[ii-2].z, vectorKeys[ii-2].Angles[0], vectorKeys[ii-2].Angles[1], vectorKeys[ii-2].Angles[2],
           vectorKeys[ii-1].x, vectorKeys[ii-1].y, vectorKeys[ii-1].z, vectorKeys[ii-1].Angles[0], vectorKeys[ii-1].Angles[1],vectorKeys[ii-1].Angles[2],
           vectorKeys[ii].x, vectorKeys[ii].y, vectorKeys[ii].z, vectorKeys[ii].Angles[0], vectorKeys[ii].Angles[1],vectorKeys[ii].Angles[2];
    
    Eigen::Vector4f tVec(pow(t,3), pow(t,2), pow(t,1), 1);


 
        Eigen::MatrixXf result;
        result = tVec.transpose() * m * p;
    
    //create array of angles to pass into rotateEuler
    GLfloat EulerAngles[3] = {result(0,3), result(0,4), result(0,5)};
    
    Eigen::Matrix4f rMatrix = rotateEuler(EulerAngles, result(0,0), result(0,1), result(0,2));
    
    forwardKinematics(&body, rMatrix);

    
    
}


