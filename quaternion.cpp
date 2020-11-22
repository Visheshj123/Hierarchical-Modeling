//
//  quaternion.cpp
//  SimpleGlut
//
//  Created by Vishesh Javangula on 9/25/20.
//  Copyright © 2020 lab. All rights reserved.
//

#include "quaternion.hpp"
#include <Eigen/Dense>
#include <GLUT/GLUT.h>
#include <vector>
#include <iostream>
#include <cmath>



//converts quaternion to rotoation matrix
void getQuatMatrix(Quaternion& q, Eigen::Matrix4f& qm, GLfloat x, GLfloat y, GLfloat z){
    qm << 1.0f - 2.0f*q.y*q.y - 2.0f*q.z*q.z, 2.0f*q.x*q.y - 2.0f*q.z*q.w,  2.0f*q.x*q.z + 2.0f*q.y*q.w, x,
           2.0f*q.x*q.y + 2.0f*q.z*q.w,  1.0f - 2.0f*q.x*q.x - 2.0f*q.z*q.z, 2.0f*q.y*q.z - 2.0f*q.x*q.w,  y,
           2.0f*q.x*q.z - 2.0f*q.y*q.w,  2.0f*q.y*q.z + 2.0f*q.x*q.w,  1.0f - 2.0f*q.x*q.x - 2.0f*q.y*q.y,z,
           0  ,0,0,1.0f;
    m1 = qm;
    q1 = q;
}

//takes euler angle converts into quaternion
void createQuat(Quaternion& q,GLfloat eAngles[], GLfloat x, GLfloat y, GLfloat z ){
    
        
        GLfloat attitude =(eAngles[0] * M_PI)/180;
        GLfloat heading =(eAngles[1] * M_PI)/180;
        GLfloat bank  = (eAngles[2] * M_PI)/180;
    
        GLfloat c1 = cos(heading * 0.5);
        GLfloat s1 = sin(heading * 0.5);
        GLfloat c2 = cos(attitude * 0.5);
        GLfloat s2 = sin(attitude * 0.5);
        GLfloat c3 = cos(bank * 0.5);
        GLfloat s3 = sin(bank * 0.5);
    
          q.w = c1 * c2 * c3 - s1 * s2 * s3;
          q.x = c1 * c2 * s3 + s1 * s2 * c3;
          q.y = s1 * c2 * c3 + c1 * s2 * s3;
          q.z = c1 * s2 * c3 - s1 * c2 * s3;
    
}

//main function that converts euler angles and translation compoents and loads transformation matrix
void rotateQuat(GLfloat eAngles[], GLfloat x, GLfloat y, GLfloat z){
    
        Quaternion q;
        createQuat(q, eAngles, x, y, z);
        Eigen::Matrix4f qm;
        getQuatMatrix(q,qm, x, y, z);
      


        GLfloat rotationMatrix[16];
        matrixToArray(qm, rotationMatrix);
    //

        
            glLoadMatrixf(rotationMatrix);
}




void interpolateQuat(){
   
  //  Eigen::Vector4f tVec;
    Eigen::Matrix4f M;
    Eigen::Matrix4f qm;
    GLfloat rotationMatrix[16];

    //create vector of quaternions from the vectorKeys to use for Geometrix matrix
    std::vector<Quaternion> qVec;
    for(int i=0;i<vectorKeys.size();i++){
    
            Quaternion q;
            createQuat(q, vectorKeys[i].Angles, vectorKeys[i].x, vectorKeys[i].y, vectorKeys[i].z);
      
            qVec.push_back(q);
        
    }
        
        //Geometrix Matrix
        Eigen::Matrix<GLfloat, 4, 7> p;
        p << vectorKeys[ii-3].x, vectorKeys[ii-3].y, vectorKeys[ii-3].z, qVec[ii-3].x, qVec[ii-3].y, qVec[ii-3].z, qVec[ii-3].w,
             vectorKeys[ii-2].x, vectorKeys[ii-2].y, vectorKeys[ii-2].z, qVec[ii-2].x, qVec[ii-2].y, qVec[ii-2].z, qVec[ii-2].w,
             vectorKeys[ii-1].x, vectorKeys[ii-1].y, vectorKeys[ii-1].z, qVec[ii-1].x, qVec[ii-1].y, qVec[ii-1].z, qVec[ii-1].w,
               vectorKeys[ii].x,   vectorKeys[ii].y,     vectorKeys[ii].z, qVec[ii].x,   qVec[ii].y, qVec[ii].z, qVec[ii].w;
                
            glPushMatrix();
            glLoadIdentity();



            
            Eigen::Vector4f tVec(pow(t,3), pow(t,2), pow(t,1), 1);



            Eigen::MatrixXf result;
        //result holds quaternion values and translation components given a certain t
            result = tVec.transpose() * m * p;
  
            
            Quaternion qres;
            qres.x = result(0,3);
            qres.y = result(0,4);
            qres.z = result(0,5);
            qres.w = result(0,6);
          
            //converts values into rotation amtrix
            getQuatMatrix(qres, qm, result(0,0), result(0,1), result(0,2));
            
            
            matrixToArray(qm,rotationMatrix);

            glLoadMatrixf(rotationMatrix);

            glutWireTeapot(1);
            glPopMatrix();

     
}
