//
//  readShape.cpp
//  SimpleGlut
//
//  Created by Vishesh Javangula on 10/6/20.
//  Copyright © 2020 lab. All rights reserved.
//

#include "readShape.hpp"

void storeVerticies(std::string line,  std::vector<std::vector<double> >&vertices){
      std::vector<double> side;
      // cout << line << endl;
       std::smatch m;
       std::regex vertex ("[-]?[[:digit:]].[[:digit:]]");
       //regex dim ("")
       //regex r ("[[:digit:]]");
      

       while(regex_search(line,m,vertex)){ //returns true and ONLY writes first match to m then stops
           //cout << m[0] << " ";
           std::stringstream ss(m[0]);
           double x = 0.0;
           ss >> x;
           side.push_back(x);
           line = m.suffix(); //everything after the full match
       }
       vertices.push_back(side);
}
void storePointers(std::string line, std::vector< std::vector<int> >& vertptrs){
    std::smatch m;
       std::vector<int> side;
       std::regex pointers ("[^.]\\d(?!\\.)");
       while(regex_search(line, m, pointers)){
           //cout << m.str(0) << " ";
           //cout << m[1];
           std::string s1 = m.str(0);
           s1 = s1.substr(1,1);
           std::stringstream ss(s1);
           double x = 0.0;
           ss >> x;
           side.push_back(x);
            line = m.suffix(); //everything after the full match
       }
       
       
           vertptrs.push_back(side);
    
}
