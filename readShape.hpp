//
//  readShape.hpp
//  SimpleGlut
//
//  Created by Vishesh Javangula on 10/6/20.
//  Copyright © 2020 lab. All rights reserved.
//

#ifndef readShape_hpp
#define readShape_hpp

#include <stdio.h>
#include <vector>
#include <string>
#include <regex>
#include <sstream>

void storeVerticies(std::string line, std::vector<std::vector<double> >&vertices);
void storePointers(std::string line,  std::vector< std::vector<int> >& vertptrs);


#endif /* readShape_hpp */
