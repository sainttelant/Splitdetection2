
#ifndef _Log_H_
#define _Log_H_

#include <iostream>

#define TRACE
#ifndef TRACE
 #define Ucitcout 0 && std::cout//或者NULL && Ucitcout
#else
 #define Ucitcout std::cout
#endif

#endif