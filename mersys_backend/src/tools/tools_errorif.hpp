
#pragma once

#include "os_compatible.hpp"
#include <fstream>
#include <iostream>
#include <map>
#include <mutex>
#include <ostream>
#include <sstream>
#include <stdarg.h> //need for such like printf(...)
#include <stdio.h>
#include <string>
#include <thread>
#include <iomanip>
#include "tools_color_printf.hpp"
#include "tools_timer.hpp"

void errif(bool condition, const char *errmsg){
    if(condition){
        perror(errmsg);
        exit(EXIT_FAILURE);
    }
}