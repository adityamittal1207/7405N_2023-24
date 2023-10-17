//
// Created by zayn on 8/4/21.
//

#ifndef INC_7405K_2021_2022_THREADING_H
#define INC_7405K_2021_2022_THREADING_H

#include "main.h"
#include <map>
#include <string>
#include <memory>

class Threading {
   private:
    int limit_;
    std::map<std::string, std::unique_ptr<pros::Task>> tasks;

   public:
    Threading(int limit);
    void start(std::string name, void (*func)(void *));
    void kill(std::string name);
    bool check_exists(std::string name);
};

#endif  //INC_7405K_2021_2022_THREADING_H