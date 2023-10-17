//
// Created by zayn on 8/4/21.
//

#include "Threading.h"

Threading::Threading(int limit) : limit_(limit) {}

void Threading::start(std::string name, void (*func)(void *)) {
    if (!check_exists(name) && tasks.size() < limit_) {
        tasks.emplace(std::move(name), new pros::Task(func));
    }
}

void Threading::kill(std::string name) {
    if (check_exists(name)) {
        tasks.at(name)->remove();
        tasks.erase(name);
    }
}

bool Threading::check_exists(std::string name) {
    return tasks.find(name) != tasks.end();
}