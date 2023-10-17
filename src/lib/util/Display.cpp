#include "Display.h"
#include "lockguard.h"

bool Screen::display() {
    if (screen == nullptr || loaded) {
        return false;
    }

    lv_scr_load(screen);
    loaded = true;

    return true;
}

Display::Display() : screens(), mutex(), task(nullptr) {
    orgScreen = lv_scr_act();
}

Display::~Display() {
    if (task) {
        lv_task_del(task);
    }

    lv_scr_load(orgScreen);
}

bool Display::show(std::string name, bool createTask, int period) {
    LockGuard guard(mutex);
    auto screen = screens.find(name);

    if (screen == screens.end()) {
        return false;
    }

    if (task != nullptr) {
        lv_task_del(task);
        task = nullptr;
    }

    // lambda to dispatch function
    if (createTask) {
        task = lv_task_create([](void* self) {
                                  reinterpret_cast<Screen*>(self)->update();
                              },
                              period, LV_TASK_PRIO_HIGH, screen->second.get());
    }

    return screen->second->display();
}

void Display::addScreen(std::string name, std::unique_ptr<Screen> screen) {
    mutex.take(TIMEOUT_MAX);
    screens.emplace(std::move(name), std::move(screen));
    mutex.give();
}

void Display::addScreen(std::string name, Screen* screen) {
    mutex.take(TIMEOUT_MAX);
    screens.emplace(std::move(name), screen);
    mutex.give();
}

void Display::removeScreen(std::string name) {
    LockGuard guard(mutex);

    auto screen = screens.find(name);

    if (screen == screens.end())
        return;

    if (screen->second->loaded) {
        lv_scr_load(orgScreen);
    }

    screens.erase(name);
}

// this 99.99% probably misuses the unique_ptr and shit but fight me
Screen* Display::getScreen(std::string name) {
    auto screen = screens.find(name);

    if (screen == screens.end())
        return nullptr;

    return screen->second.get();
}