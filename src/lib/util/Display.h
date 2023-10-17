#pragma once

#include "pros/apix.h"
#include "main.h"
#include <map>
#include <string>

/*
Supposedly threadsafe, however it is not intended to be used like that.
Most times, all screens needed while be created on initialization,
and most updates to the gui from then on should be in lv_task.
Only the active screen will have its values updated.
LVGL itself is not threadsafe, and therefore any calls need to be wrapped with a mutex or some guard. (not being done currently)
All screens/UI will be implemented from classes that inherit from here.
Documentation:
https://github.com/lvgl/docs_old/blob/7db14b1aad135839f68843a65269d9ca75b3a81f/en/docs_v5_3.zip
LVGL:
https://github.com/lvgl/lv_sim_visual_studio/commit/bd542fd9895660f013ea7435aeee60fb40a2a1f2
Simulator (Windows):
https://github.com/lvgl/lv_sim_visual_studio/tree/e607e3e02d0b4506d0761a47053547043d87cd7a
*/

class Display;

/**
 * @brief Wrapper class for all custom screens. Custom screens shall inherit from this.
 * There are two specific circumstances for when you are allowed to call LVGL functions.
 * One of them is in the update function if it is needed to update the contents of the screen.
 * The other time is in the constructor, however, it has to be before any screen has been shown.
 * All screens used in the entire program shall be created before any screen gets shown in hopes of
 * guarenteeing that the single threaded library works fine.
 */
class Screen {
    friend Display;

private:
    bool display();

protected:
    lv_obj_t* screen;

    /**
     * @brief Whether or not this screen is currently
     * being displayed.
     */
    bool loaded;

    static inline lv_coord_t percentX(double percent) {
        return 480 / 100 * percent;
    }

    static inline lv_coord_t percentY(double percent) {
        return 240 / 100 * percent;
    }

    /**
     * @brief Called every so often to update the contents of the screen.
     * One of the only two places where calls to lvgl are acceptable.
     */
    virtual void update(){};

public:
    Screen() : loaded(false) { screen = lv_obj_create(NULL, NULL); }
    virtual ~Screen() { lv_obj_del(screen); }

    bool showing() const { return loaded; }
};

/**
 * @brief UI manager controlling current screen that is showing and its task.
 */
class Display {
private:
    std::map<std::string, std::unique_ptr<Screen>> screens;
    pros::Mutex mutex;
    lv_task_t* task;      // current task that is being executed
    lv_obj_t* orgScreen;  // original screen

public:
    Display();
    ~Display();

    /**
     * @brief Shows the screen on the brain.
     *
     * @param name Name of the screen.
     * @param createTask Whether or not the screen uses the update function.
     * @param period How often the task associated to the screen gets executed (ms)
     * @return Whether or not the screen is showing.
     */
    bool show(std::string name, bool createTask = false, int period = 50);

    /**
     * @brief Adds a screen and makes it available to be shown. Takes ownership of the screen.
     *
     * @param name Screen name used by the display manager to keep track of every screen.
     * @param screen Unique pointer to a screen.
     */
    void addScreen(std::string name, std::unique_ptr<Screen> screen);

    /**
     * @brief Adds a screen and makes it available to be shown. Takes ownership of the screen.
     *
     * @param name Screen name used by the display manager to keep track of every screen.
     * @param screen Pointer to a screen. Takes ownership.
     */
    void addScreen(std::string name, Screen* screen);

    /**
     * @brief Removes screen from available screens and cleans
     * up its resources
     * @param name Name of the screen.
     */
    void removeScreen(std::string name);

    Screen* getScreen(std::string name);
};