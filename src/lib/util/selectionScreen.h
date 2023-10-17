#pragma once
#include "display.h"

enum class TeamSelection {
    MATCH1 = 1022,
    AWP = 1023,
    MATCH2 = 1024,
    MATCH3 = 1025,
    RIGHT_CENTER = 1026,
    UNKNOWN = 0,
};

class SelectionScreen : public Screen {
private:
    lv_obj_t* left;
    lv_obj_t* right;
    lv_obj_t* right_high;
    lv_obj_t* right_double;

    lv_obj_t* left_label;
    lv_obj_t* right_label;
    lv_obj_t* right_high_label;
    lv_obj_t* right_double_label;

    // lv_obj_t* red1;
    // lv_obj_t* red2;
    // lv_obj_t* blue1;
    // lv_obj_t* blue2;
    // lv_obj_t* label1;
    // lv_obj_t* label2;
    // lv_obj_t* label3;
    // lv_obj_t* label4;

public:
    SelectionScreen();

    void getSelection();
};