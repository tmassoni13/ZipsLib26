/*
    gui.cpp

    Builds and updates the brain screen GUI for pose data and auton selection.
*/

#include "includes.h"
#include "Chassis/ChassisController.hpp"
#include "pros/apix.h"
#include "liblvgl/lvgl.h"
#include <cstdio>
#include <cstring>

static lv_obj_t* label_x = nullptr;
static lv_obj_t* label_y = nullptr;
static lv_obj_t* label_theta = nullptr;
static lv_obj_t* label_x_select = nullptr;
static lv_obj_t* label_y_select = nullptr;
static lv_obj_t* label_theta_select = nullptr;
static lv_obj_t* right_box = nullptr;
static lv_obj_t* label_auton = nullptr;
static lv_obj_t* label_auton_status = nullptr;

static lv_obj_t* screen_default = nullptr;
static lv_obj_t* screen_select = nullptr;

static lv_style_t style_screen;
static lv_style_t style_box;
static lv_style_t style_text;

static const char* selected_auton_name = "None";
static AutonChoice selected_auton = AutonNone;

static void guiUpdateTask(void*) {
  while (true) {
    const int now = pros::millis();
    const int last = zips::chassis.getLastOtosUpdateMs();
    const bool hasOtosData = last >= 0 && (now - last) <= 500;
    const bool hasHeadingData =
      zips::chassis.getHeadingSource() == zips::HeadingSource::Imu || hasOtosData;

    if (!hasOtosData) {
      const char* xtext = "X: No Data";
      const char* ytext = "Y: No Data";
      if (label_x) lv_label_set_text(label_x, xtext);
      if (label_y) lv_label_set_text(label_y, ytext);
      if (label_x_select) lv_label_set_text(label_x_select, xtext);
      if (label_y_select) lv_label_set_text(label_y_select, ytext);
    } else {
      zips::Pose2D pose = zips::chassis.getPose();
      char buf[64];
      std::snprintf(buf, sizeof(buf), "X: %.2f", pose.x);
      if (label_x) lv_label_set_text(label_x, buf);
      if (label_x_select) lv_label_set_text(label_x_select, buf);
      std::snprintf(buf, sizeof(buf), "Y: %.2f", pose.y);
      if (label_y) lv_label_set_text(label_y, buf);
      if (label_y_select) lv_label_set_text(label_y_select, buf);
    }

    if (hasHeadingData) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "T: %.2f", zips::chassis.getHeading());
      if (label_theta) lv_label_set_text(label_theta, buf);
      if (label_theta_select) lv_label_set_text(label_theta_select, buf);
    } else {
      const char* ttext = "T: No Data";
      if (label_theta) lv_label_set_text(label_theta, ttext);
      if (label_theta_select) lv_label_set_text(label_theta_select, ttext);
    }

    pros::delay(50);
  }
}

static lv_obj_t* create_outline_button(lv_obj_t* parent, int w, int h) {
  lv_obj_t* btn = lv_btn_create(parent);
  lv_obj_set_size(btn, w, h);
  lv_obj_add_style(btn, &style_box, 0);
  return btn;
}

static lv_obj_t* create_button_label(lv_obj_t* btn, const char* text) {
  lv_obj_t* label = lv_label_create(btn);
  lv_label_set_text(label, text);
  lv_obj_center(label);
  lv_obj_add_style(label, &style_text, 0);
  return label;
}

static void create_pose_labels(lv_obj_t* parent,
                               lv_obj_t** out_x,
                               lv_obj_t** out_y,
                               lv_obj_t** out_theta) {
  lv_obj_t* lx = lv_label_create(parent);
  lv_label_set_text(lx, "X: No Data");
  lv_obj_align(lx, LV_ALIGN_LEFT_MID, 12, -24);
  lv_obj_add_style(lx, &style_text, 0);

  lv_obj_t* ly = lv_label_create(parent);
  lv_label_set_text(ly, "Y: No Data");
  lv_obj_align(ly, LV_ALIGN_LEFT_MID, 12, 0);
  lv_obj_add_style(ly, &style_text, 0);

  lv_obj_t* lt = lv_label_create(parent);
  lv_label_set_text(lt, "T: No Data");
  lv_obj_align(lt, LV_ALIGN_LEFT_MID, 12, 24);
  lv_obj_add_style(lt, &style_text, 0);

  if (out_x) *out_x = lx;
  if (out_y) *out_y = ly;
  if (out_theta) *out_theta = lt;
}

static void show_screen(lv_obj_t* screen) {
  if (!screen) return;
  lv_scr_load(screen);
}

static void set_selected_auton(const char* name) {
  selected_auton_name = name ? name : "None";
  if (name && std::strcmp(name, "Skills") == 0) selected_auton = AutonSkills;
  else if (name && std::strcmp(name, "Match") == 0) selected_auton = AutonMatch;
  else selected_auton = AutonNone;
  if (label_auton) {
    lv_label_set_text(label_auton, selected_auton_name);
  }
  if (label_auton_status) {
    lv_label_set_text(label_auton_status, "Selected");
  }
}

static void on_open_select(lv_event_t*) {
  show_screen(screen_select);
}

static void on_back_default(lv_event_t*) {
  show_screen(screen_default);
}

static void on_select_skills(lv_event_t*) {
  set_selected_auton("Skills");
  show_screen(screen_default);
}

static void on_select_match(lv_event_t*) {
  set_selected_auton("Match");
  show_screen(screen_default);
}

void guiInit() {
  lv_obj_t* scr = lv_scr_act();
  lv_obj_clean(scr);

  lv_style_init(&style_screen);
  lv_style_set_bg_color(&style_screen, lv_color_black());
  lv_style_set_bg_opa(&style_screen, LV_OPA_COVER);

  lv_style_init(&style_box);
  lv_style_set_bg_opa(&style_box, LV_OPA_TRANSP);
  lv_style_set_border_color(&style_box, lv_color_white());
  lv_style_set_border_width(&style_box, 2);
  lv_style_set_radius(&style_box, 8);
  lv_style_set_pad_all(&style_box, 6);

  lv_style_init(&style_text);
  lv_style_set_text_color(&style_text, lv_color_white());

  // ---------------- Default Screen ----------------
  screen_default = lv_obj_create(NULL);
  lv_obj_add_style(screen_default, &style_screen, 0);

  create_pose_labels(screen_default, &label_x, &label_y, &label_theta);

  right_box = create_outline_button(screen_default, 140, 50);
  lv_obj_set_size(right_box, 120, 50);
  lv_obj_align(right_box, LV_ALIGN_RIGHT_MID, -12, 0);
  create_button_label(right_box, "Select Autons");
  lv_obj_add_event_cb(right_box, on_open_select, LV_EVENT_CLICKED, nullptr);

  label_auton = lv_label_create(screen_default);
  lv_label_set_text(label_auton, selected_auton_name);
  lv_obj_align(label_auton, LV_ALIGN_RIGHT_MID, -12, 48);
  lv_obj_add_style(label_auton, &style_text, 0);

  label_auton_status = lv_label_create(screen_default);
  lv_label_set_text(label_auton_status, "");
  lv_obj_align(label_auton_status, LV_ALIGN_RIGHT_MID, -12, 68);
  lv_obj_add_style(label_auton_status, &style_text, 0);

  // ---------------- Select Screen ----------------
  screen_select = lv_obj_create(NULL);
  lv_obj_add_style(screen_select, &style_screen, 0);
  create_pose_labels(screen_select, &label_x_select, &label_y_select, &label_theta_select);

  lv_obj_t* skills_btn = create_outline_button(screen_select, 120, 50);
  lv_obj_align(skills_btn, LV_ALIGN_CENTER, -70, 0);
  create_button_label(skills_btn, "Skills");
  lv_obj_add_event_cb(skills_btn, on_select_skills, LV_EVENT_CLICKED, nullptr);

  lv_obj_t* match_btn = create_outline_button(screen_select, 120, 50);
  lv_obj_align(match_btn, LV_ALIGN_CENTER, 70, 0);
  create_button_label(match_btn, "Match");
  lv_obj_add_event_cb(match_btn, on_select_match, LV_EVENT_CLICKED, nullptr);

  lv_obj_t* back_btn_select = create_outline_button(screen_select, 120, 40);
  lv_obj_align(back_btn_select, LV_ALIGN_BOTTOM_MID, 0, -10);
  create_button_label(back_btn_select, "Back");
  lv_obj_add_event_cb(back_btn_select, on_back_default, LV_EVENT_CLICKED, nullptr);

  show_screen(screen_default);
  pros::Task(guiUpdateTask, nullptr, "GUI");
}

AutonChoice getSelectedAuton() {
  return selected_auton;
}
