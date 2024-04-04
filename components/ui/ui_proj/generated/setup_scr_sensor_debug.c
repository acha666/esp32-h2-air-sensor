/*
* Copyright 2024 NXP
* NXP Confidential and Proprietary. This software is owned or controlled by NXP and may only be used strictly in
* accordance with the applicable license terms. By expressly accepting such terms or by downloading, installing,
* activating and/or otherwise using the software, you are agreeing that you have read, and that you agree to
* comply with and are bound by, such license terms.  If you do not agree to be bound by the applicable license
* terms, then you may not retain, install, activate or otherwise use the software.
*/

#include "lvgl.h"
#include <stdio.h>
#include "gui_guider.h"
#include "events_init.h"
#include "widgets_init.h"
#include "custom.h"



void setup_scr_sensor_debug(lv_ui *ui)
{
	//Write codes sensor_debug
	ui->sensor_debug = lv_obj_create(NULL);
	lv_obj_set_size(ui->sensor_debug, 200, 200);
	lv_obj_set_scrollbar_mode(ui->sensor_debug, LV_SCROLLBAR_MODE_ACTIVE);

	//Write style for sensor_debug, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->sensor_debug, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

	//Write codes sensor_debug_text
	ui->sensor_debug_text = lv_label_create(ui->sensor_debug);
	lv_label_set_text(ui->sensor_debug_text, "Temp Sensor: \nSerial: \nCur. Temp: \nCur. Humi: ");
	lv_label_set_long_mode(ui->sensor_debug_text, LV_LABEL_LONG_DOT);
	lv_obj_set_pos(ui->sensor_debug_text, 2, 2);
	lv_obj_set_size(ui->sensor_debug_text, 196, 196);

	//Write style for sensor_debug_text, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->sensor_debug_text, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->sensor_debug_text, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->sensor_debug_text, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->sensor_debug_text, &lv_font_IosevkaTerm_Bold_16, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->sensor_debug_text, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->sensor_debug_text, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->sensor_debug_text, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->sensor_debug_text, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->sensor_debug_text, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->sensor_debug_text, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->sensor_debug_text, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->sensor_debug_text, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->sensor_debug_text, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->sensor_debug_text, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

	//The custom code of sensor_debug.
	

	//Update current screen layout.
	lv_obj_update_layout(ui->sensor_debug);

}
