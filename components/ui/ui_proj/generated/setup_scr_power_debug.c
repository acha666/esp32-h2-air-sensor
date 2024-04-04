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



void setup_scr_power_debug(lv_ui *ui)
{
	//Write codes power_debug
	ui->power_debug = lv_obj_create(NULL);
	lv_obj_set_size(ui->power_debug, 200, 200);
	lv_obj_set_scrollbar_mode(ui->power_debug, LV_SCROLLBAR_MODE_OFF);

	//Write style for power_debug, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->power_debug, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

	//Write codes power_debug_text
	ui->power_debug_text = lv_label_create(ui->power_debug);
	lv_label_set_text(ui->power_debug_text, "Power: ");
	lv_label_set_long_mode(ui->power_debug_text, LV_LABEL_LONG_DOT);
	lv_obj_set_pos(ui->power_debug_text, 2, 2);
	lv_obj_set_size(ui->power_debug_text, 196, 196);

	//Write style for power_debug_text, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->power_debug_text, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->power_debug_text, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->power_debug_text, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->power_debug_text, &lv_font_IosevkaTerm_Bold_16, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_opa(ui->power_debug_text, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->power_debug_text, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->power_debug_text, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->power_debug_text, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->power_debug_text, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->power_debug_text, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->power_debug_text, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->power_debug_text, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->power_debug_text, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->power_debug_text, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

	//The custom code of power_debug.
	

	//Update current screen layout.
	lv_obj_update_layout(ui->power_debug);

}
