/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "lvgl.h"

void example_lvgl_demo_ui(lv_disp_t *disp, float voltios, float ms, float div, int escala, bool recibido)
{
    lv_obj_t *scr = lv_disp_get_scr_act(disp);
    lv_obj_clean(scr);
    lv_obj_t *label = lv_label_create(scr);
    if(recibido==false){
	    if(escala==0){
			lv_label_set_text_fmt(label, "V/div: %.1f\nms/div: %.1f\nDiv: %.1f\nEscala:1",voltios,ms,div);
		}else{
			lv_label_set_text_fmt(label, "V/div: %.1f\nms/div: %.1f\nDiv: %.1f\nEscala:10",voltios,ms,div);
		}		
	}else{
		if(escala==0){
			lv_label_set_text_fmt(label, "Frec: %.1f\nPK2PK: %.1f\nOff: %.1f\nSINUSOID",voltios,2*ms,div);			
		}else{
			lv_label_set_text_fmt(label, "Frec: %.1f\nPK2PK: %.1f\nOff: %.1f\nSQUARE",voltios,2*ms,div);
		}
	}

    lv_obj_set_width(label, disp->driver->hor_res);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);
    lv_disp_flush_ready(disp);
}
