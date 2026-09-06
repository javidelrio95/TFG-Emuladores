/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "hal/lv_hal_disp.h"
#include "lvgl.h"
#include "widgets/lv_label.h"

void example_lvgl_demo_ui(lv_disp_t *disp, float frec, float ampli,
                          float offset, int forma) {
  lv_obj_t *scr = lv_disp_get_scr_act(disp);
  lv_obj_clean(scr);
  lv_obj_t *label = lv_label_create(scr);
  // lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular
  // scroll */
  if (forma == 0) {
    lv_label_set_text_fmt(label,
                          "Frec: %.1f\nAmp: %.1f\nOffset: %.1f\nSINUSOID",
                          frec, ampli, offset);
  } else {
    lv_label_set_text_fmt(label, "Frec: %.1f\nAmp: %.1f\nOffset: %.1f\nSQUARE",
                          frec, ampli, offset);
  }
  /* Size of the screen (if you use rotation 90 or 270, please set
   * disp->driver->ver_res) */
  lv_obj_set_width(label, disp->driver->hor_res);
  lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);
  lv_disp_flush_ready(disp);
}
