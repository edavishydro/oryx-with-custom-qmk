#include QMK_KEYBOARD_H
#include "version.h"

#ifndef ZSA_SAFE_RANGE
#define ZSA_SAFE_RANGE SAFE_RANGE
#endif

enum custom_keycodes {
  RGB_SLD = ZSA_SAFE_RANGE,
};


enum tap_dance_codes {
  DANCE_0,
  DANCE_1,
  DANCE_2,
};

#define DUAL_FUNC_0 LT(9, KC_F23)
#define DUAL_FUNC_1 LT(5, KC_C)
#define DUAL_FUNC_2 LT(1, KC_P)
#define DUAL_FUNC_3 LT(12, KC_T)

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [0] = LAYOUT_ergodox_pretty(
    KC_ESCAPE,      KC_1,           KC_2,           KC_3,           KC_4,           KC_5,           TG(1),                                          TG(2),          KC_6,           KC_7,           KC_8,           KC_9,           KC_0,           KC_MINUS,
    TD(DANCE_0),    KC_Q,           KC_W,           KC_E,           KC_R,           KC_T,           TG(3),                                          LGUI(LCTL(LSFT(KC_4))),KC_Y,           KC_U,           KC_I,           KC_O,           KC_P,           KC_BSLS,
    KC_LEFT_CTRL,   KC_A,           KC_S,           KC_D,           MT(MOD_LSFT, KC_F),KC_G,                                                                           MT(MOD_RSFT, KC_H),KC_J,           KC_K,           KC_L,           KC_SCLN,        KC_QUOTE,
    SC_LSPO,        KC_Z,           DUAL_FUNC_0,    DUAL_FUNC_1,    TD(DANCE_1),    KC_B,           KC_TILD,                                        TD(DANCE_2),    KC_N,           KC_M,           KC_COMMA,       KC_DOT,         KC_SLASH,       SC_RSPC,
    KC_GRAVE,       KC_MEDIA_PLAY_PAUSE,KC_LEFT_ALT,    KC_LEFT,        KC_RIGHT,                                                                                                       KC_UP,          KC_DOWN,        DUAL_FUNC_2,    DUAL_FUNC_3,    LGUI(LCTL(KC_SPACE)),
                                                                                                    LT(1, KC_AUDIO_VOL_DOWN),LGUI(LSFT(KC_LBRC)),LGUI(LSFT(KC_RBRC)),LT(2, KC_AUDIO_VOL_UP),
                                                                                                                    MT(MOD_LCTL, KC_DELETE),MT(MOD_LGUI | MOD_LSFT, KC_PAGE_UP),
                                                                                    MT(MOD_LGUI, KC_TAB),KC_BSPC,        MT(MOD_LGUI, KC_ENTER),MT(MOD_LGUI | MOD_LSFT, KC_EQUAL),KC_ENTER,       KC_SPACE
  ),
  [1] = LAYOUT_ergodox_pretty(
    KC_ESCAPE,      KC_F1,          KC_F2,          KC_F3,          KC_F4,          KC_F5,          KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_F6,          KC_F7,          KC_F8,          KC_F9,          KC_F10,         KC_TRANSPARENT,
    KC_NO,          KC_GRAVE,       KC_LABK,        KC_RABK,        KC_MINUS,       KC_PIPE,        KC_NO,                                          KC_NO,          KC_CIRC,        KC_LCBR,        KC_RCBR,        KC_DLR,         KC_ASTR,        KC_F11,
    KC_NO,          KC_EXLM,        KC_ASTR,        KC_SLASH,       KC_EQUAL,       KC_AMPR,                                                                        KC_HASH,        KC_LPRN,        KC_RPRN,        KC_SCLN,        KC_DQUO,        KC_F12,
    KC_TRANSPARENT, KC_TILD,        KC_PLUS,        KC_LBRC,        KC_RBRC,        KC_PERC,        KC_NO,                                          KC_NO,          KC_AT,          KC_COLN,        KC_COMMA,       KC_DOT,         KC_QUOTE,       KC_TRANSPARENT,
    KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,                                                                                                          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,
                                                                                                    KC_TRANSPARENT, RGB_SLD,        RGB_TOG,        KC_TRANSPARENT,
                                                                                                                    RGB_MODE_FORWARD,KC_TRANSPARENT,
                                                                                    RGB_VAD,        RGB_VAI,        KC_TRANSPARENT, KC_TRANSPARENT, RGB_HUD,        RGB_HUI
  ),
  [2] = LAYOUT_ergodox_pretty(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_SYSTEM_SLEEP,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_MS_BTN2,     KC_MS_UP,       KC_MS_BTN1,     KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_MS_LEFT,     KC_MS_DOWN,     KC_MS_RIGHT,    KC_TRANSPARENT,                                                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_MEDIA_PLAY_PAUSE,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_MEDIA_PREV_TRACK,KC_MEDIA_PLAY_PAUSE,KC_MEDIA_NEXT_TRACK,KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_MS_BTN1,     KC_MS_BTN2,                                                                                                     KC_AUDIO_VOL_UP,KC_AUDIO_VOL_DOWN,KC_AUDIO_MUTE,  KC_TRANSPARENT, KC_TRANSPARENT,
                                                                                                    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
                                                                                                                    KC_TRANSPARENT, KC_TRANSPARENT,
                                                                                    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_WWW_BACK
  ),
  [3] = LAYOUT_ergodox_pretty(
    KC_ESCAPE,      KC_1,           KC_2,           KC_3,           KC_4,           KC_5,           TG(3),                                          TG(2),          KC_6,           KC_7,           KC_8,           KC_9,           KC_0,           KC_LBRC,
    TD(DANCE_0),    KC_B,           KC_L,           KC_D,           KC_W,           KC_Z,           KC_TRANSPARENT,                                 LGUI(LCTL(LSFT(KC_4))),KC_QUOTE,       KC_F,           KC_O,           KC_U,           KC_J,           KC_SCLN,
    KC_LEFT_CTRL,   KC_N,           KC_R,           KC_T,           MT(MOD_LSFT, KC_S),KC_G,                                                                       KC_Y,           MT(MOD_RSFT, KC_H),KC_A,           KC_E,           KC_I,           KC_COMMA,
    SC_LSPO,        KC_Q,           KC_X,           KC_M,           KC_C,           KC_V,           KC_TILD,                                        TD(DANCE_2),    KC_K,           KC_P,           KC_DOT,         KC_MINUS,       KC_SLASH,       SC_RSPC,
    KC_GRAVE,       KC_MEDIA_PLAY_PAUSE,KC_LEFT_ALT,    KC_LEFT,        KC_RIGHT,                                                                                                       KC_UP,          KC_DOWN,        DUAL_FUNC_2,    DUAL_FUNC_3,    LGUI(LCTL(KC_SPACE)),
                                                                                                    LT(1, KC_AUDIO_VOL_DOWN),LGUI(LSFT(KC_LBRC)),LGUI(LSFT(KC_RBRC)),LT(2, KC_AUDIO_VOL_UP),
                                                                                                                    MT(MOD_LCTL, KC_DELETE),MT(MOD_LGUI | MOD_LSFT, KC_PAGE_UP),
                                                                                    MT(MOD_LGUI, KC_TAB),KC_BSPC,        MT(MOD_LGUI, KC_ENTER),MT(MOD_LGUI | MOD_LSFT, KC_EQUAL),KC_ENTER,       KC_SPACE
  ),
};


const char chordal_hold_layout[MATRIX_ROWS][MATRIX_COLS] PROGMEM = LAYOUT_ergodox(
  'L', 'L', 'L', 'L', 'L', 'L', 'L', 
  'L', 'L', 'L', 'L', 'L', 'L', 'L', 
  'L', 'L', 'L', 'L', 'L', 'L', 
  'L', 'L', 'L', 'L', 'L', 'L', 'L', 
  'L', 'L', 'L', 'L', 'L',
  '*', '*',
  '*', '*', '*', '*',
  'R', 'R', 'R', 'R', 'R', 'R', 'R', 
  'R', 'R', 'R', 'R', 'R', 'R', 'R', 
  'R', 'R', 'R', 'R', 'R', 'R', 
  'R', 'R', 'R', 'R', 'R', 'R', 'R', 
  'R', 'R', 'R', 'R', 'R', 
  '*', '*',
  '*', '*', '*', '*'
);




extern rgb_config_t rgb_matrix_config;

RGB hsv_to_rgb_with_value(HSV hsv) {
  RGB rgb = hsv_to_rgb( hsv );
  float f = (float)rgb_matrix_config.hsv.v / UINT8_MAX;
  return (RGB){ f * rgb.r, f * rgb.g, f * rgb.b };
}

void keyboard_post_init_user(void) {
  rgb_matrix_enable();
}

const uint8_t PROGMEM ledmap[][RGB_MATRIX_LED_COUNT][3] = {
    [1] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {140,34,255}, {33,255,255}, {33,255,255}, {140,34,255}, {140,34,255}, {140,34,255}, {33,255,255}, {33,255,255}, {140,34,255}, {140,34,255}, {140,34,255}, {140,34,255}, {139,227,210}, {139,227,210}, {140,34,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {140,34,255}, {99,243,224}, {211,255,255}, {211,255,255}, {140,34,255}, {140,34,255}, {211,255,255}, {99,243,224}, {99,243,224}, {211,255,255}, {140,34,255}, {33,255,255}, {33,255,255}, {99,243,224}, {140,34,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

};

void set_layer_color(int layer) {
  for (int i = 0; i < RGB_MATRIX_LED_COUNT; i++) {
    HSV hsv = {
      .h = pgm_read_byte(&ledmap[layer][i][0]),
      .s = pgm_read_byte(&ledmap[layer][i][1]),
      .v = pgm_read_byte(&ledmap[layer][i][2]),
    };
    if (!hsv.h && !hsv.s && !hsv.v) {
        rgb_matrix_set_color( i, 0, 0, 0 );
    } else {
        RGB rgb = hsv_to_rgb_with_value(hsv);
        rgb_matrix_set_color(i, rgb.r, rgb.g, rgb.b);
    }
  }
}

bool rgb_matrix_indicators_user(void) {
  if (rawhid_state.rgb_control) {
      return false;
  }
  if (!keyboard_config.disable_layer_led) { 
    switch (biton32(layer_state)) {
      case 1:
        set_layer_color(1);
        break;
     default:
        if (rgb_matrix_get_flags() == LED_FLAG_NONE) {
          rgb_matrix_set_color_all(0, 0, 0);
        }
    }
  } else {
    if (rgb_matrix_get_flags() == LED_FLAG_NONE) {
      rgb_matrix_set_color_all(0, 0, 0);
    }
  }

  return true;
}


typedef struct {
    bool is_press_action;
    uint8_t step;
} tap;

enum {
    SINGLE_TAP = 1,
    SINGLE_HOLD,
    DOUBLE_TAP,
    DOUBLE_HOLD,
    DOUBLE_SINGLE_TAP,
    MORE_TAPS
};

static tap dance_state[3];

uint8_t dance_step(tap_dance_state_t *state);

uint8_t dance_step(tap_dance_state_t *state) {
    if (state->count == 1) {
        if (state->interrupted || !state->pressed) return SINGLE_TAP;
        else return SINGLE_HOLD;
    } else if (state->count == 2) {
        if (state->interrupted) return DOUBLE_SINGLE_TAP;
        else if (state->pressed) return DOUBLE_HOLD;
        else return DOUBLE_TAP;
    }
    return MORE_TAPS;
}


void on_dance_0(tap_dance_state_t *state, void *user_data);
void dance_0_finished(tap_dance_state_t *state, void *user_data);
void dance_0_reset(tap_dance_state_t *state, void *user_data);

void on_dance_0(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(LGUI(LSFT(KC_P)));
        tap_code16(LGUI(LSFT(KC_P)));
        tap_code16(LGUI(LSFT(KC_P)));
    }
    if(state->count > 3) {
        tap_code16(LGUI(LSFT(KC_P)));
    }
}

void dance_0_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[0].step = dance_step(state);
    switch (dance_state[0].step) {
        case SINGLE_TAP: register_code16(LGUI(LSFT(KC_P))); break;
        case DOUBLE_TAP: register_code16(RGUI(KC_W)); break;
        case DOUBLE_HOLD: register_code16(LGUI(KC_Q)); break;
        case DOUBLE_SINGLE_TAP: tap_code16(LGUI(LSFT(KC_P))); register_code16(LGUI(LSFT(KC_P)));
    }
}

void dance_0_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[0].step) {
        case SINGLE_TAP: unregister_code16(LGUI(LSFT(KC_P))); break;
        case DOUBLE_TAP: unregister_code16(RGUI(KC_W)); break;
        case DOUBLE_HOLD: unregister_code16(LGUI(KC_Q)); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(LGUI(LSFT(KC_P))); break;
    }
    dance_state[0].step = 0;
}
void on_dance_1(tap_dance_state_t *state, void *user_data);
void dance_1_finished(tap_dance_state_t *state, void *user_data);
void dance_1_reset(tap_dance_state_t *state, void *user_data);

void on_dance_1(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_V);
        tap_code16(KC_V);
        tap_code16(KC_V);
    }
    if(state->count > 3) {
        tap_code16(KC_V);
    }
}

void dance_1_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[1].step = dance_step(state);
    switch (dance_state[1].step) {
        case SINGLE_TAP: register_code16(KC_V); break;
        case SINGLE_HOLD: register_code16(LGUI(KC_V)); break;
        case DOUBLE_TAP: register_code16(LGUI(LSFT(KC_V))); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_V); register_code16(KC_V);
    }
}

void dance_1_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[1].step) {
        case SINGLE_TAP: unregister_code16(KC_V); break;
        case SINGLE_HOLD: unregister_code16(LGUI(KC_V)); break;
        case DOUBLE_TAP: unregister_code16(LGUI(LSFT(KC_V))); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_V); break;
    }
    dance_state[1].step = 0;
}
void on_dance_2(tap_dance_state_t *state, void *user_data);
void dance_2_finished(tap_dance_state_t *state, void *user_data);
void dance_2_reset(tap_dance_state_t *state, void *user_data);

void on_dance_2(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(RGUI(RSFT(KC_A)));
        tap_code16(RGUI(RSFT(KC_A)));
        tap_code16(RGUI(RSFT(KC_A)));
    }
    if(state->count > 3) {
        tap_code16(RGUI(RSFT(KC_A)));
    }
}

void dance_2_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[2].step = dance_step(state);
    switch (dance_state[2].step) {
        case SINGLE_TAP: register_code16(RGUI(RSFT(KC_A))); break;
        case SINGLE_HOLD: register_code16(RGUI(KC_DOT)); break;
        case DOUBLE_TAP: register_code16(RGUI(RSFT(KC_T))); break;
        case DOUBLE_HOLD: register_code16(RGUI(RSFT(KC_Y))); break;
        case DOUBLE_SINGLE_TAP: tap_code16(RGUI(RSFT(KC_A))); register_code16(RGUI(RSFT(KC_A)));
    }
}

void dance_2_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[2].step) {
        case SINGLE_TAP: unregister_code16(RGUI(RSFT(KC_A))); break;
        case SINGLE_HOLD: unregister_code16(RGUI(KC_DOT)); break;
        case DOUBLE_TAP: unregister_code16(RGUI(RSFT(KC_T))); break;
        case DOUBLE_HOLD: unregister_code16(RGUI(RSFT(KC_Y))); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(RGUI(RSFT(KC_A))); break;
    }
    dance_state[2].step = 0;
}

tap_dance_action_t tap_dance_actions[] = {
        [DANCE_0] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_0, dance_0_finished, dance_0_reset),
        [DANCE_1] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_1, dance_1_finished, dance_1_reset),
        [DANCE_2] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_2, dance_2_finished, dance_2_reset),
};

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {

    case DUAL_FUNC_0:
      if (record->tap.count > 0) {
        if (record->event.pressed) {
          register_code16(KC_X);
        } else {
          unregister_code16(KC_X);
        }
      } else {
        if (record->event.pressed) {
          register_code16(LGUI(KC_X));
        } else {
          unregister_code16(LGUI(KC_X));
        }  
      }  
      return false;
    case DUAL_FUNC_1:
      if (record->tap.count > 0) {
        if (record->event.pressed) {
          register_code16(KC_C);
        } else {
          unregister_code16(KC_C);
        }
      } else {
        if (record->event.pressed) {
          register_code16(LGUI(KC_C));
        } else {
          unregister_code16(LGUI(KC_C));
        }  
      }  
      return false;
    case DUAL_FUNC_2:
      if (record->tap.count > 0) {
        if (record->event.pressed) {
          register_code16(KC_LBRC);
        } else {
          unregister_code16(KC_LBRC);
        }
      } else {
        if (record->event.pressed) {
          register_code16(KC_LCBR);
        } else {
          unregister_code16(KC_LCBR);
        }  
      }  
      return false;
    case DUAL_FUNC_3:
      if (record->tap.count > 0) {
        if (record->event.pressed) {
          register_code16(KC_RBRC);
        } else {
          unregister_code16(KC_RBRC);
        }
      } else {
        if (record->event.pressed) {
          register_code16(KC_RCBR);
        } else {
          unregister_code16(KC_RCBR);
        }  
      }  
      return false;
    case RGB_SLD:
      if (record->event.pressed) {
        rgblight_mode(1);
      }
      return false;
  }
  return true;
}

uint8_t layer_state_set_user(uint8_t state) {
    uint8_t layer = biton(state);
  ergodox_board_led_off();
  ergodox_right_led_1_off();
  ergodox_right_led_2_off();
  ergodox_right_led_3_off();
  switch (layer) {
    case 1:
      ergodox_right_led_1_on();
      break;
    case 2:
      ergodox_right_led_2_on();
      break;
    case 3:
      ergodox_right_led_3_on();
      break;
    case 4:
      ergodox_right_led_1_on();
      ergodox_right_led_2_on();
      break;
    case 5:
      ergodox_right_led_1_on();
      ergodox_right_led_3_on();
      break;
    case 6:
      ergodox_right_led_2_on();
      ergodox_right_led_3_on();
      break;
    case 7:
      ergodox_right_led_1_on();
      ergodox_right_led_2_on();
      ergodox_right_led_3_on();
      break;
    default:
      break;
  }
  return state;
};

