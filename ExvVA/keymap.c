#include QMK_KEYBOARD_H
#include "version.h"

#ifndef ZSA_SAFE_RANGE
#define ZSA_SAFE_RANGE SAFE_RANGE
#endif

enum custom_keycodes {
  RGB_SLD = ZSA_SAFE_RANGE,
  ST_MACRO_0,
};


enum tap_dance_codes {
  DANCE_0,
  DANCE_1,
  DANCE_2,
};

#define DUAL_FUNC_0 LT(5, KC_F18)
#define DUAL_FUNC_1 LT(3, KC_H)
#define DUAL_FUNC_2 LT(12, KC_R)
#define DUAL_FUNC_3 LT(8, KC_F22)
#define DUAL_FUNC_4 LT(15, KC_F10)
#define DUAL_FUNC_5 LT(13, KC_F17)
#define DUAL_FUNC_6 LT(2, KC_F12)
#define DUAL_FUNC_7 LT(7, KC_F5)
#define DUAL_FUNC_8 LT(5, KC_P)
#define DUAL_FUNC_9 LT(8, KC_P)
#define DUAL_FUNC_10 LT(10, KC_F19)
#define DUAL_FUNC_11 LT(1, KC_U)

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [0] = LAYOUT_ergodox_pretty(
    KC_ESCAPE,      DUAL_FUNC_0,    DUAL_FUNC_1,    DUAL_FUNC_2,    DUAL_FUNC_3,    DUAL_FUNC_4,    TG(1),                                          TG(2),          DUAL_FUNC_6,    DUAL_FUNC_7,    DUAL_FUNC_8,    DUAL_FUNC_9,    KC_0,           KC_MINUS,
    TD(DANCE_0),    KC_Q,           KC_W,           KC_E,           KC_R,           KC_T,           ST_MACRO_0,                                     LGUI(LCTL(LSFT(KC_4))),KC_Y,           KC_U,           KC_I,           KC_O,           KC_P,           KC_BSLS,
    KC_LEFT_CTRL,   KC_A,           KC_S,           KC_D,           KC_F,           KC_G,                                                                           KC_H,           KC_J,           KC_K,           KC_L,           KC_SCLN,        KC_QUOTE,
    SC_LSPO,        KC_Z,           KC_X,           DUAL_FUNC_5,    TD(DANCE_1),    KC_B,           LSFT(KC_GRAVE),                                 TD(DANCE_2),    KC_N,           KC_M,           KC_COMMA,       KC_DOT,         KC_SLASH,       SC_RSPC,
    KC_GRAVE,       KC_MEDIA_PLAY_PAUSE,KC_LEFT_ALT,    KC_LEFT,        KC_RIGHT,                                                                                                       KC_UP,          KC_DOWN,        DUAL_FUNC_10,   DUAL_FUNC_11,   LGUI(LCTL(KC_SPACE)),
                                                                                                    LT(1, KC_AUDIO_VOL_DOWN),LGUI(LSFT(KC_LBRC)),LGUI(LSFT(KC_RBRC)),LT(2, KC_AUDIO_VOL_UP),
                                                                                                                    MT(MOD_LCTL, KC_DELETE),MT(MOD_LGUI | MOD_LSFT, KC_PAGE_UP),
                                                                                    MT(MOD_LGUI, KC_TAB),KC_BSPC,        MT(MOD_LGUI, KC_ENTER),MT(MOD_LGUI | MOD_LSFT, KC_EQUAL),KC_ENTER,       KC_SPACE
  ),
  [1] = LAYOUT_ergodox_pretty(
    KC_ESCAPE,      KC_F1,          KC_F2,          KC_F3,          KC_F4,          KC_F5,          KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_F6,          KC_F7,          KC_F8,          KC_F9,          KC_KP_MINUS,    KC_F10,
    KC_TRANSPARENT, KC_EXLM,        KC_AT,          KC_LCBR,        KC_RCBR,        KC_PIPE,        KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_UP,          KC_7,           KC_8,           KC_9,           KC_ASTR,        KC_F11,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_DLR,         KC_LPRN,        KC_RPRN,        KC_GRAVE,                                                                       KC_DOWN,        KC_4,           KC_5,           KC_6,           KC_PLUS,        KC_F12,
    KC_TRANSPARENT, KC_PERC,        KC_CIRC,        KC_LBRC,        KC_RBRC,        KC_TILD,        KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_AMPR,        KC_1,           KC_2,           KC_3,           KC_BSLS,        KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                                                                                 KC_COMMA,       KC_KP_0,        KC_KP_DOT,      KC_EQUAL,       KC_TRANSPARENT,
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
};








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
        case DOUBLE_HOLD: register_code16(LSFT(KC_V)); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_V); register_code16(KC_V);
    }
}

void dance_1_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[1].step) {
        case SINGLE_TAP: unregister_code16(KC_V); break;
        case SINGLE_HOLD: unregister_code16(LGUI(KC_V)); break;
        case DOUBLE_TAP: unregister_code16(LGUI(LSFT(KC_V))); break;
        case DOUBLE_HOLD: unregister_code16(LSFT(KC_V)); break;
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
    case ST_MACRO_0:
    if (record->event.pressed) {
      SEND_STRING(SS_LSFT(SS_TAP(X_5))SS_DELAY(100)  SS_LSFT(SS_TAP(X_DOT))SS_DELAY(100)  SS_LSFT(SS_TAP(X_5)));
    }
    break;

    case DUAL_FUNC_0:
      if (record->tap.count > 0) {
        if (record->event.pressed) {
          register_code16(KC_1);
        } else {
          unregister_code16(KC_1);
        }
      } else {
        if (record->event.pressed) {
          register_code16(LGUI(LSFT(KC_1)));
        } else {
          unregister_code16(LGUI(LSFT(KC_1)));
        }  
      }  
      return false;
    case DUAL_FUNC_1:
      if (record->tap.count > 0) {
        if (record->event.pressed) {
          register_code16(KC_2);
        } else {
          unregister_code16(KC_2);
        }
      } else {
        if (record->event.pressed) {
          register_code16(LGUI(LSFT(KC_2)));
        } else {
          unregister_code16(LGUI(LSFT(KC_2)));
        }  
      }  
      return false;
    case DUAL_FUNC_2:
      if (record->tap.count > 0) {
        if (record->event.pressed) {
          register_code16(KC_3);
        } else {
          unregister_code16(KC_3);
        }
      } else {
        if (record->event.pressed) {
          register_code16(LGUI(LSFT(KC_3)));
        } else {
          unregister_code16(LGUI(LSFT(KC_3)));
        }  
      }  
      return false;
    case DUAL_FUNC_3:
      if (record->tap.count > 0) {
        if (record->event.pressed) {
          register_code16(KC_4);
        } else {
          unregister_code16(KC_4);
        }
      } else {
        if (record->event.pressed) {
          register_code16(LGUI(LSFT(KC_4)));
        } else {
          unregister_code16(LGUI(LSFT(KC_4)));
        }  
      }  
      return false;
    case DUAL_FUNC_4:
      if (record->tap.count > 0) {
        if (record->event.pressed) {
          register_code16(KC_5);
        } else {
          unregister_code16(KC_5);
        }
      } else {
        if (record->event.pressed) {
          register_code16(LGUI(LSFT(KC_5)));
        } else {
          unregister_code16(LGUI(LSFT(KC_5)));
        }  
      }  
      return false;
    case DUAL_FUNC_5:
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
    case DUAL_FUNC_6:
      if (record->tap.count > 0) {
        if (record->event.pressed) {
          register_code16(KC_6);
        } else {
          unregister_code16(KC_6);
        }
      } else {
        if (record->event.pressed) {
          register_code16(RGUI(RSFT(KC_6)));
        } else {
          unregister_code16(RGUI(RSFT(KC_6)));
        }  
      }  
      return false;
    case DUAL_FUNC_7:
      if (record->tap.count > 0) {
        if (record->event.pressed) {
          register_code16(KC_7);
        } else {
          unregister_code16(KC_7);
        }
      } else {
        if (record->event.pressed) {
          register_code16(RGUI(RSFT(KC_7)));
        } else {
          unregister_code16(RGUI(RSFT(KC_7)));
        }  
      }  
      return false;
    case DUAL_FUNC_8:
      if (record->tap.count > 0) {
        if (record->event.pressed) {
          register_code16(KC_8);
        } else {
          unregister_code16(KC_8);
        }
      } else {
        if (record->event.pressed) {
          register_code16(RGUI(RSFT(KC_8)));
        } else {
          unregister_code16(RGUI(RSFT(KC_8)));
        }  
      }  
      return false;
    case DUAL_FUNC_9:
      if (record->tap.count > 0) {
        if (record->event.pressed) {
          register_code16(KC_9);
        } else {
          unregister_code16(KC_9);
        }
      } else {
        if (record->event.pressed) {
          register_code16(RGUI(RSFT(KC_9)));
        } else {
          unregister_code16(RGUI(RSFT(KC_9)));
        }  
      }  
      return false;
    case DUAL_FUNC_10:
      if (record->tap.count > 0) {
        if (record->event.pressed) {
          register_code16(KC_LBRC);
        } else {
          unregister_code16(KC_LBRC);
        }
      } else {
        if (record->event.pressed) {
          register_code16(LSFT(KC_LBRC));
        } else {
          unregister_code16(LSFT(KC_LBRC));
        }  
      }  
      return false;
    case DUAL_FUNC_11:
      if (record->tap.count > 0) {
        if (record->event.pressed) {
          register_code16(KC_RBRC);
        } else {
          unregister_code16(KC_RBRC);
        }
      } else {
        if (record->event.pressed) {
          register_code16(LSFT(KC_RBRC));
        } else {
          unregister_code16(LSFT(KC_RBRC));
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

