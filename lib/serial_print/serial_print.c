#include "serial_print.h"

#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

bool serial_print_enqueue(QueueHandle_t print_queue, const log_message_t *msg,
                          TickType_t timeout_ticks) {
  if (print_queue == NULL || msg == NULL) {
    return false;
  }
  return xQueueSend(print_queue, msg, timeout_ticks) == pdTRUE;
}

bool serial_print_event(QueueHandle_t print_queue, const char *text,
                        TickType_t timeout_ticks) {
  if (print_queue == NULL || text == NULL) {
    return false;
  }
  log_message_t msg = {.kind = LOG_EVENT};
  snprintf(msg.data.text, sizeof(msg.data.text), "%s", text);
  return xQueueSend(print_queue, &msg, timeout_ticks) == pdTRUE;
}

bool serial_print_line(QueueHandle_t print_queue, const char *line,
                       TickType_t timeout_ticks) {
  if (print_queue == NULL || line == NULL) {
    return false;
  }
  log_message_t msg = {.kind = LOG_RAW_LINE};
  snprintf(msg.data.text, sizeof(msg.data.text), "%s", line);
  return xQueueSend(print_queue, &msg, timeout_ticks) == pdTRUE;
}

bool serial_print_fmt(QueueHandle_t print_queue, TickType_t timeout_ticks,
                      const char *fmt, ...) {
  if (print_queue == NULL || fmt == NULL) {
    return false;
  }
  log_message_t msg = {.kind = LOG_RAW_LINE};
  va_list args;
  va_start(args, fmt);
  int written = vsnprintf(msg.data.text, sizeof(msg.data.text), fmt, args);
  va_end(args);
  if (written < 0) {
    return false;
  }
  if ((size_t)written >= sizeof(msg.data.text) && sizeof(msg.data.text) >= 3U) {
    msg.data.text[sizeof(msg.data.text) - 3U] = '\r';
    msg.data.text[sizeof(msg.data.text) - 2U] = '\n';
    msg.data.text[sizeof(msg.data.text) - 1U] = '\0';
  }
  return xQueueSend(print_queue, &msg, timeout_ticks) == pdTRUE;
}

bool serial_print_help(QueueHandle_t print_queue, TickType_t timeout_ticks) {
  if (print_queue == NULL) {
    return false;
  }
  log_message_t msg = {.kind = LOG_HELP};
  return xQueueSend(print_queue, &msg, timeout_ticks) == pdTRUE;
}

bool serial_print_pos_out(QueueHandle_t print_queue, float output, float error,
                          bool sp_reached) {
  if (print_queue == NULL) {
    return false;
  }
  log_message_t msg = {
      .kind = LOG_POS_OUT,
      .data.pos_out =
          {
              .output = output,
              .error = error,
              .sp_reached = sp_reached,
          },
  };
  return xQueueSend(print_queue, &msg, 0) == pdTRUE;
}

bool serial_print_pulse_diag(QueueHandle_t print_queue, bool is_ton_to_motion,
                             uint32_t elapsed_ms, int32_t counts,
                             TickType_t timeout_ticks) {
  if (print_queue == NULL) {
    return false;
  }
  log_message_t msg = {
      .kind = LOG_PULSE_DIAG,
      .data.pulse_diag =
          {
              .is_ton_to_motion = is_ton_to_motion,
              .elapsed_ms = elapsed_ms,
              .counts = counts,
          },
  };
  return xQueueSend(print_queue, &msg, timeout_ticks) == pdTRUE;
}

void serial_print_write_message(const log_message_t *message) {
  if (message == NULL) {
    return;
  }

  switch (message->kind) {
  case LOG_EVENT:
    printf("EVT %s\r\n", message->data.text);
    break;

  case LOG_RAW_LINE:
    printf("%s", message->data.text);
    break;

  case LOG_HELP:
    printf(
        "CMD HELP|MODE SPEED/POS/OPEN/CLOCK|CLOCK|SPEED F/R/D <rpm>|POS <cnt>|"
        "ANG <deg>|DUTY F/R/D <0..255>|PULSE F/R <n_cycles> <duty> [freq_hz]|"
        "MINMOVE <0..255>|CAL START/READ/STOP|PID GET|PID SPEED KP/KI/KD <val>|"
        "PID POS KP/KI/KD <val>|PID CLOCK KP/KI/KD <val>|PID DEFAULT|"
        "POT ON/OFF/GET|EC11 GET/ON/OFF/ZERO|SAVE|LOAD|NVS SAVE/LOAD/RESET/GET/"
        "AUTOSAVE|LCD STATUS/BRINGUP/PATTERN/DASH/PID/ALLON/CURSOR/BLINK/"
        "DISPLAY/CLEAR|STOP|STATUS|LOG ON/OFF|LOG POS ON/OFF|LOG POSOUT ON/OFF|"
        "LOG RATE <ms>\r\n");
    break;

  case LOG_POSITION:
    printf("P,%" PRId64 ",%.2f\n", message->data.position.counts,
           (double)message->data.position.angle_deg);
    break;

  case LOG_TELEMETRY:
    if (message->data.telemetry.state == FSM_OPEN_LOOP) {
      printf("TEL O D=%u\r\n", message->data.telemetry.duty);
    } else if (message->data.telemetry.state == FSM_CLOCK ||
               message->data.telemetry.clock_mode) {
      printf("TEL C R=%.2f A=%.2f D=%u\r\n",
             (double)message->data.telemetry.reference,
             (double)message->data.telemetry.measured_angle,
             message->data.telemetry.duty);
    } else if (message->data.telemetry.position_mode) {
      printf("TEL P R=%.2f A=%.2f D=%u\r\n",
             (double)message->data.telemetry.reference,
             (double)message->data.telemetry.measured_angle,
             message->data.telemetry.duty);
    } else {
      printf("TEL V R=%.1f M=%.1f D=%u\r\n",
             (double)message->data.telemetry.reference,
             (double)message->data.telemetry.measured_rpm,
             message->data.telemetry.duty);
    }
    break;

  case LOG_POS_OUT:
    if (message->data.pos_out.sp_reached) {
      printf("POS OUT=%.2f ERR=%.2f (SP REACHED)\r\n",
             (double)message->data.pos_out.output,
             (double)message->data.pos_out.error);
    } else {
      printf("POS OUT=%.2f ERR=%.2f\r\n",
             (double)message->data.pos_out.output,
             (double)message->data.pos_out.error);
    }
    break;

  case LOG_PULSE_DIAG:
    if (message->data.pulse_diag.is_ton_to_motion) {
      printf("DBG TON_TO_MOTION %" PRIu32 "ms\r\n",
             message->data.pulse_diag.elapsed_ms);
    } else {
      printf("DBG PULSE_COUNTS %" PRId32 "\r\n",
             message->data.pulse_diag.counts);
    }
    break;
  }
}
