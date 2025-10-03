/*
 * gnss.c
 *
 *  Created on: Oct 3, 2025
 *      Author: stoica
 */

#include <string.h>
#include <math.h>
#include "gnss.h"
#include "circular_filter.h"
#include "config.h"
#include "stm32f1xx_hal.h"
#include "usart.h"

// UBX NAV-PVT
#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62
#define UBX_CLASS_NAV 0x01
#define UBX_ID_NAV_PVT 0x07
#define NAV_PVT_LEN 92

// ===== Buffer UART =====
static uint8_t gps_rx_byte;

// ===== Parser state machine =====
typedef struct __attribute__((packed)){
  uint32_t iTOW; int16_t year; uint8_t month, day;
  uint8_t hour, min, sec, valid;
  uint32_t tAcc; int32_t nano;
  uint8_t fixType, flags, flags2, numSV;
  int32_t lon, lat, height, hMSL;
  uint32_t hAcc, vAcc;
  int32_t velN, velE, velD;
  int32_t gSpeed, headMot; // heading of motion (deg*1e-5)
  uint32_t sAcc, headAcc;
  uint16_t pDOP, flags3;
  uint8_t rsv1[4];
  int32_t headVeh; int16_t magDec; uint16_t magAcc;
} UBX_NAV_PVT;

static uint8_t ck_a, ck_b;
static void ck_init(){ ck_a=0; ck_b=0; }
static void ck_add(uint8_t b){ ck_a += b; ck_b += ck_a; }

typedef enum { S_SYNC1, S_SYNC2, S_CLASS, S_ID, S_LEN1, S_LEN2,
               S_PAYLOAD, S_CK_A, S_CK_B } ubx_state_t;
static ubx_state_t st = S_SYNC1;
static uint8_t cls,id;
static uint16_t len, idx;
static uint8_t payload[128];

static int ubx_parse_byte(uint8_t b, UBX_NAV_PVT* out){
  switch(st){
    case S_SYNC1: if(b==UBX_SYNC1) st=S_SYNC2; break;
    case S_SYNC2: if(b==UBX_SYNC2) st=S_CLASS; else st=S_SYNC1; break;
    case S_CLASS: cls=b; st=S_ID; ck_init(); ck_add(b); break;
    case S_ID:    id=b;  st=S_LEN1; ck_add(b); break;
    case S_LEN1:  len=b; st=S_LEN2; ck_add(b); break;
    case S_LEN2:  len|=(uint16_t)(b<<8); ck_add(b);
                  if(len>sizeof(payload)){ st=S_SYNC1; break; }
                  idx=0; st=S_PAYLOAD; break;
    case S_PAYLOAD:
      payload[idx++]=b; ck_add(b);
      if(idx>=len) st=S_CK_A;
      break;
    case S_CK_A: if (b==ck_a) st=S_CK_B; else st=S_SYNC1; break;
    case S_CK_B:
      if (b==ck_b && cls==UBX_CLASS_NAV && id==UBX_ID_NAV_PVT
          && len>=sizeof(UBX_NAV_PVT)){
        memcpy(out, payload, sizeof(UBX_NAV_PVT));
        st=S_SYNC1;
        return 1; // success
      }
      st=S_SYNC1; break;
  }
  return 0;
}

// ===== COG filter & lock =====
static CircularEMA cog_ema;
static float lastReliableHeading=0.0f;

// ===== Ultima soluție GPS =====
static GNSSFix latest_fix;
static volatile int fix_ready = 0;

// ===== API =====
void gnss_init(uint32_t baud){
    (void)baud; // deja setat în MX_USART3_UART_Init (9600/115200 etc)
    circEMA_init(&cog_ema, 0.2f, 0.0f);
    HAL_UART_Receive_IT(&huart3, &gps_rx_byte, 1);
}

bool gnss_good(const GNSSFix* f){
    return (f->fixType >= 2) && (f->hdop > 0 && f->hdop <= 3.0f);
}

void coglock_reset(float hdg){ lastReliableHeading=hdg; }

float coglock_update(float cog_deg, float sog_mps, bool cog_stable,
                     float* heading_used_deg, float* last_rel_deg){
  float cog_f = circEMA_update(&cog_ema, cog_deg);
  if (sog_mps < SOG_LOCK_MIN_MPS || !cog_stable)
      *heading_used_deg = lastReliableHeading;
  else {
      *heading_used_deg = cog_f;
      lastReliableHeading = cog_f;
  }
  if (last_rel_deg) *last_rel_deg = lastReliableHeading;
  return cog_f;
}

bool gnss_poll(GNSSFix* out){
    if (fix_ready){
        *out = latest_fix;
        fix_ready = 0;
        return true;
    }
    return false;
}

// ===== HAL UART callback =====
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if (huart->Instance == USART3){
        static UBX_NAV_PVT pvt;
        if (ubx_parse_byte(gps_rx_byte, &pvt)){
            GNSSFix fix;
            fix.lat = pvt.lat * 1e-7;
            fix.lon = pvt.lon * 1e-7;
            fix.sog_mps = pvt.gSpeed * 1e-3f; // mm/s → m/s
            float head = pvt.headMot * 1e-5f;
            if (head < 0) head += 360.0f;
            fix.cog_deg = head;
            fix.fixType = pvt.fixType;
            fix.hdop = pvt.pDOP * 0.01f;
            fix.last_ms = HAL_GetTick();
            latest_fix = fix;
            fix_ready = 1;
        }
        // repornește recepția pentru următorul byte
        HAL_UART_Receive_IT(&huart3, &gps_rx_byte, 1);
    }
}
