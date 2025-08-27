#include <Arduino.h>
#include <string.h>

// ======================= 사용자 설정 =======================
#define BAUD            115200

// 하드웨어 시리얼 (보드에 맞게 수정)
HardwareSerial &UART_GPS = Serial2; // 메인 GPS 입력 (UBX only)
HardwareSerial &UART_FC  = Serial1; // FC로 출력
HardwareSerial &UART_RTK = Serial;  // F9P 입력 + 디버그(USB)

// 핀맵(필요 시 수정)
#define PIN_GPS_RX 16
#define PIN_GPS_TX -1          // 사용 안함
#define PIN_FC_RX  -1          // 사용 안함
#define PIN_FC_TX   2          // ESP32 -> FC
#define PIN_RTK_RX  3          // F9P -> ESP32(USB-Serial 경유)

// 정책
#define RTK_STALE_MS  500      // RTK NAV-PVT가 이 시간(ms) 이내면 "신선"
#define RTK_LOCK_MS   700      // 한 번 RTK로 쓰기 시작하면 이 시간 동안은 RTK 유지

// ======================== UBX 상수 =========================
static const uint8_t  UBX_SYNC1      = 0xB5;
static const uint8_t  UBX_SYNC2      = 0x62;
static const uint8_t  UBX_CLASS_NAV  = 0x01;
static const uint8_t  UBX_ID_NAV_PVT = 0x07;

static const uint16_t UBX_MAX        = 300; // payload 여유

// ======================= 공용 구조체 =======================
struct UbxPacket {
  uint16_t len;               // payload length
  uint16_t total;             // 전체 바이트 수(sync~ck)
  uint8_t  buf[UBX_MAX + 8];  // 전체 패킷
  uint32_t stamp_ms;          // 수신 시각
  bool     valid;
};

static UbxPacket g_rtk_navpvt  = {};  // F9P 최신 NAV-PVT
static UbxPacket g_main_navpvt = {};  // 메인 GPS 최신 NAV-PVT(폴백용)

enum UbxState { S_SYNC1, S_SYNC2, S_CLASS, S_ID, S_LEN1, S_LEN2, S_PAYLOAD, S_CK_A, S_CK_B };

struct UbxParser {
  UbxState st = S_SYNC1;
  uint8_t  cls=0, id=0;
  uint16_t len=0, idx=0;
  uint8_t  ckA=0, ckB=0;
  uint8_t  packet[UBX_MAX + 8]; // sync~ck까지 버퍼
  uint16_t total=0;
  bool     collecting=false;    // NAV-PVT면 true (저장)
};

static UbxParser parser_rtk;
static UbxParser parser_main;

static inline void ck_acc(uint8_t b, uint8_t &A, uint8_t &B){ A = A + b; B = B + A; }
static inline void parserReset(UbxParser &p){
  p.st=S_SYNC1; p.cls=0; p.id=0; p.len=0; p.idx=0; p.ckA=0; p.ckB=0; p.total=0; p.collecting=false;
}

// 히스테리시스 잠금 타이머
static uint32_t g_rtk_lock_until = 0;

// =================== F9P 수신 → 최신 NAV-PVT 저장 ===================
void feedRTK(uint8_t b){
  UbxParser &p = parser_rtk;
  switch(p.st){
    case S_SYNC1:
      if(b==UBX_SYNC1){ p.st=S_SYNC2; p.total=0; p.packet[p.total++]=b; }
      break;

    case S_SYNC2:
      if(b==UBX_SYNC2){ p.st=S_CLASS; p.packet[p.total++]=b; p.ckA=0; p.ckB=0; }
      else parserReset(p);
      break;

    case S_CLASS:
      p.cls=b; p.packet[p.total++]=b; ck_acc(b,p.ckA,p.ckB); p.st=S_ID; break;

    case S_ID:
      p.id=b; p.packet[p.total++]=b; ck_acc(b,p.ckA,p.ckB); p.st=S_LEN1; break;

    case S_LEN1:
      p.len=b; p.packet[p.total++]=b; ck_acc(b,p.ckA,p.ckB); p.st=S_LEN2; break;

    case S_LEN2:
      p.len |= (uint16_t)b<<8; p.packet[p.total++]=b; ck_acc(b,p.ckA,p.ckB);
      if(p.len>UBX_MAX){ parserReset(p); break; }
      p.idx=0; p.st=S_PAYLOAD;
      p.collecting = (p.cls==UBX_CLASS_NAV && p.id==UBX_ID_NAV_PVT);
      break;

    case S_PAYLOAD:
      if(p.collecting) p.packet[p.total++] = b;
      ck_acc(b,p.ckA,p.ckB);
      if(++p.idx >= p.len) p.st=S_CK_A;
      break;

    case S_CK_A:
      if(p.collecting) p.packet[p.total++] = b;
      if(b==p.ckA) p.st=S_CK_B; else parserReset(p);
      break;

    case S_CK_B:
      if(p.collecting){
        p.packet[p.total++] = b;
        g_rtk_navpvt.len      = p.len;
        g_rtk_navpvt.total    = p.total;
        memcpy(g_rtk_navpvt.buf, p.packet, p.total);
        g_rtk_navpvt.stamp_ms = millis();
        g_rtk_navpvt.valid    = true;
      }
      parserReset(p);
      break;
  }
}

// ===== 메인 GPS 수신 → NAV-PVT만 치환, 나머지는 즉시 패스 =====
void feedMainAndForward(uint8_t b){
  UbxParser &p = parser_main;

  switch(p.st){
    case S_SYNC1:
      if(b==UBX_SYNC1){
        p.st=S_SYNC2; p.total=0; p.packet[p.total++]=b;
      } else {
        // UBX 스트림이면 거의 안 나옴. 안전하게 그대로 패스.
        UART_FC.write(b);
      }
      break;

    case S_SYNC2:
      if(b==UBX_SYNC2){
        p.st=S_CLASS; p.packet[p.total++]=b; p.ckA=0; p.ckB=0;
      } else {
        // 0xB5 다음이 0x62가 아니면 0xB5와 현재 바이트 모두 흐르게 하거나 드롭 선택.
        UART_FC.write(p.packet[0]); // 이전 0xB5도 같이 흘림(필요 없으면 제거)
        UART_FC.write(b);
        parserReset(p);
      }
      break;

    case S_CLASS:
      p.cls=b; p.packet[p.total++]=b; ck_acc(b,p.ckA,p.ckB); p.st=S_ID; break;

    case S_ID:
      p.id=b; p.packet[p.total++]=b; ck_acc(b,p.ckA,p.ckB); p.st=S_LEN1; break;

    case S_LEN1:
      p.len=b; p.packet[p.total++]=b; ck_acc(b,p.ckA,p.ckB); p.st=S_LEN2; break;

    case S_LEN2:
      p.len |= (uint16_t)b<<8; p.packet[p.total++]=b; ck_acc(b,p.ckA,p.ckB);
      if(p.len>UBX_MAX){
        // 너무 길면 지금까지(헤더) 내보내고 리셋하거나 드롭. 여기선 내보냄.
        for(uint16_t i=0;i<p.total;i++) UART_FC.write(p.packet[i]);
        parserReset(p);
        break;
      }
      p.idx=0; p.st=S_PAYLOAD;
      p.collecting = (p.cls==UBX_CLASS_NAV && p.id==UBX_ID_NAV_PVT);
      break;

    case S_PAYLOAD:
      if(p.collecting) {
        // NAV-PVT는 일단 로컬 버퍼에 모아(차단)
        p.packet[p.total++] = b;
      } else {
        // NAV-PVT 이외는 payload 즉시 패스
        UART_FC.write(b);
      }
      ck_acc(b,p.ckA,p.ckB);
      if(++p.idx >= p.len) p.st=S_CK_A;
      break;

    case S_CK_A:
      if(p.collecting) p.packet[p.total++] = b;
      if(b==p.ckA) p.st=S_CK_B;
      else {
        // 체크섬 실패: 복구. (NAV-PVT 이외는 이미 payload 흘렸으므로 여기선 드롭)
        parserReset(p);
      }
      break;

    case S_CK_B:
      if(p.collecting) {
        // 완전한 메인 NAV-PVT 확보
        p.packet[p.total++] = b;

        g_main_navpvt.len      = p.len;
        g_main_navpvt.total    = p.total;
        memcpy(g_main_navpvt.buf, p.packet, p.total);
        g_main_navpvt.stamp_ms = millis();
        g_main_navpvt.valid    = true;

        // ---------- 치환/락 로직 ----------
        bool sent = false;
        uint32_t now = millis();

        // RTK가 신선하면 락 연장
        bool rtk_fresh = g_rtk_navpvt.valid && (now - g_rtk_navpvt.stamp_ms) <= RTK_STALE_MS;
        if (rtk_fresh) {
          g_rtk_lock_until = now + RTK_LOCK_MS;
        }

        // 신선하거나, 락이 살아있으면 RTK로 치환
        if (g_rtk_navpvt.valid && (rtk_fresh || now < g_rtk_lock_until)) {
          UART_FC.write(g_rtk_navpvt.buf, g_rtk_navpvt.total);
          sent = true;
        }

        if (!sent) {
          // 폴백: 메인 NAV-PVT 원본 전송
          UART_FC.write(g_main_navpvt.buf, g_main_navpvt.total);
        }
      } else {
        // NAV-PVT 이외: 마지막 CK_B까지 패스
        UART_FC.write(b);
      }
      parserReset(p);
      break;
  }
}

// ========================= 디버그 ==========================
void debugTick(){
  static uint32_t t_prev=0;
  uint32_t now = millis();
  if(now - t_prev >= 1000){
    t_prev = now;
    bool rtk_ok = g_rtk_navpvt.valid && (now - g_rtk_navpvt.stamp_ms) <= RTK_STALE_MS;
    UART_RTK.printf("[DBG] RTK_NAVPVT: valid=%d age=%lums, MAIN_NAVPVT: valid=%d age=%lums, LOCK=%lums\n",
      (int)rtk_ok,
      (unsigned long)(g_rtk_navpvt.valid ? (now - g_rtk_navpvt.stamp_ms) : 0),
      (int)g_main_navpvt.valid,
      (unsigned long)(g_main_navpvt.valid ? (now - g_main_navpvt.stamp_ms) : 0),
      (unsigned long)(now < g_rtk_lock_until ? (g_rtk_lock_until - now) : 0));
  }
}

// ======================== Arduino =========================
void setup(){
  UART_RTK.begin(BAUD);                             // USB-Serial 콘솔 + F9P in
  UART_FC.begin(BAUD, SERIAL_8N1, PIN_FC_RX, PIN_FC_TX);
  UART_GPS.begin(BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

  parserReset(parser_rtk);
  parserReset(parser_main);

  UART_RTK.printf("UBX bridge started @%lu bps\n", (unsigned long)BAUD);
  UART_RTK.printf("Policy: pass-through all UBX, replace NAV-PVT with F9P (STALE>%dms -> fallback, LOCK=%dms)\n",
                  RTK_STALE_MS, RTK_LOCK_MS);
}

void loop(){
  // F9P (RTK) 수신 처리: 최신 NAV-PVT 저장
  while(UART_RTK.available()){
    feedRTK((uint8_t)UART_RTK.read());
  }

  // 메인 GPS → FC: NAV-PVT만 치환, 나머지는 패스
  while(UART_GPS.available()){
    feedMainAndForward((uint8_t)UART_GPS.read());
  }

  debugTick();
}