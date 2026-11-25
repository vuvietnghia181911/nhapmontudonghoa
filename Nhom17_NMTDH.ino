/* =====================================================================
   DAY CHUYEN LAP RAP XE MAY MINI - 3 TRAM + GIAM SAT QUA RS485 (MODBUS RTU)
   ESP32 DevKit V1 + Arduino core 3.x
   Servo: MG995, cap nguon rieng 6,0V tu LM2596, GND noi chung ESP32
   Thu vien: LiquidCrystal I2C (Frank de Brabander)
   Servo dieu khien bang PWM kenh rieng, khong dung thu vien ESP32Servo

   Tram 1: dung toi da 5 giay de quet ma QR
   Tram 2: xe roi cam bien tram 1 -> chay 2 giay -> dung 10 giay de lap
   Tram 3: sau tram 2 bang tai chay lien tuc. Cam bien tram 3 thay linh kien
           -> dung ngay 1 giay, dong thoi servo gat -> bang tai chay tiep.
           Xe di qua ma khong thay linh kien -> tinh la thieu, bang tai van chay.

   RS485: ESP32 la Slave ID 3, Modbus RTU 9600 8N1, may tinh la Master
     MAX485 RO      -> cau chia 10k/20k -> GPIO 16 (RX2)
     MAX485 DI      <- GPIO 17 (TX2)
     MAX485 DE + RE <- GPIO 33
     MAX485 VCC 5V, GND chung; A, B -> bo chuyen USB-RS485 cua may tinh

   Ban do thanh ghi Modbus (dia chi bat dau tu 0):
     Discrete input (FC02): 0 CB tram 1 | 1 CB linh kien T3 | 2 CB hop A | 3 CB hop B
                            4 nut Xong | 5 bang tai chay | 6 servo dang gat | 7 LED hop A
                            8 LED hop B | 9 LED xanh | 10 LED do | 11 coi | 12 dang tam dung
     Input register (FC04): 0 trang thai | 1 model | 2 so xe vao | 3 xe model 1 | 4 xe model 2
                            5 so OK | 6 so NG | 7 so loi quet | 8 nhip x10 (0,1 s)
                            9 so giay con lai (tram 1 va tram 2)
     Holding register (FC03/FC06): 0 toc do 0-255 | 1 thoi gian toi tram 2 (ms)
                            2 thoi gian cho toi da tram 3 (ms) | 3 ghi 1 hoac 2 = quet ma XEMAY-1/2
     Coil (FC01/FC05):      0 tam dung | 1 gat servo | 2 xoa bo dem | 3 reset line ve ban dau

   Phim tren Serial Monitor:
     + / -   tang / giam toc do bang tai
     q / a   tang / giam thoi gian chay toi tram 2
     w / s   tang / giam thoi gian cho toi da o tram 3
     g       gat thu servo
     i       xem thong so, trang thai cam bien va so khung Modbus
     r       xoa bo dem
     t       gui 5 dong thu qua RS485 de kiem tra duong phat
   ===================================================================== */

#include <WiFi.h>
#include <WebServer.h>

#define CO_LCD          1     // 0 neu chua dau LCD
#define CO_CAM_BIEN_HOP 1     // 0 neu chua gan cam bien o hop A va B
#define CHE_DO_THU      0     // 1 = khong can quet QR, mac dinh model 1 (de can chinh thoi gian)

// ---------------- CHAN ----------------
#define IR_TRAM1    4         // cam bien tram 1
#define IR_T3_TREN 35         // cam bien kiem tra linh kien o tram 3
#define IR_HOP_A   34         // cam bien hop A
#define IR_HOP_B   39         // cam bien hop B - tren board in chu "VN"
#define NUT_XONG   14
#define LED_HOP_A   5
#define LED_HOP_B  18
#define LED_XANH   19
#define LED_DO     23
#define COI        27
#define PIN_ENA    25
#define PIN_IN1    26
#define PIN_SERVO  13         // day vang cua servo -> D13
#define RS485_RX   16         // RX2 <- RO cua MAX485 (qua cau chia 10k/20k)
#define RS485_TX   17         // TX2 -> DI cua MAX485
#define RS485_DE   33         // DE va RE cua MAX485 noi chung

// hai kenh PWM nam tren hai bo dem thoi gian khac nhau
#define KENH_DONG_CO  0       // kenh 0 -> bo dem 0, 1000 Hz
#define KENH_SERVO    2       // kenh 2 -> bo dem 1, 50 Hz

// ---------------- MODBUS ----------------
#define MODBUS_ID     3
#define MODBUS_BAUD   9600
#define SO_COIL       4
#define SO_DI        13
#define SO_IR        10
#define SO_HR         4

// ---------------- THAM SO ----------------
int tocDo = 120;                               // 0..255
unsigned long tToiTram2  = 2000;               // ms, chay tu tram 1 sang tram 2
unsigned long tChoTram3  = 6000;               // ms, thoi gian toi da tu luc roi tram 2
                                               //     den luc xe di qua het tram 3
const unsigned long CUA_SO_QUET = 5000;        // tram 1 dung toi da 5 giay de quet ma
const unsigned long QUET_SOM    = 2000;        // nhan ma quet som truoc khi xe toi
const unsigned long T_LAP_RAP   = 10000;       // tram 2 dung 10 giay de lap
const unsigned long T_DUNG_T3   = 1000;        // tram 3 dung 1 giay khi thay linh kien
const unsigned long T_GAT       = 5000;        // servo giu can gat 5 giay roi tu ve
const unsigned long T_RA_LINE   = 5000;        // chay not cho xe ra khoi bang tai
const int GOC_NGHI = 15, GOC_GAT = 105;        // chinh lai theo thuc te
const int XUNG_MIN = 600, XUNG_MAX = 2400;     // do rong xung (micro giay) cho MG995

#if CO_LCD
  #include <Wire.h>
  #include <LiquidCrystal_I2C.h>
  LiquidCrystal_I2C lcd(0x27, 16, 2);
#endif

WebServer server(80);

// ---------------- BIEN TRANG THAI ----------------
int soVao = 0, soXe1 = 0, soXe2 = 0, soOK = 0, soNG = 0, soLoiQuet = 0;
int maMoi = 0, modelHienTai = 0;
bool daQuet = false, layDung = false, daBaoSai = false;
bool servoDangGat = false;                     // servo dang giu can
unsigned long tServoGat = 0;                   // thoi diem servo bat dau gat
int giayCuoi = -1;
unsigned long tMaMoi = 0, tMoc = 0, tTatCoi = 0, tXongTruoc = 0;
float nhip = 0;
String thongBao = "San sang", maCuoi = "(chua co)";
bool nutTruoc = HIGH;
unsigned long tNut = 0;
bool cbTram1 = false, cbLinhKien = false, cbHopA = false, cbHopB = false;
bool bangTaiChay = false;
bool tamDung = false;                          // may tinh ra lenh tam dung
unsigned long tBatDauDung = 0;
bool trangThaiRa[40] = {false};                // muc dang xuat ra cua LED va coi
unsigned long soKhungDung = 0, soKhungLoiCrc = 0;

enum { T1_CHAY, T1_CHO_QUET, T1_LOI, T1_XE_ROI,
       T2_DI_TOI, T2_LAP,
       T3_DI_TOI, T3_DUNG, RA_LINE } tt = T1_CHAY;

// ---------------- TIEN ICH ----------------
// xuat muc ra chan va nho lai de bao ve may tinh
void ra(int pin, bool muc) {
  digitalWrite(pin, muc);
  trangThaiRa[pin] = muc;
}

// quay servo toi goc chi dinh bang xung 50 Hz, do phan giai 16 bit
void servoGoc(int goc) {
  goc = constrain(goc, 0, 180);
  uint32_t xung = XUNG_MIN + (uint32_t)goc * (XUNG_MAX - XUNG_MIN) / 180;   // micro giay
  uint32_t duty = xung * 65535UL / 20000UL;                                  // chu ky 20 ms
  ledcWrite(PIN_SERVO, duty);
}

// servo gat va tu ve sau T_GAT, khong chan chuong trinh
void gatServo() {
  servoGoc(GOC_GAT);
  servoDangGat = true;
  tServoGat = millis();
  Serial.println(">>> Servo gat ra");
}

void capNhatServo() {
  if (servoDangGat && millis() - tServoGat >= T_GAT) {
    servoGoc(GOC_NGHI);
    servoDangGat = false;
    Serial.println(">>> Servo ve vi tri nghi");
  }
}

bool docCamBien(int pin) {                     // chong nhieu: on dinh 50ms moi tinh
  static bool trangThai[40] = {false};
  static bool docTruoc[40]  = {false};
  static unsigned long tDoi[40] = {0};
  bool doc = (digitalRead(pin) == LOW);
  if (doc != docTruoc[pin]) { docTruoc[pin] = doc; tDoi[pin] = millis(); }
  if (millis() - tDoi[pin] > 50) trangThai[pin] = doc;
  return trangThai[pin];
}

bool nutVuaBam() {
  bool nay = digitalRead(NUT_XONG);
  bool bam = (nutTruoc == HIGH && nay == LOW && millis() - tNut > 300);
  if (bam) tNut = millis();
  nutTruoc = nay;
  return bam;
}

void chayBangTai(bool chay) {
  bangTaiChay = chay;
  digitalWrite(PIN_IN1, chay ? HIGH : LOW);
  ledcWrite(PIN_ENA, chay ? tocDo : 0);
}

void batCoi(unsigned long ms) { ra(COI, HIGH); tTatCoi = millis() + ms; }
void capNhatCoi() { if (tTatCoi && millis() > tTatCoi) { ra(COI, LOW); tTatCoi = 0; } }

void hienThi(String d1, String d2) {
  thongBao = d2;
  Serial.println(d1 + " | " + d2);
#if CO_LCD
  while (d1.length() < 16) d1 += ' ';
  while (d2.length() < 16) d2 += ' ';
  lcd.setCursor(0, 0); lcd.print(d1.substring(0, 16));
  lcd.setCursor(0, 1); lcd.print(d2.substring(0, 16));
#endif
}

void sangDenHop(int model) {
  ra(LED_HOP_A, model == 1);
  ra(LED_HOP_B, model == 2);
}

void ghiNhip() {
  if (tXongTruoc) nhip = (millis() - tXongTruoc) / 1000.0;
  tXongTruoc = millis();
}

void xoaBoDem() {
  soVao = soXe1 = soXe2 = soOK = soNG = soLoiQuet = 0;
  tXongTruoc = 0; nhip = 0;
  Serial.println(">>> Da xoa bo dem");
}

// tam dung hoac chay tiep theo lenh cua may tinh
void datTamDung(bool dung) {
  if (dung == tamDung) return;
  tamDung = dung;
  if (dung) {
    tBatDauDung = millis();
    chayBangTai(false);
    hienThi("TAM DUNG", "Lenh tu may tinh");
  } else {
    tMoc += millis() - tBatDauDung;            // bu thoi gian da dung, cac moc thoi gian giu nguyen
    giayCuoi = -1;
    hienThi("Chay tiep", "Lenh tu may tinh");
  }
}

// dua day chuyen ve trang thai ban dau, bo dem giu nguyen
void resetDayChuyen() {
  tamDung = false;
  maMoi = 0; modelHienTai = 0;
  daQuet = false; layDung = false; daBaoSai = false;
  sangDenHop(0);
  ra(LED_XANH, LOW); ra(LED_DO, LOW);
  ra(COI, LOW); tTatCoi = 0;
  servoGoc(GOC_NGHI); servoDangGat = false;
  giayCuoi = -1;
  tMoc = millis();
  tt = T1_CHAY;
  hienThi("RESET LINE", "Dat xe len line");
}

// ---------------- NHAN MA QR TU DIEN THOAI ----------------
void xuLyQuet() {
  String tatCa = server.uri();
  for (int i = 0; i < server.args(); i++)
    tatCa += " " + server.argName(i) + "=" + server.arg(i);
  Serial.println("[Nhan duoc] " + tatCa);

  int v = tatCa.indexOf("XEMAY-");
  int model = (v >= 0) ? tatCa.substring(v + 6, v + 7).toInt() : 0;

  if (model == 1 || model == 2) {
    int cuoiMa = v;
    while (cuoiMa < tatCa.length() && tatCa.charAt(cuoiMa) != ' ' && tatCa.charAt(cuoiMa) != '&') cuoiMa++;
    maCuoi = tatCa.substring(v, cuoiMa);
    maMoi = model;
    tMaMoi = millis();
    server.send(200, "text/plain", "OK " + maCuoi);
    Serial.printf(">>> Doc duoc ma: %s (model %d)%s\n", maCuoi.c_str(), model,
                  (tt == T1_CHAY) ? "   [chua co xe o tram 1 - giu lai 2 giay]" : "");
  } else {
    soLoiQuet++;
    server.send(400, "text/plain", "Ma khong hop le");
  }
}

// may tinh gui ma thay cho dien thoai (ghi holding register 3)
void nhanMaTuMayTinh(int model) {
  maCuoi = "XEMAY-" + String(model) + " (may tinh)";
  maMoi = model;
  tMaMoi = millis();
  Serial.printf(">>> May tinh gui ma: XEMAY-%d\n", model);
}

// ---------------- TRANG GIAM SAT ----------------
void trangGiamSat() {
  int tong = soOK + soNG;
  String h = "<!DOCTYPE html><html><head><meta charset='utf-8'>"
             "<meta name='viewport' content='width=device-width,initial-scale=1'>"
             "<meta http-equiv='refresh' content='2'><title>Day chuyen</title>"
             "<style>body{font-family:sans-serif;margin:20px}b{font-size:26px}"
             "div{margin:10px 0}</style></head><body><h2>Day chuyen lap rap xe may mini</h2>";
  h += "<div>Xe vao line: <b>" + String(soVao) + "</b> (model 1: " + String(soXe1) +
       " | model 2: " + String(soXe2) + ")</div>";
  h += "<div>Dat OK: <b>" + String(soOK) + "</b> &nbsp; Loi NG: <b>" + String(soNG) + "</b></div>";
  h += "<div>Ty le dat: <b>" + String(tong ? 100.0 * soOK / tong : 0, 1) + "%</b></div>";
  h += "<div>Nhip san xuat: <b>" + String(nhip, 1) + "</b> giay/xe</div>";
  h += "<div>Ma doc gan nhat: " + maCuoi + "</div>";
  h += "<div>Trang thai: " + thongBao + "</div>";
  h += "<div>Toc do: " + String(tocDo) + " | Toi tram 2: " + String(tToiTram2) +
       " ms | Cho toi da tram 3: " + String(tChoTram3) + " ms</div></body></html>";
  server.send(200, "text/html", h);
}

// ---------------- MAY TRANG THAI ----------------
void nhanXeVaoLine(int model) {
  modelHienTai = model;
  maMoi = 0;
  soVao++;
  if (model == 1) soXe1++; else soXe2++;
  ra(LED_DO, LOW);
  ra(LED_XANH, HIGH);
  tt = T1_XE_ROI;
  hienThi("Xe " + String(model) + " vao line", "Di toi tram 2");
}

void capNhatDayChuyen() {
  cbTram1    = docCamBien(IR_TRAM1);
  cbLinhKien = docCamBien(IR_T3_TREN);
  cbHopA     = docCamBien(IR_HOP_A);
  cbHopB     = docCamBien(IR_HOP_B);
  bool bamXong = nutVuaBam();

  if (tamDung) {                               // dang tam dung: chi doc cam bien, giu bang tai dung
    chayBangTai(false);
    return;
  }

  unsigned long troi = millis() - tMoc;

  switch (tt) {

    // ================= TRAM 1 =================
    case T1_CHAY:
      if (cbTram1) {
        tMoc = millis();
        daQuet = false; modelHienTai = 0; giayCuoi = -1;
        tt = T1_CHO_QUET;
        hienThi("Tram 1: xe toi", "Dang quet QR...");
      }
      break;

    case T1_CHO_QUET: {
      if (!daQuet && maMoi && tMaMoi + QUET_SOM >= tMoc) {
        daQuet = true; modelHienTai = maMoi; maMoi = 0;
        ra(LED_XANH, HIGH);
      }
      if (troi >= CUA_SO_QUET) {
#if CHE_DO_THU
        if (!daQuet) { daQuet = true; modelHienTai = 1; }
#endif
        if (daQuet) {
          nhanXeVaoLine(modelHienTai);
        } else {
          ra(LED_DO, HIGH);
          batCoi(600);
          tt = T1_LOI;
          hienThi("LOI tram 1", "Chua doc duoc ma");
        }
        break;
      }
      int conLai = (CUA_SO_QUET - troi + 999) / 1000;
      if (conLai != giayCuoi) {
        giayCuoi = conLai;
        hienThi(daQuet ? String("Da doc: Xe ") + modelHienTai : String("Dang quet QR"),
                "Con " + String(conLai) + " giay");
      }
      break;
    }

    case T1_LOI:
      if (maMoi && tMaMoi > tMoc) nhanXeVaoLine(maMoi);
      break;

    case T1_XE_ROI:
      if (!cbTram1) {
        ra(LED_XANH, LOW);
        maMoi = 0;
        tMoc = millis();
        tt = T2_DI_TOI;
      }
      break;

    // ================= TRAM 2 =================
    case T2_DI_TOI:
      if (troi >= tToiTram2) {
        tMoc = millis();
        layDung = false; daBaoSai = false; giayCuoi = -1;
        sangDenHop(modelHienTai);
        tt = T2_LAP;
        hienThi("Tram 2: model " + String(modelHienTai), "Lap linh kien");
      }
      break;

    case T2_LAP: {
      int conLai = (T_LAP_RAP > troi) ? (T_LAP_RAP - troi + 999) / 1000 : 0;
#if CO_CAM_BIEN_HOP
      bool chamDung = (modelHienTai == 1 && cbHopA) || (modelHienTai == 2 && cbHopB);
      bool chamSai  = (modelHienTai == 1 && cbHopB) || (modelHienTai == 2 && cbHopA);
      if (chamDung) { layDung = true; ra(LED_DO, LOW); }
      if (chamSai && !daBaoSai) {
        daBaoSai = true;
        ra(LED_DO, HIGH);
        batCoi(600);
        hienThi("Tram 2: LAY NHAM", "Sai hop linh kien");
        giayCuoi = conLai;
      }
#else
      layDung = true;
#endif
      if (bamXong && !layDung) {
        batCoi(300);
        hienThi("Chua lay dung", "linh kien!");
        giayCuoi = conLai;
      }
      bool diSom = bamXong && layDung;
      if (diSom || troi >= T_LAP_RAP) {
        sangDenHop(0);
        ra(LED_DO, LOW);
        tMoc = millis();
        tt = T3_DI_TOI;
        hienThi(diSom ? "Tram 2: da xong" : "Tram 2: het gio", "Di toi tram 3");
        break;
      }
      if (conLai != giayCuoi) {
        giayCuoi = conLai;
        hienThi(String("Lap hop ") + (modelHienTai == 1 ? "A" : "B"),
                "Con " + String(conLai) + " giay");
      }
      break;
    }

    // ================= TRAM 3 =================
    case T3_DI_TOI:                             // bang tai chay lien tuc
      if (cbLinhKien) {                         // thay linh kien -> dung ngay, gat ngay
        soOK++;
        ghiNhip();
        ra(LED_XANH, HIGH); ra(LED_DO, LOW);
        gatServo();
        tMoc = millis();
        tt = T3_DUNG;
        hienThi("Tram 3: DAT", "Dung 1s, gat xe");
      } else if (troi >= tChoTram3) {           // xe da di qua ma khong thay linh kien
        soNG++;
        ghiNhip();
        ra(LED_DO, HIGH); ra(LED_XANH, LOW);
        batCoi(800);
        tMoc = millis();
        tt = RA_LINE;
        hienThi("Tram 3: THIEU LK", "Xe chay ra cuoi");
      }
      break;

    case T3_DUNG:                               // dung dung 1 giay roi chay tiep
      if (troi >= T_DUNG_T3) {
        tMoc = millis();
        tt = RA_LINE;
        hienThi("Tram 3: DAT", "Bang tai chay tiep");
      }
      break;

    case RA_LINE:
      if (troi >= T_RA_LINE) {
        ra(LED_XANH, LOW);
        ra(LED_DO, LOW);
        tt = T1_CHAY;
        hienThi("San sang", "Dat xe tiep theo");
      }
      break;
  }

  bool dung = (tt == T1_CHO_QUET || tt == T1_LOI || tt == T2_LAP || tt == T3_DUNG);
  chayBangTai(!dung);
}

// ---------------- MODBUS RTU SLAVE QUA MAX485 ----------------
uint8_t mbVao[64];                             // khung dang nhan
uint8_t mbDai = 0;
unsigned long mbTCuoi = 0;                     // micros() luc nhan byte cuoi
const unsigned long MB_KHOANG_LANG = 4000;     // > 3,5 ky tu o 9600 bps (khoang 3,6 ms) = het khung

uint16_t tinhCrc(const uint8_t *d, int n) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < n; i++) {
    crc ^= d[i];
    for (int b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  }
  return crc;
}

// them CRC vao cuoi va gui, mang d phai con trong 2 byte
void mbGui(uint8_t *d, int n) {
  uint16_t crc = tinhCrc(d, n);
  d[n] = crc & 0xFF;
  d[n + 1] = crc >> 8;
  digitalWrite(RS485_DE, HIGH);
  Serial2.write(d, n + 2);
  Serial2.flush();                             // cho gui xong byte cuoi roi moi tra ve che do nhan
  digitalWrite(RS485_DE, LOW);
  while (Serial2.available()) Serial2.read();  // bo byte rac nhan duoc luc RO tha noi trong khi gui
}

void mbBaoLoi(uint8_t ham, uint8_t ma) {
  uint8_t tl[5] = {MODBUS_ID, (uint8_t)(ham | 0x80), ma};
  mbGui(tl, 3);
}

bool mbDocBit(uint8_t ham, uint16_t dc) {
  if (ham == 1) {                              // coil
    if (dc == 0) return tamDung;
    if (dc == 1) return servoDangGat;
    return false;                              // coil 2, 3 chi la lenh, doc luon ra 0
  }
  switch (dc) {                                // discrete input
    case 0:  return cbTram1;
    case 1:  return cbLinhKien;
    case 2:  return cbHopA;
    case 3:  return cbHopB;
    case 4:  return digitalRead(NUT_XONG) == LOW;
    case 5:  return bangTaiChay;
    case 6:  return servoDangGat;
    case 7:  return trangThaiRa[LED_HOP_A];
    case 8:  return trangThaiRa[LED_HOP_B];
    case 9:  return trangThaiRa[LED_XANH];
    case 10: return trangThaiRa[LED_DO];
    case 11: return trangThaiRa[COI];
    case 12: return tamDung;
  }
  return false;
}

uint16_t mbDocThanhGhi(uint8_t ham, uint16_t dc) {
  if (ham == 3) {                              // holding register
    switch (dc) {
      case 0: return tocDo;
      case 1: return tToiTram2;
      case 2: return tChoTram3;
    }
    return 0;                                  // dia chi 3 chi la lenh, doc luon ra 0
  }
  switch (dc) {                                // input register
    case 0: return tt;
    case 1: return modelHienTai;
    case 2: return soVao;
    case 3: return soXe1;
    case 4: return soXe2;
    case 5: return soOK;
    case 6: return soNG;
    case 7: return soLoiQuet;
    case 8: return (uint16_t)(nhip * 10 + 0.5);
    case 9: return ((tt == T1_CHO_QUET || tt == T2_LAP) && giayCuoi > 0) ? giayCuoi : 0;
  }
  return 0;
}

void mbGhiCoil(uint16_t dc, bool bat) {
  if (dc == 0) datTamDung(bat);
  else if (dc == 1 && bat) gatServo();
  else if (dc == 2 && bat) xoaBoDem();
  else if (dc == 3 && bat) resetDayChuyen();
}

// tra ve 0 neu ghi duoc, nguoc lai la ma loi Modbus
uint8_t mbGhiThanhGhi(uint16_t dc, uint16_t gt) {
  switch (dc) {
    case 0:
      if (gt > 255) return 3;
      tocDo = gt;
      return 0;
    case 1:
      if (gt < 200 || gt > 20000) return 3;
      tToiTram2 = gt;
      return 0;
    case 2:
      if (gt < 1000 || gt > 30000) return 3;
      tChoTram3 = gt;
      return 0;
    case 3:
      if (gt != 1 && gt != 2) return 3;
      nhanMaTuMayTinh(gt);
      return 0;
  }
  return 2;
}

void xuLyKhungModbus() {
  if (mbDai < 8) return;                       // khung qua ngan, bo qua
  uint16_t crcNhan = mbVao[mbDai - 2] | (mbVao[mbDai - 1] << 8);
  if (tinhCrc(mbVao, mbDai - 2) != crcNhan) { soKhungLoiCrc++; return; }
  if (mbVao[0] != MODBUS_ID) return;           // khung gui cho Slave khac
  soKhungDung++;

  uint8_t ham = mbVao[1];
  uint16_t dc = (mbVao[2] << 8) | mbVao[3];
  uint16_t gt = (mbVao[4] << 8) | mbVao[5];    // so luong (doc) hoac gia tri (ghi)
  uint8_t tl[72];
  tl[0] = MODBUS_ID;
  tl[1] = ham;

  switch (ham) {
    case 1: case 2: {                          // doc coil / discrete input
      uint16_t toiDa = (ham == 1) ? SO_COIL : SO_DI;
      if (gt == 0 || gt > 64) { mbBaoLoi(ham, 3); return; }
      if (dc + gt > toiDa) { mbBaoLoi(ham, 2); return; }
      uint8_t soByte = (gt + 7) / 8;
      tl[2] = soByte;
      memset(tl + 3, 0, soByte);
      for (uint16_t i = 0; i < gt; i++)
        if (mbDocBit(ham, dc + i)) tl[3 + i / 8] |= 1 << (i % 8);
      mbGui(tl, 3 + soByte);
      break;
    }
    case 3: case 4: {                          // doc holding / input register
      uint16_t toiDa = (ham == 3) ? SO_HR : SO_IR;
      if (gt == 0 || gt > 30) { mbBaoLoi(ham, 3); return; }
      if (dc + gt > toiDa) { mbBaoLoi(ham, 2); return; }
      tl[2] = gt * 2;
      for (uint16_t i = 0; i < gt; i++) {
        uint16_t v = mbDocThanhGhi(ham, dc + i);
        tl[3 + 2 * i] = v >> 8;
        tl[4 + 2 * i] = v & 0xFF;
      }
      mbGui(tl, 3 + gt * 2);
      break;
    }
    case 5:                                    // ghi 1 coil
      if (gt != 0xFF00 && gt != 0x0000) { mbBaoLoi(ham, 3); return; }
      if (dc >= SO_COIL) { mbBaoLoi(ham, 2); return; }
      mbGhiCoil(dc, gt == 0xFF00);
      memcpy(tl, mbVao, 6);                    // tra loi bang chinh khung da nhan
      mbGui(tl, 6);
      break;
    case 6: {                                  // ghi 1 holding register
      uint8_t ma = mbGhiThanhGhi(dc, gt);
      if (ma) { mbBaoLoi(ham, ma); return; }
      memcpy(tl, mbVao, 6);
      mbGui(tl, 6);
      break;
    }
    default:
      mbBaoLoi(ham, 1);                        // ma ham khong ho tro
  }
}

void capNhatModbus() {
  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    if (mbDai < sizeof(mbVao)) mbVao[mbDai++] = b;
    mbTCuoi = micros();
  }
  if (mbDai > 0 && micros() - mbTCuoi > MB_KHOANG_LANG) {
    xuLyKhungModbus();
    mbDai = 0;
  }
}

// ---------------- PHIM TAT TREN SERIAL ----------------
void doLenhSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  switch (c) {
    case '+': tocDo = constrain(tocDo + 10, 0, 255); Serial.printf(">>> Toc do: %d\n", tocDo); break;
    case '-': tocDo = constrain(tocDo - 10, 0, 255); Serial.printf(">>> Toc do: %d\n", tocDo); break;
    case 'q': tToiTram2 += 200; Serial.printf(">>> Toi tram 2: %lu ms\n", tToiTram2); break;
    case 'a': if (tToiTram2 > 200) tToiTram2 -= 200; Serial.printf(">>> Toi tram 2: %lu ms\n", tToiTram2); break;
    case 'w': tChoTram3 += 500; Serial.printf(">>> Cho toi da tram 3: %lu ms\n", tChoTram3); break;
    case 's': if (tChoTram3 > 1000) tChoTram3 -= 500; Serial.printf(">>> Cho toi da tram 3: %lu ms\n", tChoTram3); break;
    case 'g':
      Serial.printf(">>> Gat thu: %d -> %d do, giu %lu giay roi tu ve\n",
                    GOC_NGHI, GOC_GAT, T_GAT / 1000);
      gatServo();
      break;
    case 'r':
      xoaBoDem();
      break;
    case 't':                                  // thu duong phat RS485: may tinh phai nhan duoc 5 dong
      for (int k = 1; k <= 5; k++) {
        digitalWrite(RS485_DE, HIGH);
        Serial2.printf("RS485 TEST %d tu ESP32\r\n", k);
        Serial2.flush();
        digitalWrite(RS485_DE, LOW);
        delay(200);
      }
      while (Serial2.available()) Serial2.read();
      Serial.println(">>> Da gui 5 dong thu qua RS485");
      break;
    case 'i':
      Serial.printf(">>> IP %s | toc do %d | toi T2 %lu ms | cho toi da T3 %lu ms\n",
                    WiFi.softAPIP().toString().c_str(), tocDo, tToiTram2, tChoTram3);
      Serial.printf("    OK %d | NG %d | nhip %.1f s\n", soOK, soNG, nhip);
      Serial.printf("    Cam bien (1 = co vat): T1=%d  T3=%d  HopA=%d  HopB=%d\n",
                    cbTram1, cbLinhKien, cbHopA, cbHopB);
      Serial.printf("    Modbus ID %d: %lu khung dung | %lu khung sai CRC%s\n",
                    MODBUS_ID, soKhungDung, soKhungLoiCrc, tamDung ? " | DANG TAM DUNG" : "");
      break;
  }
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(IR_TRAM1,   INPUT);
  pinMode(IR_T3_TREN, INPUT);
  pinMode(IR_HOP_A,   INPUT);
  pinMode(IR_HOP_B,   INPUT);
  pinMode(NUT_XONG,   INPUT_PULLUP);

  int dauRa[] = {LED_HOP_A, LED_HOP_B, LED_XANH, LED_DO, COI};
  for (int p : dauRa) { pinMode(p, OUTPUT); ra(p, LOW); }

  pinMode(PIN_IN1, OUTPUT);

  // MAX485 o che do nhan truoc khi mo cong
  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW);
  Serial2.begin(MODBUS_BAUD, SERIAL_8N1, RS485_RX, RS485_TX);

  // gan kenh co dinh cho tung chan, tranh dung chung bo dem
  bool okDongCo = ledcAttachChannel(PIN_ENA,   1000, 8,  KENH_DONG_CO);
  bool okServo  = ledcAttachChannel(PIN_SERVO, 50,   16, KENH_SERVO);
  Serial.printf("PWM dong co: %s | PWM servo: %s\n",
                okDongCo ? "OK" : "LOI", okServo ? "OK" : "LOI");

  chayBangTai(false);
  servoGoc(GOC_NGHI);

#if CO_LCD
  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
#endif

  WiFi.softAP("DayChuyenXeMay", "12345678");
  Serial.println("\n==================================================");
  Serial.println("Wi-Fi: DayChuyenXeMay | mat khau: 12345678");
  Serial.print("Dia chi ESP32: ");
  Serial.println(WiFi.softAPIP());
  Serial.println("URL Binary Eye: http://192.168.4.1/scan?xe=");
  Serial.printf("RS485: Modbus RTU Slave ID %d, %d bps 8N1 (RX2=16, TX2=17, DE/RE=33)\n",
                MODBUS_ID, MODBUS_BAUD);
  Serial.println("Phim: +/- toc do | q/a tram 2 | w/s cho tram 3 | g gat thu | i thong tin | t thu RS485");
  Serial.println("==================================================");

  server.on("/", trangGiamSat);
  server.on("/scan", xuLyQuet);
  server.on("/favicon.ico", []() { server.send(204); });
  server.onNotFound(xuLyQuet);
  server.begin();

  tMoc = millis();
  hienThi("San sang", "Dat xe len line");
}

// ---------------- LOOP ----------------
void loop() {
  capNhatModbus();
  server.handleClient();
  capNhatDayChuyen();
  capNhatModbus();
  capNhatServo();                              // servo tu ve sau 5 giay
  capNhatCoi();
  doLenhSerial();
}
