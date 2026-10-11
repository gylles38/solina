// Solina String Ensemble MkII — ESP32 (ESP32-WROOM-DA) + DAC I2S PCM5102A.
// Moteur : solina_engine.h (portage du prototype Python calibré).
//
// Câblage identique au projet Minimoog :
//   I2S : BCK=26  WS=25  DATA=22  -> PCM5102
//   MIDI DIN : opto 6N138 -> GPIO3 (RX0), 31250 bauds 8N1
//
// Entrée MIDI (affectation MPK249) :
//   Note On/Off   : notes 36..84 (C2..C6)
//   Faders  18 21 22 23 24 25 26 27 : volume, tone, resonance, attack, release,
//                                     tone env, ensemble depth, ensemble rate
//   Potars   3  9 14 15 16 17 20 19 : ens dry, ens drive, ens tone, ens rate2,
//                                     phaser rate, phaser depth, phaser fb,
//                                     phaser center
//   Boutons 28 29 30 31 35 41 46 47 : violin, viola, cello, contrabass, trumpet,
//                                     horn, ensemble on/off, phaser on/off
//   Mod wheel CC1 : ensemble depth (expressif)   Pitch bend : ±2 demi-tons
//   CC 123        : All Notes Off
//
// Écoute : MIDI + note On suffit. Registres violin+viola+cello actifs au boot.
#include <ESP_I2S.h>
#include <esp_timer.h>
#include <math.h>
#include "solina_engine.h"

#define I2S_BCK 26
#define I2S_WS 25
#define I2S_DATA 22

#define MIDI_RX_PIN 3
#define MIDI_BAUD 31250

#define TAMPON_TAILLE 240

// Le moteur tourne à SolinaEngine::SR (22 050 Hz) et la sortie est
// suréchantillonnée ×2 vers 44 100 Hz pour le DAC PCM5102A.
#define SR_DAC 44100
#define FRAMES_ENG (TAMPON_TAILLE / 2)

// Limiteur de polyphonie : nombre de voix au démarrage et plafond de
// récupération automatique quand la charge CPU redescend.
#define VOIX_DEPART 4
#define VOIX_MAX 8

I2SClass i2s;
static SolinaEngine engine;

// --- File d'événements MIDI -> tâche audio (mono-producteur/mono-consommateur)
struct MidiEv
{
  uint8_t type; // 0=noteOff 1=noteOn 2=setParam 3=allNotesOff 4=switch 5=pitchBend
  uint8_t note;
  uint16_t val;
};
#define EVQ_LEN 64
static volatile MidiEv evQ[EVQ_LEN];
static volatile int evHead = 0; // écrit par la tâche MIDI
static volatile int evTail = 0; // lu par la tâche audio

static inline void pushEv(uint8_t type, uint8_t note, uint16_t val)
{
  int nh = (evHead + 1) % EVQ_LEN;
  if (nh == evTail)
    return; // plein : on abandonne l'événement
  evQ[evHead].type = type;
  evQ[evHead].note = note;
  evQ[evHead].val = val;
  evHead = nh;
}

static inline bool popEv(MidiEv &e)
{
  if (evTail == evHead)
    return false;
  e.type = evQ[evTail].type;
  e.note = evQ[evTail].note;
  e.val = evQ[evTail].val;
  evTail = (evTail + 1) % EVQ_LEN;
  return true;
}

// --- Télémétrie vers la GUI ------------------------------------------------
// Dernière valeur brute reçue pour chaque CC (0..127) : sert à l'affichage
// « valeurs et état des CC » côté PC. Ordre de la trame 'S' = CTRL_CC.
static volatile uint8_t g_cc[128];
static volatile int g_bend = 8192; // pitch bend 14 bits, centre 8192
static volatile int g_unkCC = 0;   // premier CC reçu non géré (0 = aucun)

static const uint8_t CTRL_CC[24] = {
    3, 9, 14, 15, 16, 17, 20, 19,   // potars rotatifs (gauche -> droite)
    18, 21, 22, 23, 24, 25, 26, 27, // faders (gauche -> droite)
    28, 29, 30, 31, 35, 41, 46, 47}; // boutons on/off (gauche -> droite)

// --- Affectation des CC du MPK249 ------------------------------------------
// type 4 (switch) : 0..5 = registres, 6 = ensemble, 7 = phaser.
// Retourne false si le CC n'est pas géré (diagnostic : le contrôle envoie
// peut-être un autre numéro de CC que celui attendu).
static bool ccEvent(uint8_t cc, uint8_t v)
{
  bool on = (v >= 64);
  switch (cc)
  {
  // Faders (gauche -> droite : 18 21 22 23 24 25 26 27)
  case 18: pushEv(2, SolinaEngine::P_VOLUME, v); break;
  case 21: pushEv(2, SolinaEngine::P_TONE, v); break;
  case 22: pushEv(2, SolinaEngine::P_RESO, v); break;
  case 23: pushEv(2, SolinaEngine::P_ATTACK, v); break;
  case 24: pushEv(2, SolinaEngine::P_RELEASE, v); break;
  case 25: pushEv(2, SolinaEngine::P_TONE_ENV, v); break;
  case 26: pushEv(2, SolinaEngine::P_ENS_DEPTH, v); break;
  case 27: pushEv(2, SolinaEngine::P_PH_DEPTH, v); break; // 8e fader = phaser
  // Potars rotatifs (3 9 14 15 16 17 20 19)
  case 3: pushEv(2, SolinaEngine::P_ENS_DRY, v); break;
  case 9: pushEv(2, SolinaEngine::P_ENS_DRIVE, v); break;
  case 14: pushEv(2, SolinaEngine::P_ENS_TONE, v); break;
  case 15: pushEv(2, SolinaEngine::P_ENS_RATE2, v); break;
  case 16: pushEv(2, SolinaEngine::P_PH_RATE, v); break;
  case 17: pushEv(2, SolinaEngine::P_PH_DEPTH, v); break;
  case 20: pushEv(2, SolinaEngine::P_PH_FB, v); break;
  case 19: pushEv(2, SolinaEngine::P_PH_CENTER, v); break;
  // Molette de modulation : profondeur d'ensemble (expressif)
  case 1: pushEv(2, SolinaEngine::P_ENS_DEPTH, v); break;
  // Boutons on/off (28 29 30 31 35 41 46 47)
  case 28: pushEv(4, 0, on); break; // violin
  case 29: pushEv(4, 1, on); break; // viola
  case 30: pushEv(4, 2, on); break; // cello
  case 31: pushEv(4, 3, on); break; // contrabass
  case 35: pushEv(4, 4, on); break; // trumpet
  case 41: pushEv(4, 5, on); break; // horn
  case 46: pushEv(4, 6, on); break; // ensemble on/off
  case 47: pushEv(4, 7, on); break; // phaser on/off
  case 123: pushEv(3, 0, 0); break;  // All Notes Off
  default: return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
void audioTask(void *pv)
{
  static int16_t buf[TAMPON_TAILLE * 2];
  static int16_t halfBuf[FRAMES_ENG * 2];
  float prevL = 0.0f, prevR = 0.0f;
  MidiEv e;

  const int BLOC_US = (int)(1000000LL * TAMPON_TAILLE / SR_DAC);
  int32_t renderMax = 0;
  int32_t writeMax = 0;
  int writeShort = 0;
  int loadCnt = 0;
  uint32_t lastReduce = 0;

  // Pas de démonstration au démarrage : tous les registres sont éteints
  // (silence tant qu'aucun registre/note n'est activé depuis le clavier maître).
  const int DEMO_BLOCS = (int)(2.5f * SR_DAC / TAMPON_TAILLE);
  int demoB = DEMO_BLOCS;
  bool demo = false;
  Serial.println("audio: tache demarree (attente MIDI - registres off)");

  while (1)
  {
    while (popEv(e))
    {
      if (demo && (e.type == 0 || e.type == 1))
      {
        engine.allNotesOff();
        demo = false;
        demoB = 0;
      }
      switch (e.type)
      {
      case 0:
        engine.noteOff(e.note);
        break;
      case 1:
        engine.noteOn(e.note);
        break;
      case 2:
        engine.setParam(e.note, e.val);
        break;
      case 3:
        engine.allNotesOff();
        break;
      case 4:
        if (e.note < SOLINA_NB_REG)
          engine.setRegister(e.note, e.val != 0);
        else if (e.note == 6)
          engine.ensemble = (e.val != 0);
        else if (e.note == 7)
          engine.phaser = (e.val != 0);
        break;
      case 5:
        engine.setPitchBend(e.val);
        break;
      }
    }

    int64_t tA = esp_timer_get_time();
    engine.render(halfBuf, FRAMES_ENG);
    // Suréchantillonnage ×2 (interpolation linéaire) vers 44 100 Hz.
    for (int i = 0; i < FRAMES_ENG; i++)
    {
      float l = halfBuf[2 * i];
      float r = halfBuf[2 * i + 1];
      buf[4 * i] = (int16_t)(0.5f * (prevL + l));
      buf[4 * i + 1] = (int16_t)(0.5f * (prevR + r));
      buf[4 * i + 2] = (int16_t)l;
      buf[4 * i + 3] = (int16_t)r;
      prevL = l;
      prevR = r;
    }
    int64_t tB = esp_timer_get_time();
    size_t wr = i2s.write((uint8_t *)buf, sizeof(buf));
    int64_t tC = esp_timer_get_time();

    // Diagnostic : temps de rendu DSP vs temps d'écriture I2S (séparés).
    int32_t rus = (int32_t)(tB - tA);
    int32_t wus = (int32_t)(tC - tB);
    if (rus > renderMax)
      renderMax = rus;
    if (wus > writeMax)
      writeMax = wus;
    if (wr != sizeof(buf))
      writeShort++;
    if (++loadCnt >= (SR_DAC / TAMPON_TAILLE))
    {
      // --- Limiteur de polyphonie adaptatif -------------------------------
      // Si le rendu s'approche du temps réel, on réduit le nombre de voix ;
      // si la marge redevient confortable, on le remonte prudemment.
      uint32_t now = millis();
      if (renderMax > (int32_t)(BLOC_US * 0.80f))
      {
        if (engine.maxVoices > 1)
        {
          engine.maxVoices--;
          engine.enforcePolyphony();
          lastReduce = now;
          Serial.printf("limiteur: surcharge -> voix max %d\n", engine.maxVoices);
        }
      }
      else if (renderMax < (int32_t)(BLOC_US * 0.45f) &&
               engine.maxVoices < VOIX_MAX && (now - lastReduce) > 5000)
      {
        engine.maxVoices++;
        Serial.printf("limiteur: marge -> voix max %d\n", engine.maxVoices);
      }

      Serial.printf("rendu max %ld us | ecriture max %ld us | bloc %d us | voix %d | ecrit. courtes %d\n",
                    (long)renderMax, (long)writeMax, (int)BLOC_US, engine.maxVoices, writeShort);
      renderMax = 0;
      writeMax = 0;
      writeShort = 0;
      loadCnt = 0;
    }

    if (demo && --demoB <= 0)
    {
      engine.allNotesOff();
      demo = false;
      Serial.println("(fin démo — en attente MIDI)");
    }
  }
}

// ---------------------------------------------------------------------------
void midiTask(void *pv)
{
  Serial1.begin(MIDI_BAUD, SERIAL_8N1, MIDI_RX_PIN, -1);

  uint8_t status = 0; // running status
  uint8_t data[2];
  int nData = 0;
  int need = 0;

  // Diagnostic : compteur d'octets bruts reçus (0 = câble/opto/pin).
  uint32_t nBytes = 0, bytesPrev = 0, tPrev = 0;
  uint32_t tFrame = 0;
  bool hintDone = false;

  while (1)
  {
    while (Serial1.available() > 0)
    {
      uint8_t b = Serial1.read();
      nBytes++;

      if (b & 0x80)
      {
        if (b >= 0xF8)
          continue; // temps réel : ignorés sans casser le running status
        if (b >= 0xF0)
        {
          status = 0;
          nData = 0;
          continue; // System Common / SysEx
        }
        status = b;
        nData = 0;
        need = ((b & 0xF0) == 0xC0 || (b & 0xF0) == 0xD0) ? 1 : 2;
        continue;
      }

      if (status == 0)
        continue;
      data[nData++] = b;
      if (nData < need)
        continue;

      uint8_t type = status & 0xF0;
      if (type == 0x80)
        pushEv(0, data[0], 0);
      else if (type == 0x90)
      {
        if (data[1] == 0)
          pushEv(0, data[0], 0); // Note On vélocité 0 = Note Off
        else
        {
          pushEv(1, data[0], data[1]);
          Serial.printf("Note ON  %d\n", data[0]);
        }
      }
      else if (type == 0xB0)
      {
        // Clavier maître : jour la trame de télémétrie avec la valeur brute.
        g_cc[data[0]] = data[1];
        if (!ccEvent(data[0], data[1]) && g_unkCC == 0)
          g_unkCC = data[0]; // premier CC non géré → affiché par la GUI
      }
      else if (type == 0xE0)
      {
        int v14 = data[0] | (data[1] << 7);
        g_bend = v14;
        pushEv(5, 0, v14);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));

    // Trame de télémétrie pour la GUI (5 Hz) : valeurs/état des 24 CC, molette
    // de modulation, pitch bend et nombre de voix courant.
    uint32_t nowf = millis();
    if (nowf - tFrame >= 200)
    {
      tFrame = nowf;
      char fb[192];
      int n = snprintf(fb, sizeof(fb), "S");
      for (int i = 0; i < 24; i++)
        n += snprintf(fb + n, sizeof(fb) - n, " %u", (unsigned)g_cc[CTRL_CC[i]]);
      n += snprintf(fb + n, sizeof(fb) - n, " %u %d %d %d\n",
                    (unsigned)g_cc[1], g_bend, engine.maxVoices, g_unkCC);
      Serial.print(fb); // une seule écriture : pas d'entrelacement avec l'audio
    }

    uint32_t now = millis();
    if (now - tPrev >= 1000)
    {
      if (nBytes != bytesPrev)
        Serial.printf("MIDI: %u octets recus\n", (unsigned)(nBytes - bytesPrev));
      else if (!hintDone && now > 5000)
      {
        Serial.println("MIDI: aucun octet — vérifier câble DIN / optocoupleur (GPIO3)");
        hintDone = true;
      }
      bytesPrev = nBytes;
      tPrev = now;
    }
  }
}

// ---------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== SOLINA STRING ENSEMBLE MkII - ESP32 ===");

  i2s.setPins(I2S_BCK, I2S_WS, I2S_DATA, -1, -1);
  if (!i2s.begin(I2S_MODE_STD, SR_DAC, I2S_DATA_BIT_WIDTH_16BIT,
                 I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH))
  {
    Serial.println("Echec de l'initialisation du driver I2S");
    while (1)
      ;
  }
  Serial.println("I2S (PCM5102) OK");

  for (int r = 0; r < SOLINA_NB_REG; r++)
    Serial.printf("registre %d : %s %s\n", r, SOLINA_REG_NAME[r],
                  engine.regEn[r] ? "ON" : "off");

  engine.maxVoices = VOIX_DEPART;
  Serial.printf("limiteur de polyphonie : %d voix (max %d)\n", engine.maxVoices, VOIX_MAX);

  // Valeurs d'usine affichées par la GUI avant tout mouvement de contrôle :
  // on fixe chaque paramètre à une position CC par défaut cohérente.
  static const struct { uint8_t cc, p, def; } CCDEF[] = {
      {18, SolinaEngine::P_VOLUME, 71},   {21, SolinaEngine::P_TONE, 79},
      {22, SolinaEngine::P_RESO, 8},      {23, SolinaEngine::P_ATTACK, 22},
      {24, SolinaEngine::P_RELEASE, 37},  {25, SolinaEngine::P_TONE_ENV, 57},
      {26, SolinaEngine::P_ENS_DEPTH, 44}, {27, SolinaEngine::P_PH_DEPTH, 38},
      {3, SolinaEngine::P_ENS_DRY, 70},   {9, SolinaEngine::P_ENS_DRIVE, 56},
      {14, SolinaEngine::P_ENS_TONE, 65}, {15, SolinaEngine::P_ENS_RATE2, 9},
      {16, SolinaEngine::P_PH_RATE, 5},   {17, SolinaEngine::P_PH_DEPTH, 38},
      {20, SolinaEngine::P_PH_FB, 64},    {19, SolinaEngine::P_PH_CENTER, 22}};
  for (auto &d : CCDEF)
  {
    engine.setParam(d.p, d.def);
    g_cc[d.cc] = d.def;
  }
  // Boutons : tous les registres éteints + ensemble éteint par défaut
  // (silence total au démarrage ; chaque élément s'active via le clavier maître).
  g_cc[28] = g_cc[29] = g_cc[30] = 0;
  g_cc[31] = g_cc[35] = g_cc[41] = 0;
  g_cc[46] = 0; // ensemble off
  g_cc[47] = 0; // phaser off
  engine.ensemble = false; // cohérent avec g_cc[46]

  Serial.println("CC: faders 18/21/22/23/24/25/26/27 | potars 3/9/14/15/16/17/20/19");
  Serial.println("CC: boutons 28/29/30/31/35/41/46/47 | mod 1 | pitch bend");

  xTaskCreatePinnedToCore(audioTask, "Audio", 8192, NULL, 24, NULL, 1);
  xTaskCreatePinnedToCore(midiTask, "Midi", 4096, NULL, 2, NULL, 0);

  Serial.println("Pret : envoyer des Note On (C2..C6)\n");
}

void loop()
{
  vTaskDelete(NULL);
}
