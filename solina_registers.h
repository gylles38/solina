#pragma once
// Tables des 6 registres du Solina, calibrées sur les références audio
// (phase 6 du prototype Python). Chaque registre = mélange de "taps" d'octave
// (offset relatif à la note jouée, poids) + formant : HP 1 pôle (fLo) et LP
// résonant 2 pôles (fHi, qHi). Les poids relatifs sont ceux du fit ; le niveau
// absolu (gain) est normalisé au démarrage pour des niveaux prévisibles.

#define SOLINA_NB_REG 6
#define SOLINA_MAX_TAPS 5

struct SolinaRegCfg
{
  int nTaps;
  int offset[SOLINA_MAX_TAPS];
  float weight[SOLINA_MAX_TAPS];
  float fLo;    // HP (Hz)
  float fHi;    // LP résonant (Hz)
  float qHi;    // Q du LP
  float gain;   // gain du registre
  bool lowOnly; // true = ne joue que dans le grave (<= LOW_LIMIT)
};

// Ordre : violin, viola, cello, contrabass, trumpet, horn
static const SolinaRegCfg SOLINA_REG[SOLINA_NB_REG] = {
    // violin : octaves 0/+1/+2 uniquement (pas de sous-octave) — dents de scie
    {3, {0, 1, 2, 0, 0}, {0.277f, 0.132f, 0.099f, 0.0f, 0.0f},
     182.0f, 8282.0f, 0.884f, 0.85f, false},
    // viola : La3 (ref.) — médium présent
    {5, {-2, -1, 0, 1, 2}, {0.452f, 0.009f, 0.901f, 0.431f, 0.210f},
     239.0f, 7693.0f, 1.222f, 1.00f, false},
    // cello : pas de référence dédiée — octave basse, grave
    {2, {-1, 0, 0, 0, 0}, {1.00f, 0.55f, 0.0f, 0.0f, 0.0f},
     70.0f, 2600.0f, 0.707f, 1.05f, true},
    // contrabass : fondamentale E2 dominante (ref. duo approximative)
    {2, {-2, -1, 0, 0, 0}, {0.55f, 1.00f, 0.0f, 0.0f, 0.0f},
     45.0f, 1800.0f, 0.707f, 1.05f, true},
    // trumpet : La3 (ref.) — résonance ~4.3 kHz (Q 2.2)
    {5, {-2, -1, 0, 1, 2}, {0.061f, 0.020f, 0.634f, 0.786f, 0.197f},
     83.0f, 4336.0f, 2.171f, 0.90f, false},
    // horn : La3 (ref.) — résonance marquée ~1.7 kHz (Q 3.8)
    {5, {-2, -1, 0, 1, 2}, {0.064f, 0.047f, 0.143f, 0.070f, 0.031f},
     57.0f, 1682.0f, 3.790f, 1.00f, false},
};

static const char *SOLINA_REG_NAME[SOLINA_NB_REG] = {
    "violin", "viola", "cello", "contrabass", "trumpet", "horn"};
