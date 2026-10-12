#pragma once
// Tables des 6 registres du Solina, calibrées sur les références audio
// (phase 6 du prototype Python). Chaque registre = mélange de "taps" d'octave
// (offset relatif à la note jouée, poids) + formant : HP 2 pôles (fLo) et LP
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
    // violin : octave +1 pure (fondamentale jouée supprimée, ref. C4)
    {1, {1, 0, 0, 0, 0}, {1.00f, 0.0f, 0.0f, 0.0f, 0.0f},
     30.0f, 6056.0f, 0.500f, 0.85f, false},
    // viola : fondamentale C4 dominante + léger octave +1
    {2, {0, 1, 0, 0, 0}, {1.00f, 0.22f, 0.0f, 0.0f, 0.0f},
     30.0f, 13058.0f, 0.547f, 1.00f, false},
    // cello : fondamentale de la note, grave
    {1, {0, 0, 0, 0, 0}, {1.00f, 0.0f, 0.0f, 0.0f, 0.0f},
     30.0f, 7724.0f, 1.938f, 1.05f, true},
    // contrabass : une octave sous la note, grave
    {1, {-1, 0, 0, 0, 0}, {1.00f, 0.0f, 0.0f, 0.0f, 0.0f},
     30.0f, 12386.0f, 0.500f, 1.05f, true},
    // trumpet : fondamentale atténuée (HP 2 pôles), formant haut (ref. C4)
    {1, {0, 0, 0, 0, 0}, {1.00f, 0.0f, 0.0f, 0.0f, 0.0f},
     613.0f, 9520.0f, 0.500f, 0.90f, false},
    // horn : fondamentale C4, formant bas ~900 Hz (ref. C4)
    {1, {0, 0, 0, 0, 0}, {1.00f, 0.0f, 0.0f, 0.0f, 0.0f},
     60.0f, 920.0f, 1.134f, 1.00f, false},
};

static const char *SOLINA_REG_NAME[SOLINA_NB_REG] = {
    "violin", "viola", "cello", "contrabass", "trumpet", "horn"};
