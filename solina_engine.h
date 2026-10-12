#pragma once
// Moteur Solina String Ensemble — portage C++ du prototype Python
// (tools/sim_solina.py). Autonome, sans dépendance Arduino : peut être compilé
// et testé sur hôte.
//
// Chaîne : 12 oscillateurs maîtres (divide-down, carrés band-limités polyBLEP,
// phases verrouillées) -> 6 registres (taps d'octave + formant HP/LP résonant,
// chaque voix pondérée par sa propre enveloppe AR) -> filtre passe-bas global
// (SVF) -> ensemble 3 BBD + 2 LFO (stéréo) -> phaser optionnel -> sortie.
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "solina_registers.h"

class SolinaEngine
{
public:
  // Moteur à 22 050 Hz : deux fois moins d'échantillons à calculer. Le DAC est
  // alimenté en 44 100 Hz par suréchantillonnage ×2 dans le firmware.
  static const int SR = 22050;
  static const int TOP_MIDI = 96; // C7 : note la plus haute de la division
  static const int KEY_LO = 36;   // C2
  static const int KEY_HI = 84;   // C6
  static const int LOW_LIMIT = 55; // cello/contrabass limités au grave
  static const int MAX_HELD = 16;
  static const int NB_L = 10; // divisions d'octave utiles (L = 0..9)
  static const int MAX_SQ = 12 * NB_L;
  // Nombre de voix simultanées possibles : notes tenues + queues de release qui
  // se chevauchent (une même note rejouée pendant sa release crée une nouvelle
  // voix, d'où la marge par rapport à MAX_HELD).
  static const int MAX_SOUNDING = 24;
  static const int MAX_TAPS = MAX_SOUNDING * SOLINA_MAX_TAPS;
  static const int ENBUF_LEN = 1600;
  static const int PH_STAGES = 4;
  static const int PH_DECIM = 8; // recalcule du coeff phaser tous les 8 éch.

  // --- Réglages (défauts du prototype, ajustables) ------------------------
  float crescendo = 0.35f; // attaque (s)
  float sustainLen = 1.20f; // relâche (s)
  float tone = 0.62f;      // coupure de base 0..1
  float toneEnv = 0.45f;   // profondeur d'enveloppe sur la coupure
  float resonance = 0.06f;
  float outGain = 0.25f;

  bool ensemble = false; // désactivé au démarrage (état silencieux, à activer via CC)
  float ensRate = 0.55f, ensRate2 = 0.42f, ensDepth = 0.28f;
  float ensBase[3] = {6.5e-3f, 8.2e-3f, 10.5e-3f};
  float ensBbdFc = 5000.0f, ensDrive = 1.6f, ensDry = 0.55f;

  bool phaser = false;
  float phRate = 0.35f, phDepth = 0.75f, phFeedback = 0.45f, phCenter = 600.0f;

  bool regEn[SOLINA_NB_REG];

  // Polyphonie maximale : au-delà, la note la plus ancienne est volée. Réduit
  // automatiquement par le firmware si la charge CPU dépasse le temps réel.
  int maxVoices = 4;

  // Paramètres continus adressables par MIDI CC (via setParam).
  enum Param
  {
    P_TONE, P_TONE_ENV, P_RESO, P_VOLUME,
    P_ATTACK, P_RELEASE,
    P_ENS_DEPTH, P_ENS_RATE, P_ENS_RATE2, P_ENS_DRIVE, P_ENS_TONE, P_ENS_DRY,
    P_PH_RATE, P_PH_DEPTH, P_PH_FB, P_PH_CENTER,
    P_COUNT
  };

  SolinaEngine() { begin(); }

  // -----------------------------------------------------------------------
  void begin()
  {
    for (int pc = 0; pc < 12; pc++)
    {
      mphase[pc] = 0.0f;
      mcount[pc] = 0;
      mbase[pc] = 440.0f * powf(2.0f, (TOP_MIDI + pc - 69) / 12.0f) / (float)SR;
      minc[pc] = mbase[pc];
    }

    float sumAll = 0.0f;
    for (int r = 0; r < SOLINA_NB_REG; r++)
      sumAll += 1.0f; // (registres indépendants)

    for (int r = 0; r < SOLINA_NB_REG; r++)
    {
      // Tous les registres sont éteints par défaut : chaque registre
      // (violon, alto, violoncelle, contrebasse, trompette, cor) s'active via CC.
      regEn[r] = false;
      const SolinaRegCfg &c = SOLINA_REG[r];
      float sw = 0.0f;
      for (int t = 0; t < c.nTaps; t++)
        sw += fabsf(c.weight[t]);
      float inv = (sw > 1e-9f) ? (c.gain / sw) : 0.0f;
      for (int t = 0; t < c.nTaps; t++)
        effW[r][t] = c.weight[t] * inv;

      // formant : HP 1 pôle
      formHP_a[r] = expf(-2.0f * (float)M_PI * c.fLo / (float)SR);
      hpLp[r] = 0.0f;
      hpLp2[r] = 0.0f;
      // formant : LP résonant (SVF TPT, coeffs fixes)
      float fc = c.fHi;
      if (fc > 0.45f * SR)
        fc = 0.45f * SR;
      float g = tanf((float)M_PI * fc / (float)SR);
      float k = 1.0f / ((c.qHi > 0.05f) ? c.qHi : 0.05f);
      frA1[r] = 1.0f / (1.0f + g * (g + k));
      frA2[r] = g * frA1[r];
      frA3[r] = g * frA2[r];
      frIc1[r] = 0.0f;
      frIc2[r] = 0.0f;
    }

    atkC = 1.0f - expf(-1.0f / (crescendo * SR));
    relC = expf(-1.0f / (sustainLen * SR));

    gfIc1 = gfIc2 = 0.0f;

    ensIncA = 2.0f * (float)M_PI * ensRate / (float)SR;
    ensIncB = 2.0f * (float)M_PI * ensRate2 / (float)SR;
    aBbd = expf(-2.0f * (float)M_PI * ensBbdFc / (float)SR);
    ensLfoA = ensLfoB = 0.0f;
    ensDepthCur = ensDepth;
    bbd0 = bbd1 = bbd2 = 0.0f;
    ensW = 0;
    memset((void *)ensBuf, 0, sizeof(ensBuf));

    dcYl = dcYr = 0.0f;
    aDc = expf(-2.0f * (float)M_PI * 12.0f / (float)SR);

    phInc = 2.0f * (float)M_PI * phRate / (float)SR;
    phLfo = 0.0f;
    phFbL = phFbR = 0.0f;
    memset((void *)phX, 0, sizeof(phX));
    memset((void *)phY, 0, sizeof(phY));

    memset((void *)voices, 0, sizeof(voices));
    nVoices = 0;
    nActiveNote = 0;
    activeVoices = 0;
    memset((void *)noteEnvSum, 0, sizeof(noteEnvSum));
    lastEnv = 0.0f;

    rebuildActive();
  }

  // -----------------------------------------------------------------------
  int heldCount() const
  {
    int c = 0;
    for (int v = 0; v < nVoices; v++)
      if (voices[v].gate)
        c++;
    return c;
  }

  void setRegister(int r, bool on)
  {
    if (r < 0 || r >= SOLINA_NB_REG)
      return;
    regEn[r] = on;
    rebuildActive();
  }

  // Applique un paramètre continu à partir d'une valeur MIDI CC 0..127.
  void setParam(int p, int v)
  {
    if (v < 0)
      v = 0;
    if (v > 127)
      v = 127;
    float n = (float)v / 127.0f;
    switch (p)
    {
    case P_TONE:
      tone = n;
      break;
    case P_TONE_ENV:
      toneEnv = n;
      break;
    case P_RESO:
      resonance = n;
      break;
    case P_VOLUME:
      outGain = 0.45f * n;
      break;
    case P_ATTACK:
      crescendo = 0.005f + n * 1.995f;
      atkC = 1.0f - expf(-1.0f / (crescendo * (float)SR));
      break;
    case P_RELEASE:
      sustainLen = 0.05f + n * 3.95f;
      relC = expf(-1.0f / (sustainLen * (float)SR));
      break;
    case P_ENS_DEPTH:
      ensDepth = n * 0.8f;
      break;
    case P_ENS_RATE:
      ensRate = 0.05f + n * 4.95f;
      ensIncA = 2.0f * (float)M_PI * ensRate / (float)SR;
      break;
    case P_ENS_RATE2:
      ensRate2 = 0.05f + n * 4.95f;
      ensIncB = 2.0f * (float)M_PI * ensRate2 / (float)SR;
      break;
    case P_ENS_DRIVE:
      ensDrive = 0.5f + n * 2.5f;
      break;
    case P_ENS_TONE:
      ensBbdFc = 800.0f + n * 8200.0f;
      aBbd = expf(-2.0f * (float)M_PI * ensBbdFc / (float)SR);
      break;
    case P_ENS_DRY:
      ensDry = n;
      break;
    case P_PH_RATE:
      phRate = 0.03f + n * 7.97f;
      phInc = 2.0f * (float)M_PI * phRate / (float)SR;
      break;
    case P_PH_DEPTH:
      phDepth = n * 2.5f;
      break;
    case P_PH_FB:
      phFeedback = n * 0.9f;
      break;
    case P_PH_CENTER:
      phCenter = 100.0f + n * 2900.0f;
      break;
    }
  }

  // Pitch bend : valeur 14 bits (0..16383, centre 8192), ±2 demi-tons.
  void setPitchBend(int v14)
  {
    float semis = ((float)v14 - 8192.0f) / 8192.0f * 2.0f;
    float k = powf(2.0f, semis / 12.0f);
    for (int pc = 0; pc < 12; pc++)
      minc[pc] = mbase[pc] * k;
  }

  // Chaque note-on crée une voix indépendante (rejouer la même note pendant sa
  // release crée une voix de plus) avec sa propre enveloppe AR : les queues de
  // release se chevauchent naturellement et aucune note n'est coupée quand on en
  // rejoue une autre. Une voix n'est libérée qu'à extinction de son enveloppe.
  void noteOn(int n)
  {
    if (n < KEY_LO || n > KEY_HI || n > 127)
      return;
    int mv = (maxVoices < 1) ? 1 : maxVoices;
    while (countGated() >= mv) // vol de voix : libère les plus anciennes tenues
    {
      int victim = oldestGated();
      if (victim < 0)
        break;
      voices[victim].gate = false;
    }
    bool wasSilent = (nVoices == 0);
    int idx = allocVoice();
    if (idx < 0)
      return;
    voices[idx].note = (int8_t)n;
    voices[idx].gate = true;
    voices[idx].env = 0.0f; // attaque neuve (avec pulse) pour chaque note-on
    voices[idx].pulse = 0;
    if (wasSilent)
      activeVoices = 0; // nouvelle phrase : la normalisation repart de 1 voix
    int g = countGated();
    if (g > activeVoices)
      activeVoices = g;
    rebuildActive();
  }

  void noteOff(int n)
  {
    if (n < 0 || n > 127)
      return;
    for (int v = 0; v < nVoices; v++)
      if (voices[v].note == n && voices[v].gate) // plus ancienne voix tenue
      {
        voices[v].gate = false;
        return;
      }
  }

  void allNotesOff()
  {
    nVoices = 0;
    nActiveNote = 0;
    activeVoices = 0;
    memset((void *)noteEnvSum, 0, sizeof(noteEnvSum));
    lastEnv = 0.0f;
    rebuildActive();
  }

  // Ramène la polyphonie au nombre de voix tenues autorisé (appelé après
  // réduction de maxVoices par le firmware). Les queues de release continuent.
  void enforcePolyphony()
  {
    int mv = (maxVoices < 1) ? 1 : maxVoices;
    while (countGated() > mv)
    {
      int victim = oldestGated();
      if (victim < 0)
        break;
      voices[victim].gate = false;
    }
    rebuildActive();
  }

  // -----------------------------------------------------------------------
  // Rend un bloc stéréo entrelacé (int16).
  void render(int16_t *out, int frames)
  {
    // Retire les voix dont la release est terminée (reconstruit les taps si le
    // jeu de notes a changé).
    compactVoices();

    // Enveloppe représentative (max des voix) pour le suivi d'enveloppe du
    // filtre global, dont les coeffs sont recalculés une fois par bloc.
    float env = maxEnv();

    // Filtre global : coeffs recalculés une fois par bloc (env varie lentement)
    // Enveloppe de tone multiplicative bornée : une addition dans le domaine de
    // toneToHz (croissance très rapide) saturait la coupure contre Nyquist, ce
    // qui rendait la résonance inaudible dès que toneEnv > 0.
    float fc = toneToHz(tone) * powf(2.0f, toneEnv * env * 2.5f);
    if (fc > 0.45f * SR)
      fc = 0.45f * SR;
    float g = tanf((float)M_PI * fc / (float)SR);
    float kq = 1.0f / (0.5f + 3.5f * resonance);
    gfA1 = 1.0f / (1.0f + g * (g + kq));
    gfA2 = g * gfA1;
    gfA3 = g * gfA2;

    // Ensemble : LFO calculés une seule fois par bloc puis interpolés
    // linéairement entre le début et la fin du bloc (le LFO est très lent).
    float eM0s = 0, eM1s = 0, eM2s = 0, eM0e = 0, eM1e = 0, eM2e = 0;
    float ensDepthS = ensDepthCur, ensDepthE = ensDepthCur;
    float invFrames = (frames > 0) ? 1.0f / (float)frames : 0.0f;
    // Compensation de polyphonie (normalisation en énergie) : évite que la
    // somme des voix ne sature le soft-clip de sortie quand on joue un accord.
    float voiceScale = 1.0f / sqrtf((float)(activeVoices > 0 ? activeVoices : 1));
    // La saturation BBD est normalisée par le nombre de voix : la somme d'un
    // accord a un facteur de crête plus élevé qu'une note seule, donc elle
    // saturait davantage (intermodulation). On applique voiceScale à la drive
    // pour que l'étage sature pareil quelle que soit la polyphonie.
    float ensDriveEff = ensDrive * voiceScale;
    if (ensemble)
    {
      eM0s = sinf(ensLfoA);
      eM1s = sinf(ensLfoA + 2.0943951f);
      eM2s = sinf(ensLfoB);
      float ea = ensLfoA + ensIncA * (float)frames;
      float eb = ensLfoB + ensIncB * (float)frames;
      eM0e = sinf(ea);
      eM1e = sinf(ea + 2.0943951f);
      eM2e = sinf(eb);
      ensLfoA = ea;
      ensLfoB = eb;
      if (ensLfoA >= 6.2831853f)
        ensLfoA -= 6.2831853f;
      if (ensLfoB >= 6.2831853f)
        ensLfoB -= 6.2831853f;
      // profondeur lissée sur le bloc (évite le clic quand le CC varie)
      ensDepthE = ensDepthS + (ensDepth - ensDepthS) * ENS_DEPTH_SMOOTH;
      ensDepthCur = ensDepthE;
    }

    for (int i = 0; i < frames; i++)
    {
      // 0. enveloppes AR par voix, puis somme par note (pour les taps)
      for (int j = 0; j < nActiveNote; j++)
        noteEnvSum[activeNote[j]] = 0.0f;
      for (int v = 0; v < nVoices; v++)
      {
        Voice &e = voices[v];
        if (e.gate)
        {
          if (e.env < 1.0f)
          {
            e.env += (1.0f - e.env) * atkC;
            if (e.env > 1.0f)
              e.env = 1.0f;
          }
          if (e.pulse < PULSE_LEN)
          {
            e.env *= 0.4f + 0.6f * ((float)e.pulse / (float)PULSE_LEN);
            e.pulse++;
          }
        }
        else
        {
          e.env *= relC;
          if (e.env < ENV_EPS)
            e.env = 0.0f;
        }
        noteEnvSum[e.note] += e.env;
      }

      // 1. maîtres
      for (int pc = 0; pc < 12; pc++)
      {
        float ph = mphase[pc] + minc[pc];
        if (ph >= 1.0f)
        {
          ph -= 1.0f;
          mcount[pc]++;
        }
        mphase[pc] = ph;
      }

      // 2. carrés distincts nécessaires
      for (int j = 0; j < nSq; j++)
      {
        int id = sqList[j];
        sqVal[id] = oscAt(id / NB_L, id % NB_L);
      }

      // 3. accumulation par registre + formant (HP 1 pôle -> LP résonant)
      float bus = 0.0f;
      for (int r = 0; r < SOLINA_NB_REG; r++)
      {
        float acc = 0.0f;
        int nt = regTapsN[r];
        for (int t = 0; t < nt; t++)
          acc += sqVal[regTapsIdx[r][t]] * regTapsW[r][t] * noteEnvSum[regTapsNote[r][t]];

        float x = acc;
        float lpv = hpLp[r];
        lpv += (1.0f - formHP_a[r]) * (x - lpv);
        hpLp[r] = lpv;
        x -= lpv; // HP 1er pôle
        lpv = hpLp2[r];
        lpv += (1.0f - formHP_a[r]) * (x - lpv);
        hpLp2[r] = lpv;
        x -= lpv; // HP 2e pôle (pente 12 dB/oct)

        float c1 = frIc1[r], c2 = frIc2[r];
        float v3 = x - c2;
        float v1 = frA1[r] * c1 + frA2[r] * v3;
        float v2 = c2 + frA2[r] * c1 + frA3[r] * v3;
        frIc1[r] = 2.0f * v1 - c1;
        frIc2[r] = 2.0f * v2 - c2;
        bus += v2;
      }
      bus *= voiceScale;

      // (l'enveloppe AR est désormais appliquée par voix, avant le formant)

      // 5. filtre passe-bas global (SVF TPT)
      float c1 = gfIc1, c2 = gfIc2;
      float v3 = bus - c2;
      float v1 = gfA1 * c1 + gfA2 * v3;
      float v2 = c2 + gfA2 * c1 + gfA3 * v3;
      gfIc1 = 2.0f * v1 - c1;
      gfIc2 = 2.0f * v2 - c2;
      bus = v2;

      // 6. ensemble 3 BBD + 2 LFO (modulation interpolée depuis le bloc)
      float L, R;
      if (ensemble)
      {
        float t = invFrames * (float)i;
        float m0 = eM0s + (eM0e - eM0s) * t;
        float m1 = eM1s + (eM1e - eM1s) * t;
        float m2 = eM2s + (eM2e - eM2s) * t;
        float ed = ensDepthS + (ensDepthE - ensDepthS) * t;

        ensBuf[0][ensW] = bus;
        ensBuf[1][ensW] = bus;
        ensBuf[2][ensW] = bus;

        float s0 = readDelay(ensBuf[0], ensBase[0] * (1.0f + ed * m0));
        float s1 = readDelay(ensBuf[1], ensBase[1] * (1.0f + ed * m1));
        float s2 = readDelay(ensBuf[2], ensBase[2] * (1.0f + ed * m2));

        bbd0 += (1.0f - aBbd) * (s0 - bbd0);
        bbd1 += (1.0f - aBbd) * (s1 - bbd1);
        bbd2 += (1.0f - aBbd) * (s2 - bbd2);
        s0 = fastTanh(ensDriveEff * bbd0);
        s1 = fastTanh(ensDriveEff * bbd1);
        s2 = fastTanh(ensDriveEff * bbd2);

        ensW++;
        if (ensW >= ENBUF_LEN)
          ensW = 0;

        float dry = bus * ensDry;
        L = dry + 0.6f * s0 + 0.5f * s1;
        R = dry + 0.5f * s1 + 0.6f * s2;
      }
      else
      {
        L = R = bus;
      }

      // 7. phaser (optionnel). Le coefficient est recalculé tous les PH_DECIM
      // échantillons : l'échelle de temps du LFO (≪ 8 Hz) rend le lissage
      // imperceptible, et évite sinf/powf/tanf à chaque échantillon (ces
      // fonctions coûtent ~0,5 ms par bloc sur l'ESP32, ce qui faisait
      // s'écrouler la polyphonie quand le phaser était activé).
      if (phaser)
      {
        if ((i & (PH_DECIM - 1)) == 0)
        {
          phLfo += phInc * PH_DECIM;
          float fmod = phCenter * powf(2.0f, phDepth * 1.5f * sinf(phLfo));
          if (fmod > 0.45f * SR)
            fmod = 0.45f * SR;
          float tg = tanf((float)M_PI * fmod / (float)SR);
          phAp = (1.0f - tg) / (1.0f + tg);
        }
        L = allpass(L, 0, phAp);
        R = allpass(R, 1, phAp);
      }

      // 8. sortie : HP infra 12 Hz puis gain (l'enveloppe est déjà par voix)
      dcYl += (1.0f - aDc) * (L - dcYl);
      dcYr += (1.0f - aDc) * (R - dcYr);
      float yl = fastTanh((L - dcYl) * outGain);
      float yr = fastTanh((R - dcYr) * outGain);
      out[2 * i] = (int16_t)(yl * 32767.0f);
      out[2 * i + 1] = (int16_t)(yr * 32767.0f);
    }
  }

private:
  static const int PULSE_LEN = (int)(0.012f * SR);
  static constexpr float ENV_EPS = 0.001f; // seuil de fin de release
  // Lissage par bloc de la profondeur d'ensemble : la profondeur module le
  // retard BBD ; un saut par CC produit un clic (craquements en bougeant la
  // molette de modulation). On interpole la valeur sur le bloc.
  static constexpr float ENS_DEPTH_SMOOTH = 0.5f;

  // 1/2^L précalculés (évite des divisions dans oscAt).
  static constexpr float INV_TWO_L[NB_L] = {
      1.0f, 0.5f, 0.25f, 0.125f, 0.0625f, 0.03125f,
      0.015625f, 0.0078125f, 0.00390625f, 0.001953125f};

  // maîtres
  float mphase[12];
  uint32_t mcount[12];
  float minc[12];
  float mbase[12];

  // registres
  float effW[SOLINA_NB_REG][SOLINA_MAX_TAPS];
  float formHP_a[SOLINA_NB_REG];
  float hpLp[SOLINA_NB_REG];
  float hpLp2[SOLINA_NB_REG];
  float frA1[SOLINA_NB_REG], frA2[SOLINA_NB_REG], frA3[SOLINA_NB_REG];
  float frIc1[SOLINA_NB_REG], frIc2[SOLINA_NB_REG];

  // taps actifs
  bool sqUsed[MAX_SQ];
  int sqList[MAX_SQ];
  int nSq;
  float sqVal[MAX_SQ];
  int regTapsN[SOLINA_NB_REG];
  int regTapsIdx[SOLINA_NB_REG][MAX_TAPS];
  float regTapsW[SOLINA_NB_REG][MAX_TAPS];
  int regTapsNote[SOLINA_NB_REG][MAX_TAPS]; // note MIDI source (pour l'enveloppe)

  // Voix : chaque note-on crée une voix avec sa propre enveloppe AR. Une note
  // relâchée garde sa voix jusqu'à extinction (queue), et rejouer la même note
  // pendant sa release crée une voix supplémentaire : les deux se chevauchent.
  struct Voice
  {
    int8_t note;
    bool gate;
    float env;
    int pulse;
  };
  Voice voices[MAX_SOUNDING];
  int nVoices;
  int activeVoices; // compteur de normalisation de polyphonie (par phrase)
  int nActiveNote;              // notes distinctes actives
  int activeNote[MAX_SOUNDING]; // liste de ces notes

  // enveloppe
  float atkC, relC;
  float noteEnvSum[128]; // somme des enveloppes par note (calculée par échantillon)
  float lastEnv;         // enveloppe max courante (pour l'éclairage/telemetry)

  // filtre global
  float gfIc1, gfIc2, gfA1, gfA2, gfA3;

  // ensemble
  float ensBuf[3][ENBUF_LEN];
  int ensW;
  float aBbd, ensIncA, ensIncB, ensLfoA, ensLfoB;
  float ensDepthCur; // profondeur lissée (évite les clics quand le CC varie)
  float bbd0, bbd1, bbd2;

  // phaser
  float phX[2][PH_STAGES], phY[2][PH_STAGES];
  float phAp = 0.0f; // coefficient allpass courant (phaser)
  float phInc, phLfo, phFbL, phFbR;

  // sortie
  float dcYl, dcYr, aDc;

  // -----------------------------------------------------------------------
  static float toneToHz(float t)
  {
    if (t < 0.0f)
      t = 0.0f;
    if (t > 1.0f)
      t = 1.0f;
    float f = 20.0f * powf(1800.0f, t);
    if (f > 19000.0f)
      f = 19000.0f;
    return f;
  }

  // tanh approché par un polynôme impair deg 15 : AUCUNE division (les
  // divisions flottantes sont logicielles et très coûteuses sur ESP32).
  static inline float fastTanh(float x)
  {
    if (x <= -3.0f)
      return -1.0f;
    if (x >= 3.0f)
      return 1.0f;
    float x2 = x * x;
    return x * (1.0f + x2 * (-0.2927065404f + x2 * (0.0886561572f + x2 * (-0.02158218498f + x2 * (0.00370626384f + x2 * (-0.00040430803f + x2 * (0.000024733133f + x2 * -0.000000641287f)))))));
  }

  static inline float polyBlep(float t, float dt)
  {
    if (t < dt)
    {
      float x = t / dt;
      return x + x - x * x - 1.0f;
    }
    if (t > 1.0f - dt)
    {
      float x = (t - 1.0f) / dt;
      return x * x + x + x + 1.0f;
    }
    return 0.0f;
  }

  // Oscillateur maître divisé (L divisions d'octave). Dents de scie
  // band-limitées (polyBLEP) : spectre complet, sans trous aux multiples de 8
  // que produisait l'ancien carré 50 % après sommation des octaves.
  inline float oscAt(int pc, int L) const
  {
    uint32_t twoL = 1u << L;
    uint32_t m = mcount[pc] & (twoL - 1u);
    float inv = INV_TWO_L[L]; // 1/2^L : évite 3 divisions par carré et par échantillon
    float ph = ((float)m + mphase[pc]) * inv;
    if (ph >= 1.0f)
      ph -= 1.0f;
    float dt = minc[pc] * inv;
    float s = 2.0f * ph - 1.0f;
    s -= polyBlep(ph, dt);
    return s;
  }

  inline float readDelay(const float *buf, float ds) const
  {
    float rd = (float)ensW - ds * (float)SR;
    while (rd < 0.0f)
      rd += (float)ENBUF_LEN;
    int i0 = (int)rd;
    float fr = rd - (float)i0;
    int i1 = i0 + 1;
    if (i1 >= ENBUF_LEN)
      i1 = 0;
    return buf[i0] * (1.0f - fr) + buf[i1] * fr;
  }

  float allpass(float x, int ch, float ap)
  {
    float v = x + ((ch == 0) ? phFbL : phFbR) * phFeedback;
    for (int s = 0; s < PH_STAGES; s++)
    {
      float y = -ap * v + phX[ch][s] + ap * phY[ch][s];
      phX[ch][s] = v;
      phY[ch][s] = y;
      v = y;
    }
    float fb = fastTanh(v);
    if (ch == 0)
      phFbL = fb;
    else
      phFbR = fb;
    return v * 0.7f;
  }

  int countGated() const
  {
    int c = 0;
    for (int v = 0; v < nVoices; v++)
      if (voices[v].gate)
        c++;
    return c;
  }

  float maxEnv() const
  {
    float m = 0.0f;
    for (int v = 0; v < nVoices; v++)
      if (voices[v].env > m)
        m = voices[v].env;
    return m;
  }

  int oldestGated() const
  {
    for (int v = 0; v < nVoices; v++)
      if (voices[v].gate)
        return v;
    return -1;
  }

  // Alloue une voix libre, ou vole la plus silencieuse si le pool est plein.
  int allocVoice()
  {
    if (nVoices < MAX_SOUNDING)
      return nVoices++;
    int victim = -1;
    float lo = 1e9f;
    for (int v = 0; v < nVoices; v++)
      if (!voices[v].gate && voices[v].env < lo)
      {
        lo = voices[v].env;
        victim = v;
      }
    return victim;
  }

  // Retire les voix éteintes (release terminée) et reconstruit les taps si le
  // jeu de notes a changé.
  void compactVoices()
  {
    int w = 0;
    bool removed = false;
    for (int v = 0; v < nVoices; v++)
    {
      if (!voices[v].gate && voices[v].env <= 0.0f)
      {
        removed = true;
        continue;
      }
      if (w != v)
        voices[w] = voices[v];
      w++;
    }
    nVoices = w;
    if (nVoices == 0 && activeVoices != 0)
      activeVoices = 0;
    if (removed)
      rebuildActive();
  }

  void rebuildActive()
  {
    // Reconstruit la liste des notes distinctes qui sonnent (tenues ou en
    // release) puis la table des taps par registre, en mémorisant la note
    // source de chaque tap (pour appliquer son enveloppe).
    nActiveNote = 0;
    nSq = 0;
    memset((void *)sqUsed, 0, sizeof(sqUsed));
    for (int r = 0; r < SOLINA_NB_REG; r++)
      regTapsN[r] = 0;

    for (int v = 0; v < nVoices; v++)
    {
      int n = voices[v].note;
      bool known = false;
      for (int j = 0; j < nActiveNote; j++)
        if (activeNote[j] == n)
        {
          known = true;
          break;
        }
      if (known)
        continue;
      activeNote[nActiveNote++] = n;

      int pc = n % 12;
      int k = (TOP_MIDI + pc - n) / 12;
      for (int r = 0; r < SOLINA_NB_REG; r++)
      {
        if (!regEn[r])
          continue;
        const SolinaRegCfg &c = SOLINA_REG[r];
        if (c.lowOnly && n > LOW_LIMIT)
          continue;
        for (int t = 0; t < c.nTaps; t++)
        {
          int L = k - c.offset[t];
          if (L < 0 || L >= NB_L)
            continue;
          int id = pc * NB_L + L;
          if (!sqUsed[id])
          {
            sqUsed[id] = true;
            sqList[nSq++] = id;
          }
          int nt = regTapsN[r];
          if (nt < MAX_TAPS)
          {
            regTapsIdx[r][nt] = id;
            regTapsW[r][nt] = effW[r][t];
            regTapsNote[r][nt] = n;
            regTapsN[r] = nt + 1;
          }
        }
      }
    }
  }
};
