// =============================================================================
// Experiments offered by "exp list", in console order. EXPS[0] is selected at
// boot. Each later stage adds one line per new experiment.
// =============================================================================
void registerExperiments() {
  EXPS[N_EXPS++] = &EXP_SWINGBAL;
  EXPS[N_EXPS++] = &EXP_FREESWING;
  EXPS[N_EXPS++] = &EXP_SIGNCHECK;
  EXPS[N_EXPS++] = &EXP_STEPTEST;
  curExp = EXPS[0];
}
