#ifdef DEBUG
float debug_v, debug_v0, debug_v1, debug_v2, debug_v3;

void Debug() {
#if defined(Debug_single)
  debug_v = yaw*RadToDeg;
  Serial.println(debug_v);
#elif defined(Debug_multi)
  debug_v0 = PWM_ref[0];
  debug_v1 = PWM_ref[1];
  debug_v2 = PWM_ref[2];
  debug_v3 = PWM_ref[3];
  Serial.print("Var0:");
  Serial.print(debug_v0);
  Serial.print("\t");
  Serial.print("Var1:");
  Serial.print(debug_v1);
  Serial.print("\t");
  Serial.print("Var2:");
  Serial.print(debug_v2);
  Serial.print("\t");
  Serial.print("Var3:");
  Serial.print(debug_v3);
  Serial.println("\t");
#endif
}
#endif

