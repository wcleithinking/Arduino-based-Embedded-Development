#ifdef DEBUG
float debug_v, debug_v0, debug_v1, debug_v2, debug_v3;

void Serial_debug() {
#if defined(Debug_single)
  debug_v = Rate_measure[IndexRoll]*RadToDeg;
  Serial.println(debug_v);
#elif defined(Debug_multi)
  debug_v0 = pulse1; debug_v1 = pulse2; debug_v2 = pulse3; debug_v3 = pulse4;
  Serial.print("Var0:");Serial.print(debug_v0);Serial.print("\t");
  Serial.print("Var1:");Serial.print(debug_v1);Serial.print("\t");
  Serial.print("Var2:");Serial.print(debug_v2);Serial.print("\t");
  Serial.print("Var3:");Serial.print(debug_v3);Serial.println("\t");
#endif
}
#endif
