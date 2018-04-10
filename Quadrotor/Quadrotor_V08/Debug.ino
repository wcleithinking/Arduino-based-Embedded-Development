#ifdef DEBUG
double debug_v, debug_v0, debug_v1, debug_v2, debug_v3;

void Debug() {
#if defined(Debug_graph)
  debug_v = time_diff;
  Serial.println(v);
#elif defined(Debug_table)
  debug_v0 = u[0];
  debug_v1 = u[1];
  debug_v2 = u[2];
  debug_v3 = u[3];
  Serial.print("Variable0:");
  Serial.print(debug_v0);
  Serial.print("\t");
  Serial.print("Variable1:");
  Serial.print(debug_v1);
  Serial.print("\t");
  Serial.print("Variable2:");
  Serial.print(debug_v2);
  Serial.print("\t");
  Serial.print("Variable3:");
  Serial.print(debug_v3);
  Serial.println("\t");
#endif
}
#endif

