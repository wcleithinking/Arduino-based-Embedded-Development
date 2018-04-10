#define N 5
void getInverse(double A[N][N], int n, double A_inv[N][N]) {
  double A_adj[N][N];
  double det = getDeterminant(A, n);
  getAdjoint(A, n, A_adj);
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) A_inv[i][j] = A_adj[i][j] / det;
  }
}

double getDeterminant(double A[N][N], int n) {
  if (n == 1) return A[0][0];
  double det = 0;
  double temp[N][N];
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n - 1; j++) {
      for (int k = 0; k < n - 1; k++) temp[j][k] = A[j + 1][(k >= i) ? k + 1 : k];
    }
    double subdet = getDeterminant(temp, n - 1);
    if (i % 2 == 0) det += A[0][i] * subdet;
    else det -= A[0][i] * subdet;
  }
  return det;
}

void getAdjoint(double A[N][N], int n, double ans[N][N]) {
  if (n == 1) {
    ans[0][0] = 1;
    return;
  }
  double temp[N][N];
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      for (int s = 0; s < n - 1; s++) {
        for (int t = 0; t < n - 1; t++) temp[s][t] = A[s >= i ? s + 1 : s][t >= j ? t + 1 : t];
      }
      ans[j][i]  =  getDeterminant(temp, n - 1);
      if ((i + j) % 2 == 1) ans[j][i] = - ans[j][i];
    }
  }
}

