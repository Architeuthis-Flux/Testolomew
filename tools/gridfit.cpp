// Fit the row grid to anchors from a file: row hole x y z sigmaMm per line. Prints the fit and every anchor's miss.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "RowGrid.h"
int main(int argc, char** argv) {
    FILE* f = fopen(argv[1], "r"); RowAnchor a[100]; int n = 0; float row, hole, x, y, z, s;
    while (n < 100 && fscanf(f, "%f %f %f %f %f %f", &row, &hole, &x, &y, &z, &s) == 6) { a[n++] = { (int)row, (int)hole, { x, y, z }, s }; }
    RowGrid grid; rowGridDefault(&grid, -10, 22);
    float miss[100]; RowFitReport rep = rowGridFit(&grid, a, n, miss);
    printf("%d anchors, fit kind %d: rms %.2f mm, worst %.2f mm (row %d)\n", n, rep.kind, rep.rmsMm, rep.worstMm, a[rep.worst].row);
    printf("scale along %+.1f %%, across %+.1f %%, angle %.1f deg\n", (rowGridScaleAlong(&grid) - 1) * 100, (rowGridScaleAcross(&grid) - 1) * 100, rowGridAngleDeg(&grid));
    printf("#define ROWCOUNT_GRID_AT_BOOT { %.5ff, %.5ff, %.3ff, %.5ff, %.5ff, %.3ff }\n", grid.ax, grid.ay, grid.a0, grid.cx, grid.cy, grid.c0);
    double sumSq = 0; int inRow = 0;
    for (int k = 0; k < n; k++) { RowPlace p = rowGridPlace(&grid, a[k].position); RowPlace w = rowGridHolePlace(a[k].row, a[k].hole);
        float du = p.along - w.along, dv = p.acrossMm - w.acrossMm; sumSq += du * du; if (fabsf(du) < 0.5f) inRow++;
        printf("  row %2d hole %d: along %+.2f rows, across %+.2f mm\n", a[k].row, a[k].hole, du, dv); }
    printf("along miss rms %.2f rows; %d of %d within half a row\n", sqrt(sumSq / n), inRow, n);
}
