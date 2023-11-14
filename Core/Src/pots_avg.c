#include <stdio.h>
#include <stdlib.h>

// Comparison function for qsort
int compare(const void *a, const void *b) {
    return (*(int*)a - *(int*)b);
}

int Median(int p1, int p2, int p3) {
    int x[] = {p1, p2, p3};
    int n = sizeof(x) / sizeof(x[0]);

    // Sort the array using qsort with the comparison function
    qsort(x, n, sizeof(int), compare);

    return x[1];
}


