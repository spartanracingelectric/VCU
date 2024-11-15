
#include "mathFunctions.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "DL.h"

// Function Prototypes
void PL_populateHashTable(HashTable* hash);

int main() {
    HashTable* hash = HashTable_new();

    // Populate the hash table
    PL_populateHashTable(hash);
    int d = int_lowerStepInterval(287,5);
    int e = int_lowerStepInterval(2086,160);
    // Retrieve value from the hash table
    int a = HashTable_getValue(hash, 290, 2320);
    int b = PL_getTorqueFromLUT(hash, 371, 2600);//my method
    int c = PL_getTorqueFromLUT2(hash, 293, 4000);//shaun method
    // Print the retrieved value
    printf("Val from floor287: %d\n", d);
    printf("Val from cel2086: %d\n", e);
    printf("Val from hashtable: %d\n", a);
    printf("Val from LUT: %d\n", b);
    printf("Val from LUT2: %d\n", c);
    return 0;
}
