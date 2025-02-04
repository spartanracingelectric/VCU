#include "hashTable.h"
#include "powerLimit.h"
#include "mathFunctions.h"

void PL_populateHashTable(HashTable* table);
int PL_getTorqueFromLUT(HashTable* torqueHashTable, int voltage, int rpm);
int PL_getTorqueFromLUT2(HashTable* torqueHashTable, int voltage, int rpm);