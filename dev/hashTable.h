/*****************************************************************************
 * hashTable.h - Hash Table implementation
 * Initial Author: Harleen Sandhu / Mehul Williams
 ******************************************************************************
 * General purpose hash table implementation, initially designed for yaw values in Torque Vectoring.
 ****************************************************************************/

#ifndef _HASH_TABLE_H
#define _HASH_TABLE_H

#include "IO_Driver.h"

#define TABLE_SIZE 100

// Define a structure for the hash table entry
typedef struct _HashEntry {
    ubyte2 key1;
    ubyte2 key2;
    ubyte4 value;
    struct _HashEntry* next; // For chaining in case of collisions
} HashEntry;


// Define a structure for the hash table itself
typedef struct _HashTable{
    HashEntry* entries[TABLE_SIZE]; // Array of pointers to hash entries
} HashTable;


HashTable* HashTable_new();
ubyte1 hash(ubyte2 key1, ubyte2 key2);
void insert(HashTable* table, ubyte2 key1, ubyte2 key2, ubyte4 value);
ubyte4 get(HashTable* table, ubyte2 key1, ubyte2 key2);
void destroyHashTable(HashTable* table);


#endif // _HASH_TABLE_H