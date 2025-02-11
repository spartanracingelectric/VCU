/*****************************************************************************
 * hashTable.h - Hash Table implementation
 * Initial Author: Harleen Sandhu / Mehul Williams
 ******************************************************************************
 * General purpose hash table implementation, initially designed for yaw values in Torque Vectoring.
 ****************************************************************************/

#ifndef _HASH_TABLE_H
#define _HASH_TABLE_H


#define TABLE_SIZE 700

// Define a structure for the hash table entry
typedef struct _HashEntry {
    int key1;
    int key2;
    int value;
    struct _HashEntry* next; // For chaining in case of collisions
} HashEntry;


// Define a structure for the hash table itself
typedef struct _HashTable{
    HashEntry* entries[TABLE_SIZE]; // Array of pointers to hash entries
} HashTable;


HashTable* HashTable_new();
int hash(int key1, int key2);
void HashTable_insertPair(HashTable* table, int key1, int key2, int value);
int HashTable_getValue(HashTable* table, int key1, int key2);
void destroyHashTable(HashTable* table);


#endif // _HASH_TABLE_H