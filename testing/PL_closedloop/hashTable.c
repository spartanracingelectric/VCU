/*****************************************************************************
 * hashTable.c - Hash Table implementation
 * Initial Author: Harleen Sandhu / Mehul Williams
 ******************************************************************************
 ****************************************************************************/

#include <stdlib.h>



// Static constant hashtable
static const HashTable STATIC_HASH_TABLE = {{NULL}}; // Initialized with all pointers set to NULL


// Create a new hash table
HashTable* HashTable_new() {
    HashTable* me = (HashTable*)malloc(sizeof(HashTable));
    for (int i = 0; i < TABLE_SIZE; i++) {
        me->entries[i] = NULL;
    }
    return me;
}


// Hash function 
int hash(int key1, int key2) {
    // A simple hash function combining key1 and key2
     return ((key1 * 31 + key2) % TABLE_SIZE + TABLE_SIZE) % TABLE_SIZE;
}


// Insert a key-value pair into the hash table
void HashTable_insertPair(HashTable* table, int key1, int key2, int value) {
    int index = hash(key1, key2);
    
    // Create a new entry
    HashEntry* entry = (HashEntry*)malloc(sizeof(HashEntry));
    entry->key1 = key1;
    entry->key2 = key2;
    entry->value = value;
    entry->next = NULL;
    
    // Handle collisions by chaining
    if (table->entries[index] != NULL) {
        HashEntry* current = table->entries[index];
        while (current->next != NULL) {
            current = current->next;
        }
        current->next = entry;
    } else {
        table->entries[index] = entry;
    }
}


// Retrieve a value from the hash table
int HashTable_getValue(HashTable* table, int key1, int key2) {
    int index = hash(key1, key2);
    HashEntry* entry = table->entries[index];
    while (entry != NULL) {
        if (entry->key1 == key1 && entry->key2 == key2) {
            return entry->value;
        }
        entry = entry->next;
    }
    return -1; // Key not found
}



// Destroy the hash table and free allocated memory
void destroyHashTable(HashTable* table) {
    for (int i = 0; i < TABLE_SIZE; i++) {
        HashEntry* current = table->entries[i];
        while (current != NULL) {
            HashEntry* next = current->next;
            free(current); 
            current = next;
        }
    }
    free(table);
}


// Main function
/*
float main() {
    // Create a hash table and initialize it with values from the lookup table
    HashTable* table = createHashTable();
    initializeHashTable(table);
    
    // Retrieve values
    printf("Value for (velocity = %d, steering = %d): %.2f\n", 5, 5, get(table, 5, 5));
    
    // Destroy the hash table
    destroyHashTable(table);

    return 0;

}
*/