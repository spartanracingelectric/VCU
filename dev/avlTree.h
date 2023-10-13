//http://www.zentut.com/c-tutorial/c-avl-tree/

#ifndef AVLTREE_H_INCLUDED
#define AVLTREE_H_INCLUDED

#include "IO_Driver.h"

typedef struct AVLNode
{
    ubyte1 data[8];

    ubyte4 timeBetweenMessages_Min; //Fastest rate at which messages will be sent
    ubyte4 lastMessage_timeStamp;   //Last time message was sent/received

    bool required;
    ubyte4 timeBetweenMessages_Max; //Slowest rate at which messages will be sent, OR max time between receiving messages before throwing an error

} AVLNode;

//Note on passing arrays: http://stackoverflow.com/questions/5573310/difference-between-passing-array-and-array-pointer-into-function-in-c
AVLNode *AVL_insert(AVLNode **t, ubyte4 messageID, ubyte1 messageData[8], ubyte4 timeBetweenMessages_Min, ubyte4 timeBetweenMessages_Max, bool required);

#endif // AVLTREE_H_INCLUDED