//http://www.zentut.com/c-tutorial/c-avl-tree/

#ifndef AVLTREE_H_INCLUDED
#define AVLTREE_H_INCLUDED

#include "IO_Driver.h"

typedef struct AVLNode
{
    //Message Metadata -----------------------------------------------------
    //int data;
    //ubyte4 id;           /**< ID for CAN communication             */
    ubyte1 data[8];

    ubyte4 timeBetweenMessages_Min; //Fastest rate at which messages will be sent
    ubyte4 lastMessage_timeStamp;   //Last time message was sent/received

    bool required;
    ubyte4 timeBetweenMessages_Max; //Slowest rate at which messages will be sent, OR max time between receiving messages before throwing an error

    //Tree stuff -----------------------------------------------------
    //struct AVLNode*  left;
    //struct AVLNode*  right;
    //int      height;
} AVLNode;

//Note on passing arrays: http://stackoverflow.com/questions/5573310/difference-between-passing-array-and-array-pointer-into-function-in-c
AVLNode *AVL_insert(AVLNode **t, ubyte4 messageID, ubyte1 messageData[8], ubyte4 timeBetweenMessages_Min, ubyte4 timeBetweenMessages_Max, bool required);
//////////////////AVLNode* AVL_find(AVLNode *t, ubyte4 messageID);
//int AVL_getData(AVLNode* n);
//AVLNode* AVL_findMin(AVLNode *t);
//AVLNode* AVL_findMax(AVLNode *t);
//void AVL_display(AVLNode* t);
//void AVL_dispose(AVLNode* t);
//AVLNode* AVL_delete(int id, AVLNode *t);

/*
//C AVL tree program main.c
#include <stdio.h>
#include "avltree.h"

int main()
{
AVLNode *t , *p;
int i;
int j = 0;
const int max = 10;

printf("--- C AVL Tree Demo  ---\n");

t = NULL;

printf("Insert: ");
for( i = 0; i < max; i++, j = ( j + 7 ) % max )
{

t = insert( j, t );
printf("%d ",j);

}
printf(" into the tree\n\n");

display_avl(t);

dispose(t);

return 0;
}
*/

#endif // AVLTREE_H_INCLUDED