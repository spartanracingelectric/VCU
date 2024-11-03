// http://www.zentut.com/c-tutorial/c-avl-tree/

#include <stdlib.h> //malloc
#include <string.h> //memcpy

#include "avlTree.h"
#include "IO_Driver.h"
#include "IO_RTC.h"
#include "mathFunctions.h"

//-------------------------------------------------------------------
// Private functions
//-------------------------------------------------------------------

///////////////////////*
//////////////////////get the height of a node
//////////////////////*/
//////////////////////static int AVL_getHeight(AVLNode* n)
//////////////////////{
//////////////////////    if (n == NULL)
//////////////////////    {
//////////////////////        return -1;
//////////////////////    }
//////////////////////    else
//////////////////////    {
//////////////////////        return n->height;
//////////////////////    }
//////////////////////}
//////////////////////
///////////////////////*
//////////////////////get maximum value of two integers
//////////////////////static int max(int l, int r)
////////////////////////Moved to mathFunctions.c
//////////////////////{
//////////////////////return l > r ? l : r;
//////////////////////}
//////////////////////*/
//////////////////////
///////////////////////*
//////////////////////perform a rotation between a k2 node and its left child
//////////////////////
//////////////////////note: call AVL_singleRotateWithLeft only if k2 node has a left child
//////////////////////*/
//////////////////////
//////////////////////static AVLNode* AVL_singleRotateWithLeft(AVLNode* k2)
//////////////////////{
//////////////////////    AVLNode* k1 = NULL;
//////////////////////
//////////////////////    k1 = k2->left;
//////////////////////    k2->left = k1->right;
//////////////////////    k1->right = k2;
//////////////////////
//////////////////////    k2->height = max(AVL_getHeight(k2->left), AVL_getHeight(k2->right)) + 1;
//////////////////////    k1->height = max(AVL_getHeight(k1->left), k2->height) + 1;
//////////////////////    return k1; /* new root */
//////////////////////}
//////////////////////
///////////////////////*
//////////////////////perform a rotation between a node (k1) and its right child
//////////////////////
//////////////////////note: call AVL_singleRotateWithRight only if
//////////////////////the k1 node has a right child
//////////////////////*/
//////////////////////
//////////////////////static AVLNode* AVL_singleRotateWithRight(AVLNode* k1)
//////////////////////{
//////////////////////    AVLNode* k2;
//////////////////////
//////////////////////    k2 = k1->right;
//////////////////////    k1->right = k2->left;
//////////////////////    k2->left = k1;
//////////////////////
//////////////////////    k1->height = max(AVL_getHeight(k1->left), AVL_getHeight(k1->right)) + 1;
//////////////////////    k2->height = max(AVL_getHeight(k2->right), k1->height) + 1;
//////////////////////
//////////////////////    return k2;  /* New root */
//////////////////////}
//////////////////////
///////////////////////*
//////////////////////
//////////////////////perform the left-right double rotation,
//////////////////////
//////////////////////note: call AVL_doubleRotateWithLeft only if k3 node has
//////////////////////a left child and k3's left child has a right child
//////////////////////*/
//////////////////////
//////////////////////static AVLNode* AVL_doubleRotateWithLeft(AVLNode* k3)
//////////////////////{
//////////////////////    /* Rotate between k1 and k2 */
//////////////////////    k3->left = AVL_singleRotateWithRight(k3->left);
//////////////////////
//////////////////////    /* Rotate between K3 and k2 */
//////////////////////    return AVL_singleRotateWithLeft(k3);
//////////////////////}
//////////////////////
///////////////////////*
//////////////////////perform the right-left double rotation
//////////////////////
//////////////////////notes: call AVL_doubleRotateWithRight only if k1 has a
//////////////////////right child and k1's right child has a left child
//////////////////////*/
//////////////////////
//////////////////////
//////////////////////
//////////////////////static AVLNode* AVL_doubleRotateWithRight(AVLNode* k1)
//////////////////////{
//////////////////////    /* rotate between K3 and k2 */
//////////////////////    k1->right = AVL_singleRotateWithLeft(k1->right);
//////////////////////
//////////////////////    /* rotate between k1 and k2 */
//////////////////////    return AVL_singleRotateWithRight(k1);
//////////////////////}

/*
Recursively display AVL tree or subtree
void display_avl(AVLNode* t)
{
if (t == NULL)
return;
printf("%d", t->id);

if (t->left != NULL)
printf("(L:%d)", t->left->id);
if (t->right != NULL)
printf("(R:%d)", t->right->id);
printf("\n");

display_avl(t->left);
display_avl(t->right);
}
*/

/*
remove all nodes of an AVL tree
void dispose(AVLNode* t)
{
if (t != NULL)
{
dispose(t->left);
dispose(t->right);
free(t);
}
}
*/

/*
remove a node in the tree
AVLNode* delete(int e, AVLNode* t)
{
printf("Sorry; Delete is unimplemented; %d remains\n", e);
return t;
}
*/
/*
struct AVLNode* deleteNode(struct AVLNode* root, int key)
{
// STEP 1: PERFORM STANDARD BST DELETE

if (root == NULL)
return root;

// If the key to be deleted is smaller than the root's key,
// then it lies in left subtree
if (key < root->key)
root->left = deleteNode(root->left, key);

// If the key to be deleted is greater than the root's key,
// then it lies in right subtree
else if (key > root->key)
root->right = deleteNode(root->right, key);

// if key is same as root's key, then This is the node
// to be deleted
else
{
// node with only one child or no child
if ((root->left == NULL) || (root->right == NULL))
{
struct AVLNode *temp = root->left ? root->left : root->right;

// No child case
if (temp == NULL)
{
temp = root;
root = NULL;
}
else // One child case
*root = *temp; // Copy the contents of the non-empty child

free(temp);
}
else
{
// node with two children: Get the inorder successor (smallest
// in the right subtree)
struct AVLNode* temp = minValueNode(root->right);

// Copy the inorder successor's data to this node
root->key = temp->key;

// Delete the inorder successor
root->right = deleteNode(root->right, temp->key);
}
}

// If the tree had only one node then return
if (root == NULL)
return root;

// STEP 2: UPDATE HEIGHT OF THE CURRENT NODE
root->height = max(AVL_getHeight(root->left), AVL_getHeight(root->right)) + 1;

// STEP 3: GET THE BALANCE FACTOR OF THIS NODE (to check whether
//  this node became unbalanced)
int balance = getBalance(root);

// If this node becomes unbalanced, then there are 4 cases

// Left Left Case
if (balance > 1 && getBalance(root->left) >= 0)
return rightRotate(root);

// Left Right Case
if (balance > 1 && getBalance(root->left) < 0)
{
root->left = leftRotate(root->left);
return rightRotate(root);
}

// Right Right Case
if (balance < -1 && getBalance(root->right) <= 0)
return leftRotate(root);

// Right Left Case
if (balance < -1 && getBalance(root->right) > 0)
{
root->right = rightRotate(root->right);
return leftRotate(root);
}

return root;
}
*/

//-------------------------------------------------------------------
// Public functions
//-------------------------------------------------------------------

/*
insert a new node into the tree
*/
// AVLNode* AVL_insert(AVLNode* t, ubyte4 messageID, ubyte1 messageData[8], ubyte4 minTime, ubyte4 maxTime, bool req)
AVLNode *AVL_insert(AVLNode **messageHistoryArray, ubyte4 messageID, ubyte1 messageData[8], ubyte4 minTime,
                    ubyte4 maxTime, bool req)
{
    // This function has been hijacked for an emergency quick fix

    AVLNode *message = (AVLNode *) malloc(sizeof(AVLNode));
    if (message == NULL) // malloc failed
    {
        // fprintf(stderr, "Out of memory!!! (insert)\n");
        // exit(1);
    }
    else
    {
        // message->id = messageID;
        message->timeBetweenMessages_Min = minTime;
        message->timeBetweenMessages_Max = maxTime;
        IO_RTC_StartTime(&message->lastMessage_timeStamp);

        // To copy an entire array, http://stackoverflow.com/questions/9262784/array-equal-another-array
        memcpy(messageData, message->data, sizeof(messageData));

        message->required = req;

        messageHistoryArray[messageID] = message;
    }
    return message;

    ////ACTUAL AVL INSERT CODE BELOW
    // if (t == NULL) //If the tree is empty
    //{
    //     /* Create and return a one-node tree */
    //     t = (AVLNode*)malloc(sizeof(AVLNode));
    //     if (t == NULL)
    //     {
    //         //fprintf(stderr, "Out of memory!!! (insert)\n");
    //         //exit(1);
    //     }
    //     else
    //     {
    //         t->id = messageID;
    //         t->timeBetweenMessages_Min = minTime;
    //         t->timeBetweenMessages_Max = maxTime;
    //         //To copy an entire array, http://stackoverflow.com/questions/9262784/array-equal-another-array
    //         memcpy(messageData, t->data, sizeof(messageData));
    //         IO_RTC_StartTime(&t->lastMessage_timeStamp);

    //        t->height = 0;
    //        t->left = t->right = NULL;
    //    }
    //}
    // else if (messageID < t->id)
    //{
    //    t->left = AVL_insert(t->left, messageID, messageData, minTime, maxTime, req);
    //    if (AVL_getHeight(t->left) - AVL_getHeight(t->right) == 2)
    //    {
    //        if (messageID < t->left->id)
    //        {
    //            t = AVL_singleRotateWithLeft(t);
    //        }
    //        else
    //        {
    //            t = AVL_doubleRotateWithLeft(t);
    //        }
    //    }
    //}
    // else if (messageID > t->id)
    //{
    //    t->right = AVL_insert(t->right, messageID, messageData, minTime, maxTime, req);
    //    if (AVL_getHeight(t->right) - AVL_getHeight(t->left) == 2)
    //    {
    //        if (messageID > t->right->id)
    //        {
    //            t = AVL_singleRotateWithRight(t);
    //        }
    //        else
    //        {
    //            t = AVL_doubleRotateWithRight(t);
    //        }
    //    }
    //}
    ///* Else X is in the tree already; we'll do nothing */

    // t->height = max(AVL_getHeight(t->left), AVL_getHeight(t->right)) + 1;
    // return t;
    // return message;
}

///////////////////////////*
//////////////////////////find a specific node's key in the tree
//////////////////////////*/
//////////////////////////AVLNode* AVL_find(AVLNode* t, ubyte4 messageID)
//////////////////////////{
//////////////////////////    if (t == NULL)
//////////////////////////    {
//////////////////////////        return NULL;
//////////////////////////    }
//////////////////////////    if (messageID < t->id)
//////////////////////////    {
//////////////////////////        return AVL_find(t->left, messageID);
//////////////////////////    }
//////////////////////////    else if (messageID > t->id)
//////////////////////////    {
//////////////////////////        return AVL_find(t->right, messageID);
//////////////////////////    }
//////////////////////////    else
//////////////////////////    {
//////////////////////////        return t;
//////////////////////////    }
//////////////////////////}

///*
// data data of a node
//*/
// int AVL_getData(AVLNode* n)
//{
//	return n->id;
// }
//
///*
// find minimum node's key
//*/
// AVLNode* AVL_findMin(AVLNode* t)
//{
//     if (t == NULL)
//         return NULL;
//     else if (t->left == NULL)
//         return t;
//     else
//         return AVL_findMin(t->left);
// }
//
///*
// find maximum node's key
//*/
// AVLNode* AVL_findMax(AVLNode* t)
//{
//     if (t != NULL)
//         while (t->right != NULL)
//             t = t->right;
//
//     return t;
// }
