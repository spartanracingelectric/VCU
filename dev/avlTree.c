//http://www.zentut.com/c-tutorial/c-avl-tree/

#include <string.h> //memcpy
#include <stdlib.h> //malloc

#include "IO_RTC.h"
#include "IO_Driver.h"
#include "mathFunctions.h"
#include "avlTree.h"

AVLNode *AVL_insert(AVLNode **messageHistoryArray, ubyte4 messageID, ubyte1 messageData[8], ubyte4 minTime, ubyte4 maxTime, bool req)
{
    //This function has been hijacked for an emergency quick fix

    AVLNode *message = (AVLNode *)malloc(sizeof(AVLNode));
    if (message == NULL) //malloc failed
    {
        //fprintf(stderr, "Out of memory!!! (insert)\n");
        //exit(1);
    }
    else
    {
        //message->id = messageID;
        message->timeBetweenMessages_Min = minTime;
        message->timeBetweenMessages_Max = maxTime;
        IO_RTC_StartTime(&message->lastMessage_timeStamp);

        //To copy an entire array, http://stackoverflow.com/questions/9262784/array-equal-another-array
        memcpy(messageData, message->data, sizeof(messageData));

        message->required = req;

        messageHistoryArray[messageID] = message;
    }
    return message;

}
