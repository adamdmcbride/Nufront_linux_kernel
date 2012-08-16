#include "rmsgq.h"
#include <linux/kernel.h>
#include <linux/mm.h>
#include <asm/io.h>

short RMSGQ_init(void *rqueue)
{
    RING_MSGQ *pQueue = (RING_MSGQ*)rqueue;
    *((volatile unsigned long*)pQueue) = 0;
    return RMSGQ_ERR;
}

short RMSGQ_clear(void *rqueue)
{
    RING_MSGQ *pQueue = (RING_MSGQ*)rqueue;
    pQueue->readIdx = 0;
    pQueue->writeIdx = 0;
    return RMSGQ_SUCCESS;
}

short RMSGQ_get(void *rqueue, void *cmd, unsigned short size)
{
    RING_MSGQ *pQueue = (RING_MSGQ*)rqueue;
    unsigned short ri = pQueue->readIdx;
    if(!RMSGQ_isEmpty(rqueue))
    {
        memcpy(cmd, &pQueue->cmdArray[ri], size);
        ri = (ri + 1)&(MAX_NUM_MSG-1);
        pQueue->readIdx = ri;
        return RMSGQ_SUCCESS;
    }
    return RMSGQ_ERR;
}


short RMSGQ_put(void *rqueue, void *cmd, unsigned short size)
{
    RING_MSGQ *pQueue = (RING_MSGQ*)rqueue;
    unsigned short wi = pQueue->writeIdx;
    if(!RMSGQ_isFull(rqueue))
    {
        memcpy(&pQueue->cmdArray[wi], cmd, size);

        wi = (wi + 1)&(MAX_NUM_MSG-1);
        pQueue->writeIdx = wi;

        return RMSGQ_SUCCESS;
    }
    return RMSGQ_ERR;
}

short RMSGQ_isFull(void *rqueue)
{
    RING_MSGQ *pQueue = (RING_MSGQ*)rqueue;
    unsigned short ri = pQueue->readIdx;
    unsigned short wi = pQueue->writeIdx;
    if(((wi+1)&(MAX_NUM_MSG-1)) == ri)
        return RMSGQ_TRUE;
    return RMSGQ_FALSE;
}

short RMSGQ_isEmpty(void *rqueue)
{
    RING_MSGQ *pQueue = (RING_MSGQ*)rqueue;
    unsigned short ri = pQueue->readIdx;
    unsigned short wi = pQueue->writeIdx;
    if(wi == ri)
        return RMSGQ_TRUE;
    return RMSGQ_FALSE;
}



