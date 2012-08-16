#if !defined(_RMSGQ_H_)
#define _RMSGQ_H_


#define MAX_NUM_MSG      8 /** must be pow(2,n) */
#define MAX_SIZE_MSG  0x40L
#define RMSGQ_TRUE       1
#define RMSGQ_FALSE      0
#define RMSGQ_ERR       -1
#define RMSGQ_SUCCESS    0

typedef struct {
    unsigned long padding[16];
}RMSGQ_MSG;

typedef struct {
    volatile unsigned short readIdx;
    volatile unsigned short writeIdx;
    RMSGQ_MSG cmdArray[MAX_NUM_MSG];
} RING_MSGQ;

short RMSGQ_init (void *rqueue);
short RMSGQ_get (void *rqueue, void *cmd, unsigned short size);
short RMSGQ_put (void *rqueue, void *cmd, unsigned short size);
short RMSGQ_isFull (void *rqueue);
short RMSGQ_isEmpty (void *rqueue);
short RMSGQ_clear (void *rqueue);

#endif /** _LINK_RMSGQ_H_ */
