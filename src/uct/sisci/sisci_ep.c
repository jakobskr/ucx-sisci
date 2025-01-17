

#include "sisci_ep.h"
#include "sisci_iface.h"

static UCS_CLASS_CLEANUP_FUNC(uct_sci_ep_t)
{   
    sci_error_t sci_error;
    //printf("UCS_SICSCI_EP_CLEANUP_FUNC() %d \n", self->remote_segment_id);
    
    
    SCIUnmapSegment(self->remote_map, 0, &sci_error);
    
    self->buf = NULL;

    if (sci_error != SCI_ERR_OK) { 
        printf("SCI_UNMAP_SEGMENT: %s\n", SCIGetErrorString(sci_error));
    }

    /* Deprecated code
    SCIUnmapSegment(self->ctl_map, 0, &sci_error);
    self->sci_ctl = NULL;

    if (sci_error != SCI_ERR_OK) { 
        printf("SCI_UNMAP_CTL: %s\n", SCIGetErrorString(sci_error));
    }
    */

    SCIDisconnectSegment(self->remote_segment, 0, &sci_error);

    if (sci_error != SCI_ERR_OK) { 
        printf("SCI_DISCONNECT_SEGMENT: %s\n", SCIGetErrorString(sci_error));
    }
    
    DEBUG_PRINT("ep deleted segment_id %d node_id %d\n", self->remote_segment_id, self->remote_node_id);
}


static UCS_CLASS_INIT_FUNC(uct_sci_ep_t, const uct_ep_params_t *params) {

    sci_error_t sci_error;
    uct_sci_iface_addr_t* iface_addr =  (uct_sci_iface_addr_t*) params->iface_addr;
    uct_sci_device_addr_t* dev_addr = (uct_sci_device_addr_t*) params->dev_addr;
    sci_remote_data_interrupt_t req_interrupt;
    sci_local_data_interrupt_t  ans_interrupt;
    unsigned int local_interrupt_id =    ucs_generate_uuid(getpid());
    int ans_length          = sizeof(con_ans_t);
    conn_req_t request;
    con_ans_t answer;


    unsigned int segment_id = 0; //(unsigned int) params->segment_id;
    unsigned int node_id = 0; //(unsigned int) params->node_id;
    uct_sci_iface_t* iface = ucs_derived_of(params->iface, uct_sci_iface_t);
    uct_sci_md_t* md = ucs_derived_of(iface->super.md, uct_sci_md_t);


    UCT_EP_PARAMS_CHECK_DEV_IFACE_ADDRS(params);

    segment_id = (unsigned int) iface_addr->segment_id;
    node_id = (unsigned int) dev_addr->node_id;


    if(iface->segment_id == segment_id) {
        printf("%d connecting to a local segment %d\n", getpid(), segment_id);
    }

    DEBUG_PRINT("EP created segment_id %d node_id %d\n", segment_id, node_id);

    self->super.super.iface = params->iface;
    self->remote_segment_id = segment_id;
    self->remote_node_id = node_id;

    UCS_CLASS_CALL_SUPER_INIT(uct_base_ep_t, &iface->super); //segfaults without this line, probably has something to do with the stats member...

    do {
        SCIConnectDataInterrupt(md->sci_virtual_device, &req_interrupt, node_id, 0, segment_id, 0, 0, &sci_error);
    } while (sci_error != SCI_ERR_OK);

    //printf("%d connected to remote interrupt!, ret_int %d\n", getpid(),local_interrupt_id);
    //printf("size of answer %zd size of struct answer %zd\n", sizeof(answer), sizeof(con_ans_t));
    request.status     = 1;
    request.interrupt  = local_interrupt_id;
    request.node_id    = iface->device_addr;
    request.ctl_offset = iface->eps * sizeof(sci_ctl_t);
    request.ctl_id     = iface->ctl_id;

    SCICreateDataInterrupt(md->sci_virtual_device, &ans_interrupt, 0, &local_interrupt_id,  
                            NULL, NULL, SCI_FLAG_FIXED_INTNO, &sci_error);

    if(sci_error != SCI_ERR_OK) {
        printf("SCI Trigger Interrupt: %s\n", SCIGetErrorString(sci_error));
        return UCS_ERR_NO_RESOURCE;
    }            

    
    SCITriggerDataInterrupt(req_interrupt, (void *) &request, sizeof(request), SCI_NO_FLAGS, &sci_error);
    
    if(sci_error != SCI_ERR_OK) {
        printf("SCI Trigger Interrupt: %s\n", SCIGetErrorString(sci_error));
        return UCS_ERR_NO_RESOURCE;
    }

    //printf("%d sent interrupt of %zd to %d\n", getpid(), sizeof(request), segment_id);
              
    SCIWaitForDataInterrupt(ans_interrupt, (void*) &answer, &ans_length,SCI_INFINITE_TIMEOUT, 0, &sci_error);

    if(sci_error != SCI_ERR_OK) {
        printf("SCI Wait For Interrupt: %s\n", SCIGetErrorString(sci_error));
        return UCS_ERR_NO_RESOURCE;
    }        


    //printf("node %d segment %d\n", answer.node_id, answer.segment_id);


    self->remote_node_id    = answer.node_id;
    self->remote_segment_id = answer.segment_id;
    self->offset            = answer.offset;
    self->send_size         = answer.send_size;
    self->queue_size        = answer.queue_size;
    self->ctl_offset        = iface->eps * sizeof(sci_ctl_t);
    /* quick fix for weird behaviour when queue size was 1...*/
    self->seq               = self->queue_size > 1 ? 1 : 0;


    /*  Clean up for connection.  */
    SCIDisconnectDataInterrupt(req_interrupt, SCI_NO_FLAGS, &sci_error);

    if(sci_error == SCI_ERR_BUSY) {
        printf("SCIRemoveDataInterrupt: Interrupt still being used by another proccess");
    }

    SCIRemoveDataInterrupt(ans_interrupt, SCI_NO_FLAGS, &sci_error);

    if(sci_error == SCI_ERR_BUSY) {
        printf("SCIRemoveDataInterrupt: Interrupt still being used by another proccess");
    }

    do {
    DEBUG_PRINT("waiting to connect %d %s\n", sci_error,  SCIGetErrorString(sci_error));
    
    SCIConnectSegment(iface->vdev_ep, &self->remote_segment, self->remote_node_id, self->remote_segment_id, 
                ADAPTER_NO, NULL, NULL, 0, 0, &sci_error);

    } while (sci_error != SCI_ERR_OK);

    self->buf = (void *) SCIMapRemoteSegment(self->remote_segment, &self->remote_map, self->offset, iface->send_size * self->queue_size, NULL, 0, &sci_error);

    if (sci_error != SCI_ERR_OK) { 
        printf("SCI_MAP_REM_SEG: %s\n", SCIGetErrorString(sci_error));
        return UCS_ERR_NO_RESOURCE;
    }

    /*
    self->sci_ctl = (sci_ctl_t*) SCIMapLocalSegment(iface->ctl_segment, &self->ctl_map, sizeof(sci_ctl_t) * iface->eps, sizeof(sci_ctl_t), NULL, SCI_NO_FLAGS , &sci_error);

    if(sci_error != SCI_ERR_OK) {
        printf("SCI_MAP_CTL: %s\n", SCIGetErrorString(sci_error));
        return UCS_ERR_NO_RESOURCE;
    }*/

    iface->eps += 1;    
    DEBUG_PRINT("EP connected to segment %d at node %d\n",  self->remote_segment_id, self->remote_node_id);
    return UCS_OK;
}



UCS_CLASS_DEFINE(uct_sci_ep_t, uct_base_ep_t);

UCS_CLASS_DEFINE_NEW_FUNC(uct_sci_ep_t, uct_ep_t, const uct_ep_params_t *);
UCS_CLASS_DEFINE_DELETE_FUNC(uct_sci_ep_t, uct_ep_t);


/* //SECTION RDMA*/
ucs_status_t uct_sci_ep_put_short (uct_ep_h tl_ep, const void *buffer,
                                 unsigned length, uint64_t remote_addr,
                                 uct_rkey_t rkey)
{
    //TODO
    printf("uct_sci_ep_put_short()\n");
    return UCS_ERR_NOT_IMPLEMENTED;
}

ssize_t uct_sci_ep_put_bcopy(uct_ep_h tl_ep, uct_pack_callback_t pack_cb,
                            void *arg, uint64_t remote_addr, uct_rkey_t rkey)
{
    //TODO
    printf("uct_sci_ep_put_bcopy()\n");
    return UCS_ERR_NOT_IMPLEMENTED;
}

ucs_status_t uct_sci_ep_get_bcopy(uct_ep_h tl_ep, uct_unpack_callback_t unpack_cb,
                                 void *arg, size_t length,
                                 uint64_t remote_addr, uct_rkey_t rkey,
                                 uct_completion_t *comp)
{
    //TODO
    printf("uct_sci_ep_get_bcopy()\n");
    return UCS_ERR_NOT_IMPLEMENTED;
}


/*//!SECTION*/

/* //SECTION ATOMICS*/

ucs_status_t uct_sci_ep_atomic32_post(uct_ep_h ep, unsigned opcode, uint32_t value,
                                     uint64_t remote_addr, uct_rkey_t rkey)
{
    //TODO
    printf("uct_sci_ep_atomic32_post()\n");
    return UCS_ERR_NOT_IMPLEMENTED;
}

ucs_status_t uct_sci_ep_atomic64_post(uct_ep_h ep, unsigned opcode, uint64_t value,
                                     uint64_t remote_addr, uct_rkey_t rkey)
{
    //TODO
    printf("uct_sci_ep_atomic64_post()\n");
    return UCS_ERR_NOT_IMPLEMENTED;
}

ucs_status_t uct_sci_ep_atomic64_fetch(uct_ep_h ep, uct_atomic_op_t opcode,
                                      uint64_t value, uint64_t *result,
                                      uint64_t remote_addr, uct_rkey_t rkey,
                                      uct_completion_t *comp)
{
    //TODO
    printf("uct_sci_ep_atomic64_fetch()\n");
    return UCS_ERR_NOT_IMPLEMENTED;
}

ucs_status_t uct_sci_ep_atomic32_fetch(uct_ep_h ep, uct_atomic_op_t opcode,
                                      uint32_t value, uint32_t *result,
                                      uint64_t remote_addr, uct_rkey_t rkey,
                                      uct_completion_t *comp)
{
    //TODO
    printf("uct_sci_ep_atomic32_fetch()\n");
    return UCS_ERR_NOT_IMPLEMENTED;
}

ucs_status_t uct_sci_ep_atomic_cswap64(uct_ep_h tl_ep, uint64_t compare,
                                      uint64_t swap, uint64_t remote_addr,
                                      uct_rkey_t rkey, uint64_t *result,
                                      uct_completion_t *comp)
{
    //TODO
    printf("uct_sci_ep_atomic_cswap64()\n");
    return UCS_ERR_NOT_IMPLEMENTED;
}

ucs_status_t uct_sci_ep_atomic_cswap32(uct_ep_h tl_ep, uint32_t compare,
                                      uint32_t swap, uint64_t remote_addr,
                                      uct_rkey_t rkey, uint32_t *result,
                                      uct_completion_t *comp)
{
    //TODO
    printf("uct_sci_ep_atomic_cswap32()\n");
    return UCS_ERR_NOT_IMPLEMENTED;
}

/* //!SECTION */

/*  // SECTION Active messages */

ucs_status_t uct_sci_ep_am_short(uct_ep_h tl_ep, uint8_t id, uint64_t header,
                                  const void *payload, unsigned length)
{

    uct_sci_ep_t* ep       = ucs_derived_of(tl_ep, uct_sci_ep_t);
    sci_packet_t* packet; 
    uct_sci_iface_t* iface = ucs_derived_of(tl_ep->iface, uct_sci_iface_t);
    sci_ctl_t* ctl         = iface->ctls + ep->ctl_offset;
    uint32_t offset = 0; 
    
    if (ep->seq - ctl->ack >= iface->queue_size) {
        return UCS_ERR_NO_RESOURCE;
    }
        
    offset = ep->send_size * (ep->seq % ep->queue_size);
    packet = ep->buf + offset;
    ctl->status = 1;
    packet->am_id = id;
    packet->length = length + sizeof(header);
    uct_am_short_fill_data(ep->buf + offset + sizeof(sci_packet_t), header, payload, length);
    SCIFlush(NULL, SCI_FLAG_FLUSH_CPU_BUFFERS_ONLY);    
    packet->status = 1;
    SCIFlush(NULL, SCI_FLAG_FLUSH_CPU_BUFFERS_ONLY);
    ep->seq++;
    DEBUG_PRINT("EP_SEG %d EP_NOD %d AM_ID %d size %d SEQ:%d\n", ep->remote_segment_id, ep->remote_node_id, id, packet->length, ep->seq);
    return UCS_OK;
}

ucs_status_t uct_sci_ep_am_short_iov(uct_ep_h tl_ep, uint8_t id,
                                      const uct_iov_t *iov, size_t iovcnt)
{
    //TODO short_iov
    printf("uct_sci_ep_am_short_iov()\n");
    return UCS_ERR_NOT_IMPLEMENTED;
}

ssize_t uct_sci_ep_am_bcopy(uct_ep_h tl_ep, uint8_t id,
                             uct_pack_callback_t pack_cb, void *arg,
                             unsigned flags)
{
    //TODO bcopy    
    uct_sci_ep_t*    ep    = ucs_derived_of(tl_ep, uct_sci_ep_t);
    sci_packet_t*  packet;
    uct_sci_iface_t* iface = ucs_derived_of(tl_ep->iface, uct_sci_iface_t);
    sci_ctl_t* ctl         = iface->ctls + ep->ctl_offset;
    ssize_t length         = 0;
    uint32_t offset        = 0;

    if(ep->seq - ctl->ack >= iface->queue_size) {
        return UCS_ERR_NO_RESOURCE;
    }


    offset = ep->send_size * (ep->seq % ep->queue_size);
    packet = ep->buf + offset;

    ctl->status = 1;
    length              = pack_cb((void*) packet + sizeof(sci_packet_t),  arg);
    packet->am_id       = id;
    packet->length      = length;
    SCIFlush(NULL, SCI_FLAG_FLUSH_CPU_BUFFERS_ONLY);

    packet->status = 1;
    SCIFlush(NULL, SCI_FLAG_FLUSH_CPU_BUFFERS_ONLY);
    ep->seq++;

    DEBUG_PRINT("EP_SEG %d EP_NOD %d AM_ID %d size %d \n", ep->remote_segment_id, ep->remote_node_id, id, packet->length);

    return length;
}

ucs_status_t uct_sci_ep_am_zcopy(uct_ep_h uct_ep, uint8_t id, const void *header, unsigned header_length, 
                            const uct_iov_t *iov, size_t iovcnt, unsigned flags, uct_completion_t *comp) 
{

    uct_sci_ep_t* ep            = ucs_derived_of(uct_ep, uct_sci_ep_t);
    uct_sci_iface_t* iface      = ucs_derived_of(uct_ep->iface, uct_sci_iface_t);
    sci_packet_t* sci_header; 
    void* tx                    = iface->dma_buf;
    sci_packet_t* tx_pack       = (sci_packet_t*) tx;
    size_t iov_total_len        = uct_iov_total_length(iov, iovcnt);
    sci_ctl_t* ctl              = iface->ctls + ep->ctl_offset;
    size_t bytes_copied;
    ucs_iov_iter_t uct_iov_iter;
    sci_error_t sci_error;
    uint32_t offset;


    if(ep->seq - ctl->ack >= iface->queue_size) {
        return UCS_ERR_NO_RESOURCE;
    }

    //printf("1 %zd\n", iface->send_size);

    ctl->status = 1;

    offset = ep->send_size * (ep->seq % ep->queue_size);
    
    
    sci_header = ep->buf + offset;


    UCT_CHECK_LENGTH(header_length + iov_total_len + sizeof(sci_packet_t), 0 , iface->send_size, "am_zcopy");
    UCT_CHECK_AM_ID(id);
    /* Convert the iov into a contiguous buffer */
    ucs_iov_iter_init(&uct_iov_iter);

    //printf("2.2\n");

    bytes_copied = uct_iov_to_buffer(iov, iovcnt, &uct_iov_iter, tx + sizeof(sci_packet_t) + header_length, iface->send_size);

    if(bytes_copied != iov_total_len) {
        /* Might wanna replace this with an assert */
        printf("PANIK\n");
    }

    //printf("2.5\n");

    /* Set header values */
    tx_pack->am_id = id;
    tx_pack->length = iov_total_len + header_length;

        //printf("3\n");


    if (header_length != 0)
    {
        memcpy(tx + sizeof(sci_packet_t), header, header_length);
    }
    
    SCIStartDmaTransfer(iface->dma_queue, iface->dma_segment, ep->remote_segment, 
                        0, iov_total_len + header_length + SCI_PACKET_SIZE, offset,
                        SCI_NO_CALLBACK, NULL, SCI_NO_FLAGS, &sci_error);
    

    if(sci_error != SCI_ERR_OK) {
        printf("DMA Transfer Error: %s\n", SCIGetErrorString(sci_error));
    }

    //printf("4\n");


    //SCIWaitForDMAQueue(iface->dma_queue, SCI_INFINITE_TIMEOUT, SCI_NO_FLAGS, &sci_error);
    ep->seq++;
    sci_header->status = 1;
    SCIFlush(NULL, SCI_FLAG_FLUSH_CPU_BUFFERS_ONLY);


    DEBUG_PRINT("EP_SEG %d EP_NOD %d AM_ID %d size %d \n", ep->remote_segment_id, ep->remote_node_id, id, sci_header->length);

    return UCS_OK;    
}

/* //!SECTION*/

