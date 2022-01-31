

#include "sisci_ep.h"
#include "sisci_iface.h"

static UCS_CLASS_CLEANUP_FUNC(uct_sci_ep_t)
{   
    sci_error_t sci_error;
    printf("UCS_SICSCI_EP_CLEANUP_FUNC() %d \n", self->remote_segment_id);
    

    //TODO: Find out why this code causes a segfault... When in running in devel mode. Something with how allocates the maps.
    
    SCIUnmapSegment(self->remote_map, 0, &sci_error);
    
    self->buf = NULL;

    if (sci_error != SCI_ERR_OK) { 
        printf("SCI_UNMAP_SEGMENT: %s\n", SCIGetErrorString(sci_error));
    }

    


    SCIDisconnectSegment(self->remote_segment, 0, &sci_error);

    if (sci_error != SCI_ERR_OK) { 
        printf("SCI_DISCONNECT_SEGMENT: %s\n", SCIGetErrorString(sci_error));
    }


    
    printf("EP_DELETED : )\n");
}


static UCS_CLASS_INIT_FUNC(uct_sci_ep_t, const uct_ep_params_t *params)
{

    sci_error_t sci_error;
    uct_sci_iface_addr_t* iface_addr =  (uct_sci_iface_addr_t*) params->iface_addr;
    uct_sci_device_addr_t* dev_addr = (uct_sci_device_addr_t*) params->dev_addr;

    unsigned int segment_id = 0; //(unsigned int) params->segment_id;
    unsigned int node_id = 0; //(unsigned int) params->node_id;
    uct_sci_iface_t* iface = ucs_derived_of(params->iface, uct_sci_iface_t);
    uct_sci_md_t* md = ucs_derived_of(iface->super.md, uct_sci_md_t);


    UCT_EP_PARAMS_CHECK_DEV_IFACE_ADDRS(params);

    segment_id = (unsigned int) iface_addr->segment_id;
    node_id = (unsigned int) dev_addr->node_id;

    printf("create_ep: nodeID: %d segID: %d\n", segment_id, node_id);
    self->super.super.iface = params->iface;
    self->remote_segment_id = segment_id;
    self->remote_node_id = node_id;

    UCS_CLASS_CALL_SUPER_INIT(uct_base_ep_t, &iface->super); //segfaults without this line, probably has something to do with the stats member...


    do {
    SCIConnectSegment(md->sci_virtual_device, &self->remote_segment, self->remote_node_id, self->remote_segment_id, 
                ADAPTER_NO, NULL, NULL, 0, 0, &sci_error);

    printf("waiting to connect\n");
  } while (sci_error != SCI_ERR_OK);
    

    self->buf = (void *) SCIMapRemoteSegment(self->remote_segment, &self->remote_map, 0, iface->send_size, NULL, 0, &sci_error);

    if (sci_error != SCI_ERR_OK) { 
        printf("SCI_MAP_REM_SEG: %s\n", SCIGetErrorString(sci_error));
        return UCS_ERR_NO_RESOURCE;
    }
    
    printf("EP connected to %d %d\n", self->remote_node_id, self->remote_segment_id);
    return UCS_OK;
}

UCS_CLASS_DEFINE(uct_sci_ep_t, uct_base_ep_t);
UCS_CLASS_DEFINE_NEW_FUNC(uct_sci_ep_t, uct_ep_t, const uct_ep_params_t *);
UCS_CLASS_DEFINE_DELETE_FUNC(uct_sci_ep_t, uct_ep_t);

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

//from sm self.c

ucs_status_t uct_sci_ep_am_short(uct_ep_h tl_ep, uint8_t id, uint64_t header,
                                  const void *payload, unsigned length)
{
    //TODO
    uct_sci_ep_t* ep = ucs_derived_of(tl_ep, uct_sci_ep_t);
    sisci_packet_t* packet = ep->buf; 
    //char* test = (char*) payload;
    
    //uint* tmp = (uint* ) ep->buf;
    //void * map = (void *) SCIGetMapPointer(ep->remote_map);


    printf("sizeof adress %zd sizeof unsigned %zd size of uint %zd size of void %zd\n", sizeof(uct_sicsci_ep_addr_t),sizeof(length), sizeof(uint), sizeof(void*));
    packet->am_id = id;
    packet->length = length + sizeof(header);
    //memcpy(packet->data, payload, length);
    uct_am_short_fill_data(ep->buf + sizeof(sisci_packet_t), header, payload, length);
    //memcpy(ep->buf + sizeof(sisci_packet_t), payload, length);
    SCIFlush(NULL, SCI_NO_FLAGS);    
    packet->status = 1;
    SCIFlush(NULL, SCI_NO_FLAGS);

    printf("uct_sci_ep_am_short() %d %ld %d \n", id, header, length);
    
    return UCS_OK;
}

ucs_status_t uct_sci_ep_am_short_iov(uct_ep_h tl_ep, uint8_t id,
                                      const uct_iov_t *iov, size_t iovcnt)
{
    //TODO
    printf("uct_sci_ep_am_short_iov()\n");
    return UCS_ERR_NOT_IMPLEMENTED;
}

ssize_t uct_sci_ep_am_bcopy(uct_ep_h tl_ep, uint8_t id,
                             uct_pack_callback_t pack_cb, void *arg,
                             unsigned flags)
{
    //TODO
    printf("uct_sci_ep_am_bcopy()\n");
    return -8;
}

ucs_status_t uct_sci_ep_am_zcopy(uct_ep_h ep, uint8_t id, const void *header, unsigned header_length, 
                            const uct_iov_t *iov, size_t iovcnt, unsigned flags, uct_completion_t *comp) 
{
    printf("uct_sci_ep_am_zcopy()\n");
    return UCS_ERR_NOT_IMPLEMENTED;;    
}


