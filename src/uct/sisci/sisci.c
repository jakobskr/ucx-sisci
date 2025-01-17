
#include <ucs/type/class.h>
#include <ucs/type/status.h>
#include <ucs/sys/string.h>

#include "stdio.h"

#include "sisci.h"
#include "sisci_ep.h"
#include "sisci_iface.h" //TODO, is this needed?
//#include "sci_iface.c"


/* Forward declarations */
static uct_iface_ops_t uct_sci_iface_ops;
static uct_component_t uct_sci_component;



static ucs_config_field_t uct_sci_iface_config_table[] = {
    {"", "MAX_NUM_EPS=16", NULL,
     ucs_offsetof(uct_sci_iface_config_t, super),
     UCS_CONFIG_TYPE_TABLE(uct_iface_config_table)},

    {"SEND_SIZE", "16k",
     "Size of copy-out buffer",
     ucs_offsetof(uct_sci_iface_config_t, send_size), UCS_CONFIG_TYPE_MEMUNITS},

    {
        "MAX_EPS", "24", "Max EPs for SCI tl",
        ucs_offsetof(uct_sci_iface_config_t, max_eps),
        UCS_CONFIG_TYPE_UINT
    },

    {
        "QUEUE_SIZE", "5", "Message Queue size for each connection",
        ucs_offsetof(uct_sci_iface_config_t, queue_size),
        UCS_CONFIG_TYPE_UINT
    },

    {NULL}
};


/*Forward declaration of the md config table*/
/*static ucs_config_field_t uct_sci_md_config_table[] = {
    NULL
};*/

/**
 * @brief This function handles incoming connection requests and assigns a sci file descriptor to that connection.
 * Then it replies with information back to the connecting request to enable the incoming conneciton to connect and offset correctly
 * into the iface's recv buffer. The iface also connects to the connectors control block, so we can signal it when we are ready to recv data.
 * 
 * @param arg iface 
 * @param interrupt which interrupt was triggered
 * @param data: Information received from the connecting process
 * @param length: length of data
 * @param sci_error: Not used
 * @return sci_callback_action_t: Returns callback_continue  
 */
sci_callback_action_t conn_handler(void* arg, sci_local_data_interrupt_t interrupt, void* data, unsigned int length, sci_error_t sci_error) {
    sci_remote_data_interrupt_t ans_interrupt;
    //sci_error_t sci_error;
    con_ans_t   answer;
    conn_req_t* request = (conn_req_t*) data;
    uct_sci_iface_t* iface = (uct_sci_iface_t*) arg;
    uct_sci_md_t* md = ucs_derived_of(iface->super.md, uct_sci_md_t); 
    size_t i;

    //printf("%d expected %zd ret_int %d  ret_node %d \n", length, sizeof(conn_req_t), request->node_id, request->interrupt);
    //printf("%d callback started \n", getpid());


    do {
        SCIConnectDataInterrupt(md->sci_virtual_device, &ans_interrupt, request->node_id, 0, request->interrupt, 1000, 0, &sci_error);
        //printf("waiting to connect to %d\n", request->interrupt);
    } while (sci_error != SCI_ERR_OK);

    //printf("connected to answer request\n");
    
    //todo add spin lock:

    /*   Enter critical   */

    pthread_mutex_lock(&iface->lock);

    for (i = 0; i < iface->max_eps; i++)
    {
        if(iface->sci_cds[i].status == 0) {
            iface->sci_cds[i].status = 2;
            break;
        }
    }

    iface->connections++;

    /*  leave critical  */
    pthread_mutex_unlock(&iface->lock);
    answer.node_id    = iface->device_addr;
    answer.segment_id = iface->segment_id;
    answer.offset     = iface->sci_cds[i].offset;
    answer.send_size  = iface->send_size;
    answer.queue_size = iface->queue_size;

    SCITriggerDataInterrupt(ans_interrupt, (void *) &answer, sizeof(answer), SCI_NO_FLAGS, &sci_error);

    if(sci_error != SCI_ERR_OK) {
        printf("SCI Trigger Interrupt: %s/n", SCIGetErrorString(sci_error));
    }    

    /*  set status to ready  */ 

    do {
        DEBUG_PRINT("waiting to connect to ctl %s\n", SCIGetErrorString(sci_error));
        SCIConnectSegment(iface->vdev_ctl, &iface->sci_cds[i].ctl_segment, request->node_id, request->ctl_id, 
                ADAPTER_NO, NULL, NULL, 0, 0, &sci_error);
        
    } while (sci_error != SCI_ERR_OK);


    //printf("cd ctl offset %d \n", request->ctl_offset);
    iface->sci_cds[i].ctl_buf = (sci_ctl_t *) SCIMapRemoteSegment(iface->sci_cds[i].ctl_segment, &iface->sci_cds[i].ctl_map, request->ctl_offset, 
                                                                  sizeof(sci_ctl_t), NULL, 0, &sci_error);

    if (sci_error != SCI_ERR_OK) { 
        printf("SCI_MAP_REM_SEG: %s\n", SCIGetErrorString(sci_error));
        return UCS_ERR_NO_RESOURCE;
    }

    iface->sci_cds[i].status = 1;
    /* NOTE: does not return any error messages of any kind */
    SCIDisconnectDataInterrupt(ans_interrupt, SCI_NO_FLAGS, &sci_error);

   //printf("%d callback done \n", getpid());
    return SCI_CALLBACK_CONTINUE;
}

int sci_opened = 0;
int iface_query_printed = 0;

/*
    The linux version initialization of the sci api doesnt do much except for comparing the api version against the adapter version, and setting up some ref handles
    So we dont really need handle anything special except for the initialization faliing :). 
    Since the api doesn't have any good way to check if the driver is initialized we have to keep track of it ourselves. 
*/
static unsigned int uct_sci_open(){
    sci_error_t sci_error = 0;

    DEBUG_PRINT("sci_open(%d)\n", sci_opened);
    if (sci_opened == 0)
    {
        SCIInitialize(0,&sci_error);
        if (sci_error != SCI_ERR_OK)
        {
            printf("sci_init error: %s/n", SCIGetErrorString(sci_error));
            return 0;
        }
        sci_opened = 1;

    }
    return 1;
}

/*
    Closing the api is even more hands off than 
*/
static unsigned int uct_sci_close(){
    DEBUG_PRINT("sci_close(%d)\n", sci_opened);
    if (sci_opened == 1)
    {
        SCITerminate();
        sci_opened = 0;
    }
    return 1;    
}

//also known as "macro hell"
/**
 * @brief Construct a new ucs class init func object
 * 
 * @param md 
 * @param worker 
 * @param params 
 * @param tl_config 
 */
static UCS_CLASS_INIT_FUNC(uct_sci_iface_t, uct_md_h md, uct_worker_h worker,
                           const uct_iface_params_t *params,
                           const uct_iface_config_t *tl_config)
{
    unsigned int trash = getpid();
    unsigned int nodeID;
    unsigned int adapterID = 0;
    unsigned int flags = 0;
    ssize_t i = 0;
    //size_t alignment, align_offset;
    //ucs_status_t status;
    sci_error_t sci_error;
    unsigned dma_seg_id;
    sci_cb_data_interrupt_t callback = conn_handler;
    uct_sci_iface_config_t* config = ucs_derived_of(tl_config, uct_sci_iface_config_t); 
    uct_sci_md_t * sci_md = ucs_derived_of(md, uct_sci_md_t);

    UCT_CHECK_PARAM(params->field_mask & UCT_IFACE_PARAM_FIELD_OPEN_MODE,
                    "UCT_IFACE_PARAM_FIELD_OPEN_MODE is not defined");
    
    if (!(params->open_mode & UCT_IFACE_OPEN_MODE_DEVICE)) {
        ucs_error("only UCT_IFACE_OPEN_MODE_DEVICE is supported");
        return UCS_ERR_UNSUPPORTED;
    }


    if (ucs_derived_of(worker, uct_priv_worker_t)->thread_mode == UCS_THREAD_MODE_MULTI) {
        ucs_error("SCI transport does not support multi-threaded worker");
        return UCS_ERR_INVALID_PARAM;
    }

     /* 
        Lock usage taken from here.
        https://www.thegeekstuff.com/2012/05/c-mutex-examples/
     */
    
    if (pthread_mutex_init(&self->lock, NULL) != 0) {
        printf("\n mutex init failed\n");
        return UCS_ERR_NO_RESOURCE;
    }



    UCS_CLASS_CALL_SUPER_INIT(
            uct_base_iface_t, &uct_sci_iface_ops, &uct_base_iface_internal_ops,
            md, worker, params,
            tl_config UCS_STATS_ARG(
                    (params->field_mask & UCT_IFACE_PARAM_FIELD_STATS_ROOT) ?
                            params->stats_root :
                            NULL) UCS_STATS_ARG(UCT_SCI_NAME));
    


    //---------- IFACE sci --------------------------
    SCIGetLocalNodeId(adapterID, &nodeID, flags, &sci_error);

    if (sci_error != SCI_ERR_OK) { 
        printf("SCI_IFACE_INIT: %s\n", SCIGetErrorString(sci_error));
    } 

    DEBUG_PRINT("CONFIG\n\tSEND_SIZE: %zd \n\tMAX_EPS: %u\n", config->send_size, config->max_eps);
    

    self->device_addr = nodeID;
    self->segment_id  = ucs_generate_uuid(trash);
    self->ctl_id      = ucs_generate_uuid(trash);
    self->send_size   = config->send_size; //this is probbably arbitrary, and could be higher. 2^16 was just selected for looks
    self->eps         = 0;
    self->max_eps     = MIN(SCI_MAX_EPS, config->max_eps);
    self->connections = 0;
    self->queue_size  = config->queue_size;

    SCIOpen(&self->vdev_ep, 0, &sci_error);

    if (sci_error != SCI_ERR_OK) { 
        printf("SCI_OPEN_EP_VDEVS: %s\n", SCIGetErrorString(sci_error));
        return UCS_ERR_NO_RESOURCE;
    }

    SCIOpen(&self->vdev_ctl, 0, &sci_error);

    if (sci_error != SCI_ERR_OK) { 
        printf("SCI_OPEN_EP_VDEVS: %s\n", SCIGetErrorString(sci_error));
        return UCS_ERR_NO_RESOURCE;
    }     

    /*  recv segment    */

    SCICreateSegment(sci_md->sci_virtual_device, &self->local_segment, self->segment_id, self->send_size * self->max_eps * self->queue_size, NULL, NULL, 0, &sci_error);
    
    if (sci_error != SCI_ERR_OK) { 
            printf("SCI_CREATE_RECV_SEGMENT: %s\n", SCIGetErrorString(sci_error));
            return UCS_ERR_NO_RESOURCE;
    }

    SCIPrepareSegment(self->local_segment, 0, 0, &sci_error);
    
    if (sci_error != SCI_ERR_OK) { 
        printf("SCI_PREPARE_SEGMENT: %s\n", SCIGetErrorString(sci_error));
        return UCS_ERR_NO_RESOURCE;

    }

    SCISetSegmentAvailable(self->local_segment, 0, 0, &sci_error);
    
    if (sci_error != SCI_ERR_OK) { 
        printf("SCI_SET_AVAILABLE: %s\n", SCIGetErrorString(sci_error));
        return UCS_ERR_NO_RESOURCE;
    }

    /*    ctl segment    */
    
    SCICreateSegment(sci_md->sci_virtual_device, &self->ctl_segment, self->ctl_id, sizeof(sci_ctl_t) * self->max_eps, NULL, NULL, 0, &sci_error);
    if (sci_error != SCI_ERR_OK) { 
        printf("SCI_CREATE_CTL_SEGMENT: %s\n", SCIGetErrorString(sci_error));
        return UCS_ERR_NO_RESOURCE;
    }
    
    SCIPrepareSegment(self->ctl_segment, 0, 0, &sci_error); 
    if (sci_error != SCI_ERR_OK) { 
        printf("SCI_PREPARE_SEGMENT: %s\n", SCIGetErrorString(sci_error));
        return UCS_ERR_NO_RESOURCE;
    }

    SCISetSegmentAvailable(self->ctl_segment, 0, 0, &sci_error);
    if (sci_error != SCI_ERR_OK) { 
        printf("SCI_SET_AVAILABLE: %s\n", SCIGetErrorString(sci_error));
        return UCS_ERR_NO_RESOURCE;
    }

    self->ctls = (void*) SCIMapLocalSegment(self->ctl_segment, &self->ctl_map, 0, sizeof(sci_ctl_t) * self->max_eps, NULL, SCI_NO_FLAGS, &sci_error);

    if(sci_error != SCI_ERR_OK) {
        printf("DMA ctl segment: %s \n", SCIGetErrorString(sci_error));
        return UCS_ERR_NO_RESOURCE;
    } 


    self->tx_buf = (void*) SCIMapLocalSegment(self->local_segment, &self->local_map, 0, self->send_size * self->max_eps * self->queue_size, NULL,0, &sci_error);

    if (sci_error != SCI_ERR_OK) { 
            printf("SCI_MAP_LOCAL_SEG: %s\n", SCIGetErrorString(sci_error));
            return UCS_ERR_NO_RESOURCE;
    }

    for(i = 0; i < self->max_eps; i++) {
        self->sci_cds[i].status = 0;
        self->sci_cds[i].size = self->send_size * self->queue_size;
        self->sci_cds[i].offset = i * self->send_size * self->queue_size; 
        self->sci_cds[i].cd_buf = (void*) self->tx_buf + self->sci_cds[i].offset;
        self->sci_cds[i].packet = (sci_packet_t*) self->sci_cds[i].cd_buf;
        self->sci_cds[i].last_ack = 0;
    }

    /*----------------- DMA starts here ---------------*/
    /*TODO: add a reasonable number of max entries for SCICreateDMAQueue instead of 5.*/
    SCICreateDMAQueue(sci_md->sci_virtual_device, &self->dma_queue, 0, 10, SCI_NO_FLAGS, &sci_error);

    if(sci_error != SCI_ERR_OK) {
        printf("CreateDMAQueue: %s \n", SCIGetErrorString(sci_error));
        return UCS_ERR_NO_RESOURCE;
    } 

    dma_seg_id = ucs_generate_uuid(trash);
    SCICreateSegment(sci_md->sci_virtual_device, &self->dma_segment, dma_seg_id, self->send_size, NULL, NULL, SCI_NO_FLAGS, &sci_error);

    if(sci_error != SCI_ERR_OK) {
        printf("DMA create segment: %s \n", SCIGetErrorString(sci_error));
        return UCS_ERR_NO_RESOURCE;
    } 

    SCIPrepareSegment(self->dma_segment, 0, SCI_NO_FLAGS, &sci_error);

    if(sci_error != SCI_ERR_OK) {
        printf("DMA prepare segment: %s \n", SCIGetErrorString(sci_error));
        return UCS_ERR_NO_RESOURCE;
    } 

    self->dma_buf = SCIMapLocalSegment(self->dma_segment, &self->dma_map, 0, self->send_size, NULL, SCI_NO_FLAGS, &sci_error);

    if(sci_error != SCI_ERR_OK) {
        printf("DMA map segment: %s \n", SCIGetErrorString(sci_error));
        return UCS_ERR_NO_RESOURCE;
    } 



    /*------------------------- INTERRUPTS --------------------------------- */
    //TODO

    self->interruptNO = ucs_generate_uuid(trash);

    SCICreateDataInterrupt(sci_md->sci_virtual_device, &self->interrupt, 0, &self->interruptNO,  
                            callback, self, SCI_FLAG_USE_CALLBACK, &sci_error);


    if(sci_error != SCI_ERR_OK) {
        printf("SCI CREATE INTERRUPT: %s %d\n", SCIGetErrorString(sci_error), SCI_FLAG_USE_CALLBACK);
        return UCS_ERR_NO_RESOURCE;
    } 

    /*Need to find out how mpool works and how it is used by the underlying systems in ucx*/
    /*status = uct_iface_param_am_alignment(params, self->send_size, 0, 0,
                                          &alignment, &align_offset);


    if (status != UCS_OK) {
        printf("failed to init sci mpool\n");
        return status;
    }*/



    DEBUG_PRINT("iface_addr: %d dev_addr: %d segment_size %zd\n", self->interruptNO, self->device_addr, self->send_size);
    return UCS_OK;
}


/**
 * @brief Construct a new ucs class cleanup func object
 * 
 */

static UCS_CLASS_CLEANUP_FUNC(uct_sci_iface_t)
{
    /* 
        TODO: Add proper cleanup for iface, i.e free resources that were allocated on init. 
    */
    sci_error_t sci_error;
    
    DEBUG_PRINT("closed iface\n");

    uct_base_iface_progress_disable(&self->super.super,
                                    UCT_PROGRESS_SEND |
                                    UCT_PROGRESS_RECV);

    pthread_mutex_destroy(&self->lock);


    /* DMA */

    SCIUnmapSegment(self->dma_map, SCI_NO_FLAGS, &sci_error);
    SCIRemoveSegment(self->dma_segment, SCI_FLAG_FORCE_REMOVE, &sci_error);
    SCIRemoveDMAQueue(self->dma_queue, SCI_NO_FLAGS, &sci_error);

    if(sci_error != SCI_ERR_OK) {
        printf("IFACE CLOSE, Failed to remove dma queue: %s\n", SCIGetErrorString(sci_error));
    }

    // TODO: THIS!
    for(ssize_t i = 0; i < self->connections; i++) {
        self->sci_cds[i].status = 3;
        
        SCIUnmapSegment(self->sci_cds[i].ctl_map, 0, &sci_error);
    
        if (sci_error != SCI_ERR_OK) { 
        printf("SCI_UNMAP_SEGMENT: %s\n", SCIGetErrorString(sci_error));
        }

        SCIDisconnectSegment(self->sci_cds[i].ctl_segment, 0, &sci_error);

        if (sci_error != SCI_ERR_OK) { 
            printf("SCI_DISCONNECT_SEGMENT: %s\n", SCIGetErrorString(sci_error));
        }
    
    }


    /* RX  */

    SCIUnmapSegment(self->local_map, 0, &sci_error);

    SCISetSegmentUnavailable(self->local_segment,0,SCI_NO_FLAGS,&sci_error);

    if (sci_error != SCI_ERR_OK) { 
            printf("SCI_SET_RX_UNAVAILABLE: %s\n", SCIGetErrorString(sci_error));
    }

    SCIRemoveSegment(self->local_segment, SCI_FLAG_FORCE_REMOVE , &sci_error);

    if (sci_error != SCI_ERR_OK) { 
            printf("SCI_REMOVE_RX: %s\n", SCIGetErrorString(sci_error));
    }

    /* CTL */

    SCIUnmapSegment(self->ctl_map, 0, &sci_error);

    SCISetSegmentUnavailable(self->ctl_segment,0,SCI_NO_FLAGS,&sci_error);

    if (sci_error != SCI_ERR_OK) { 
            printf("SCI_SET_CTL_UNAVAILABLE: %s\n", SCIGetErrorString(sci_error));
    }

    SCIRemoveSegment(self->ctl_segment, SCI_NO_FLAGS , &sci_error);

    if (sci_error != SCI_ERR_OK) { 
            printf("SCI_REMOVE_CTL: %s\n", SCIGetErrorString(sci_error));
    }


    /* Closing device descriptors used for connections */
    SCIClose(self->vdev_ctl, SCI_NO_FLAGS, &sci_error);
    SCIClose(self->vdev_ep, SCI_NO_FLAGS, &sci_error);
}


/* block of macros defining the interface class */ 
UCS_CLASS_DEFINE(uct_sci_iface_t, uct_base_iface_t);

static UCS_CLASS_DEFINE_DELETE_FUNC(uct_sci_iface_t, uct_iface_t);

static UCS_CLASS_DEFINE_NEW_FUNC(uct_sci_iface_t, uct_iface_t, uct_md_h,
                                 uct_worker_h, const uct_iface_params_t*,
                                 const uct_iface_config_t*);


static ucs_status_t uct_sci_query_devices(uct_md_h md,
                                   uct_tl_device_resource_t **devices_p,
                                   unsigned *num_devices_p)
{
    ucs_status_t status = -1;
       
    status = uct_single_device_resource(md, UCT_SCI_NAME,
                                      UCT_DEVICE_TYPE_NET,
                                      UCS_SYS_DEVICE_ID_UNKNOWN, devices_p,
                                      num_devices_p);
    
    return status; 
}




static ucs_status_t uct_sci_md_query(uct_md_h md, uct_md_attr_t *attr)
{
    /* Dummy memory registration provided. No real memory handling exists */
    attr->cap.flags               = UCT_MD_FLAG_NEED_RKEY; /* TODO ignore rkey in rma/amo ops */
    attr->cap.max_alloc           = 0;
    attr->cap.reg_mem_types       = UCS_BIT(UCS_MEMORY_TYPE_HOST);
    attr->cap.alloc_mem_types     = 0;
    attr->cap.access_mem_types    = UCS_BIT(UCS_MEMORY_TYPE_HOST);
    attr->cap.detect_mem_types    = 0;
    attr->cap.max_reg             = ULONG_MAX;
    attr->rkey_packed_size        = 0;
    attr->reg_cost                = ucs_linear_func_make(0, 0);
    memset(&attr->local_cpus, 0xff, sizeof(attr->local_cpus));
    return UCS_OK;
}

static ucs_status_t uct_sci_mem_reg(uct_md_h md, void *address, size_t length,
                                     unsigned flags, uct_mem_h *memh_p)
{

    DEBUG_PRINT("Empty func\n");

    /* We have to emulate memory registration. Return dummy pointer */
    *memh_p = (void *) 0xdeadbeef;
    return UCS_OK;
}

static ucs_status_t uct_sci_mem_dereg(uct_md_h uct_md,
                                       const uct_md_mem_dereg_params_t *params)
{
    DEBUG_PRINT("Empty func\n");
    UCT_MD_MEM_DEREG_CHECK_PARAMS(params, 0);

    ucs_assert(params->memh == (void*)0xdeadbeef);

    return UCS_OK;
}


static void uct_sci_md_close(uct_md_h md) {
    //TODO: Maybe free up all segments or something lmao
    
    uct_sci_md_t * sci_md = ucs_derived_of(md, uct_sci_md_t);
    sci_error_t sci_error;
    DEBUG_PRINT("md closed\n");

    SCIClose(sci_md->sci_virtual_device, 0 , &sci_error);

    if (sci_error != SCI_ERR_OK)
        {
            /*NOTE*/
            printf("Error closing Virtual_Device error: %s \n", SCIGetErrorString(sci_error));
        }
    
    uct_sci_close();
}

static ucs_status_t uct_sci_md_open(uct_component_t *component, const char *md_name,
                                     const uct_md_config_t *config, uct_md_h *md_p)
{
    /* NOTE   */
    uct_sci_md_config_t *md_config = ucs_derived_of(config, uct_sci_md_config_t);

    static uct_md_ops_t md_ops = {
        .close              = uct_sci_md_close, 
        .query              = uct_sci_md_query,
        .mkey_pack          = ucs_empty_function_return_success,
        .mem_reg            = uct_sci_mem_reg,
        .mem_dereg          = uct_sci_mem_dereg,
        .detect_memory_type = ucs_empty_function_return_unsupported
    };

    //create sci memory domain struct
    static uct_sci_md_t md;
    sci_error_t errors;
    uct_sci_open();
    SCIOpen(&md.sci_virtual_device, 0, &errors);


    if (errors != SCI_ERR_OK)
        {
            printf("md_open error: %s/n", SCIGetErrorString(errors));
            return UCS_ERR_NO_RESOURCE;
        }
    

    md.super.ops       = &md_ops;
    md.super.component = &uct_sci_component;
    //md.super.component->name = "sci"
    md.num_devices     = md_config->num_devices;
    md.segment_id = 11;
    
    *md_p = &md.super;
    md_name = "sci";

    //uct_md_h = sci_md;

    DEBUG_PRINT("md opened \n");
    return UCS_OK;
}

int uct_sci_iface_is_reachable(const uct_iface_h tl_iface,
                                       const uct_device_addr_t *dev_addr,
                                       const uct_iface_addr_t *iface_addr)
{
   /*NOTE We have no good way to actually check if given address is reachable, so we just return 1*/
    
    #if DEBUG > 0
        uct_sci_iface_t* iface = ucs_derived_of(tl_iface, uct_sci_iface_t);
        uct_sci_device_addr_t* sci_dev_addr = (uct_sci_device_addr_t *) dev_addr;
        uct_sci_iface_addr_t*  sci_iface_addr = (uct_sci_iface_addr_t*) iface_addr;
        DEBUG_PRINT("FROM if_addr: %d dev_addr: %d  TO: iface_addr: %d dev_addr: %d \n",iface->interruptNO, iface->device_addr,  sci_iface_addr->segment_id, sci_dev_addr->node_id);
    #endif

    return 1;
}



ucs_status_t uct_sci_get_device_address(uct_iface_h iface, uct_device_addr_t *addr) {
    uct_sci_iface_t* sci_iface = ucs_derived_of(iface, uct_sci_iface_t);
    //uct_sci_md_t* md =  ucs_derived_of(sci_iface->super.md, uct_sci_md_t);  UNUSED
    uct_sci_device_addr_t* sci_addr = (uct_sci_device_addr_t *) addr;
    sci_addr->node_id = sci_iface->device_addr;
    DEBUG_PRINT("segment_id %d node_id %d\n", sci_iface->segment_id, sci_iface->device_addr);
    return UCS_OK;
}


/**
 * @brief returns the ID used for the connection interrupt
 *  
 */
ucs_status_t uct_sci_iface_get_address(uct_iface_h tl_iface,
                                               uct_iface_addr_t *addr)
{
    
    uct_sci_iface_t* iface = ucs_derived_of(tl_iface, uct_sci_iface_t);
    
    uct_sci_iface_addr_t* iface_addr = (uct_sci_iface_addr_t *) addr;
    
    iface_addr->segment_id = iface->interruptNO;
    
    DEBUG_PRINT("uct_iface_get_address()\n");
    return UCS_OK;
}


void uct_sci_iface_progress_enable(uct_iface_h iface, unsigned flags) {
    uct_base_iface_progress_enable(iface, flags);
    DEBUG_PRINT("Progress Enabled\n");
}

unsigned uct_sci_iface_progress(uct_iface_h tl_iface) {
    uct_sci_iface_t* iface = ucs_derived_of(tl_iface, uct_sci_iface_t);
    int       count  = 0;
    int found_message = 0;
    uint32_t  offset = 0;
    ucs_status_t status;
    sci_packet_t* packet;

    retry:

    for (size_t i = 0; i < iface->connections; i++) {
        sci_cd_t* cd = &iface->sci_cds[i];
        
        if(cd->status != 1) {
            continue;
        }

        offset = iface->send_size * ((cd->last_ack + 1) % iface->queue_size);
        packet = cd->cd_buf + offset; 
        
        if (packet->status != 1) {
            continue;
        }
        
        status = uct_iface_invoke_am(&iface->super, packet->am_id, cd->cd_buf + offset + sizeof(sci_packet_t), packet->length,0);
    
        if(status == UCS_INPROGRESS) {
            DEBUG_PRINT("UCS_IN_PROGRESS\n");
        }
        
        if(status == UCS_OK) {
            packet->status = 0;
            cd->ctl_buf->status = 0;
            cd->ctl_buf->ack = cd->last_ack + 1; 
            SCIFlush(NULL, SCI_FLAG_FLUSH_CPU_BUFFERS_ONLY);
            cd->last_ack++;
            found_message = 1;
        }

        else {
            printf("something went wrong %d\n", status);
        }

        ++count;
                
    }

    if (found_message) {
        found_message = 0;
        goto retry;
    }
    
    return count;
}

static ucs_status_t uct_sci_iface_query(uct_iface_h tl_iface, uct_iface_attr_t *attr)
{
    

    uct_sci_iface_t* iface = ucs_derived_of(tl_iface, uct_sci_iface_t);

    if (!iface_query_printed) {
        DEBUG_PRINT("iface querried\n");
    }

    /*  
        https://github.com/openucx/ucx/issues/6879
        According to this, we should call uct_base_iface_query()
    */

    uct_base_iface_query(ucs_derived_of(tl_iface, uct_base_iface_t), attr);   
    
    /* These flags advertises the functionality of our transport. We currently only support active message  */
    attr->cap.flags =   UCT_IFACE_FLAG_CONNECT_TO_IFACE | 
                        UCT_IFACE_FLAG_AM_SHORT         |
                        UCT_IFACE_FLAG_CB_SYNC          |
                        UCT_IFACE_FLAG_AM_BCOPY         |
                        UCT_IFACE_FLAG_AM_ZCOPY;
    attr->cap.event_flags  = 0;//UCT_IFACE_FLAG_EVENT_SEND_COMP |
                             //UCT_IFACE_FLAG_EVENT_RECV      |
                             //UCT_IFACE_FLAG_EVENT_ASYNC_CB ;
                             //UCT_IFACE_FLAG_EVENT_RECV_SIG;

    attr->device_addr_len  = sizeof(uct_sci_device_addr_t);
    attr->ep_addr_len      = sizeof(uct_sicsci_ep_addr_t);
    attr->iface_addr_len   = sizeof(uct_sci_iface_addr_t);
    
    //TODO: sane numbers, no lies.
    /* AM flags - TODO: these might need to be fine tuned at a later stage */
    attr->cap.am.max_short = iface->send_size;
    attr->cap.am.max_bcopy = 2048;
    attr->cap.am.min_zcopy = 32768;
    attr->cap.am.max_zcopy = iface->send_size;


    /*TODO Sane numbers, and not guesses for fun.*/
    attr->cap.am.max_iov   = 10;
    attr->cap.am.max_hdr   = 100;


    attr->latency                 = ucs_linear_func_make(0, 0);;
    attr->bandwidth.dedicated     = 10 * UCS_MBYTE;
    attr->bandwidth.shared        = 0;
    attr->overhead                = 10e-9;
    attr->priority                = 0;

    

    /*
        Iface gets queried multiple times so we had to disallow more than one debug print : )
    */
    if(!iface_query_printed) {
        DEBUG_PRINT("max_eps: %zd iface->attr->cap.flags: %ld event_flags-> %ld\n",attr->max_num_eps, attr->cap.flags, attr->cap.event_flags);
        iface_query_printed = 1;
    }
    return UCS_OK;
    //return UCS_ERR_NOT_IMPLEMENTED;
}


static ucs_status_t uct_sci_md_rkey_unpack(uct_component_t *component,
                                            const void *rkey_buffer, uct_rkey_t *rkey_p,
                                            void **handle_p)
{
    /**
     * Pseudo stub function for the key unpacking
     * Need rkey == 0 due to work with same process to reuse uct_base_[put|get|atomic]*
     */
    DEBUG_PRINT("uct_sci_md_rkey_unpack()");
    printf("uct_sci_md_rkey_unpack\n");
    *rkey_p   = 0;
    *handle_p = NULL;
    return UCS_OK;
}

/*
    TODO: Figure out what to change the commented lines to : )
*/
static uct_component_t uct_sci_component = {
    .query_md_resources = uct_md_query_single_md_resource, 
    .md_open            = uct_sci_md_open,
    .cm_open            = ucs_empty_function_return_unsupported, //UCS_CLASS_NEW_FUNC_NAME(uct_tcp_sockcm_t), //change me
    .rkey_unpack        = uct_sci_md_rkey_unpack, //change me
    .rkey_ptr           = ucs_empty_function_return_unsupported, //change me 
    .rkey_release       = ucs_empty_function_return_success, //change me
    .name               = UCT_SCI_NAME, //change me
    .md_config          = UCT_MD_DEFAULT_CONFIG_INITIALIZER,
    /*.md_config          = {
        .name           = "Self memory domain",
        .prefix         = "sci_",
        .table          = uct_sci_md_config_table,
        .size           = sizeof(uct_sci_md_config_t),
    },*/
    .tl_list            = UCT_COMPONENT_TL_LIST_INITIALIZER(&uct_sci_component),
    .flags              = 0, //UCT_COMPONENT_FLAG_CM,
    .md_vfs_init        = (uct_component_md_vfs_init_func_t)ucs_empty_function
};
UCT_COMPONENT_REGISTER(&uct_sci_component)


//the functions of the functionality that we support.
static uct_iface_ops_t uct_sci_iface_ops = {
     

    .ep_put_short             = uct_sci_ep_put_short,     // not implemented yet
    .ep_put_bcopy             = uct_sci_ep_put_bcopy,     // not implemented yet
    .ep_get_bcopy             = uct_sci_ep_get_bcopy,     // not implemented yet
    .ep_am_short              = uct_sci_ep_am_short,      // implemented
    .ep_am_short_iov          = uct_sci_ep_am_short_iov,  // not implemented yet
    .ep_am_bcopy              = uct_sci_ep_am_bcopy,      // implemented
    .ep_am_zcopy              = uct_sci_ep_am_zcopy,      // implemented
    .ep_atomic_cswap64        = uct_sci_ep_atomic_cswap64,// not implemented yet
    .ep_atomic64_post         = uct_sci_ep_atomic64_post, // not implemented yet
    .ep_atomic64_fetch        = uct_sci_ep_atomic64_fetch,// not implemented yet
    .ep_atomic_cswap32        = uct_sci_ep_atomic_cswap32,// not implemented yet
    .ep_atomic32_post         = uct_sci_ep_atomic32_post, // not implemented yet
    .ep_atomic32_fetch        = uct_sci_ep_atomic32_fetch,// not implemented yet
    .ep_flush                 = uct_base_ep_flush,        // maybe TODO, trenger vi å endre dette
    .ep_fence                 = uct_base_ep_fence,        // covered av uct base
    .ep_check                 = ucs_empty_function_return_success,        //covered 
    .ep_pending_add           = ucs_empty_function_return_busy,           //covered
    .ep_pending_purge         = ucs_empty_function,                       //covered
    .ep_create                = UCS_CLASS_NEW_FUNC_NAME(uct_sci_ep_t),    // implemented
    .ep_destroy               = UCS_CLASS_DELETE_FUNC_NAME(uct_sci_ep_t), // implemented
    .iface_flush              = uct_base_iface_flush,                     //covered av uct base
    .iface_fence              = uct_base_iface_fence,                     // covered av uct base
    .iface_progress_enable    = uct_sci_iface_progress_enable,            // covered
    .iface_progress_disable   = uct_base_iface_progress_disable,          // covered
    .iface_progress           = uct_sci_iface_progress,                   // implemented
    .iface_event_arm          = ucs_empty_function_return_success,        // covered
    .iface_close              = UCS_CLASS_DELETE_FUNC_NAME(uct_sci_iface_t), // implemented
    .iface_query              = uct_sci_iface_query,                         // implemented
    .iface_get_device_address = uct_sci_get_device_address,                  // implemented
    .iface_get_address        = uct_sci_iface_get_address,                   // implemented
    .iface_is_reachable       = uct_sci_iface_is_reachable                   // implemented
};


/**
 * @brief Construct a new uct tl define object
 *  component:
 *  tranport name
 *  device_query()
 *  iface type
 *  config prefix
 *  config table
 *  type of config table
 */
UCT_TL_DEFINE(&uct_sci_component, sci, uct_sci_query_devices, uct_sci_iface_t,
              UCT_SCI_CONFIG_PREFIX, uct_sci_iface_config_table, uct_sci_iface_config_t);
