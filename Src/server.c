#include "server.h"
#include "string.h"


static err_t srv_conn_accepted(void *arg, struct tcp_pcb *newpcb, err_t err);
//static void srv_conn_close(struct tcp_pcb *newpcb, struct tcp_echoserver_struct * eserv);
static err_t srv_conn_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void srv_conn_err(void *arg, err_t err);

struct tcp_pcb * initServer(uint16_t port)
{

    struct tcp_pcb *replTcpSoc = tcp_new();

    if(replTcpSoc == NULL)
        return NULL;

    if(tcp_bind(replTcpSoc,IP4_ADDR_ANY, port) != ERR_OK){
        tcp_abort(replTcpSoc);
        return NULL;
    }

    replTcpSoc = tcp_listen(replTcpSoc);

    tcp_accept(replTcpSoc, &srv_conn_accepted);

    return  replTcpSoc;
}


/* Static callbacks */

static err_t srv_conn_accepted(void *arg, struct tcp_pcb *newpcb, err_t err)
{

//    struct tcp_echoserver_struct * eserv =
//            (struct tcp_echoserver_struct *)
//                    mem_malloc(sizeof(struct tcp_echoserver_struct));

//    if(eserv != NULL)
//    {
//        eserv->state = ES_ACCEPTED;
//        eserv->pcb = newpcb;
//        eserv->retries = 0;
//        eserv->p = NULL;

//        tcp_arg(newpcb, eserv);
        tcp_recv(newpcb, srv_conn_recv);
        tcp_err(newpcb, srv_conn_err);


//    }
//    else{
//        srv_conn_close(newpcb, eserv);
//        return ERR_MEM;
//    }


    char *string = "{\"hop\": 0}";
    if(err == ERR_OK)
    {
        tcp_write(newpcb, string, strlen(string), 0);
        tcp_output(newpcb);
    }

    return err;

}

//static void srv_conn_close(struct tcp_pcb *newpcb, struct tcp_echoserver_struct * eserv)
//{

//}

static err_t srv_conn_recv(void *arg, struct tcp_pcb *newpcb, struct pbuf *p, err_t err)
{
    if(err == ERR_OK)
    {
        tcp_write(newpcb, p->payload, p->len, 0);
        tcp_output(newpcb);
    }
    return err;
}

static void srv_conn_err(void *arg, err_t err)
{

}

